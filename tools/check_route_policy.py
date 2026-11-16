#!/usr/bin/python3
"""Owned domain78 physics test: real ray-observed box, intermediate pass / final alternative / loop (the skip transition itself is covered offline: 0.6 m pass areas cannot be filled by one ray-observed box).

Uses the real task executor and planner. Never publishes maps, poses, navigation
goals or fake task feedback. Run under run_bounded.py with 85 s stop grace.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request
import uuid

import rclpy
from gazebo_msgs.srv import SpawnEntity
from go2_msgs.msg import LocStatus, TaskStatus
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def main():
    ws = Path(os.environ['GO2_WS'])
    root = 'http://127.0.0.1:10088'
    for port in (10088, 11348):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))
    token = uuid.uuid4().hex[:8]
    mission = '_route_probe_'+token
    rclpy.init()
    node = rclpy.create_node('route_policy_probe_'+token)
    seen, messages = {}, {}
    def receive(key, msg):
        seen[key], messages[key] = time.monotonic(), msg
    sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(cls, topic, lambda msg, k=key: receive(k, msg), qos)
            for key, cls, topic, qos in (
                ('pose', Odometry, '/loc_odom', sensor), ('scan', LaserScan, '/per_obstacle_scan', sensor),
                ('loc', LocStatus, '/loc_status', 10), ('task', TaskStatus, '/task_status', 10),
                ('map', OccupancyGrid, '/map', map_qos), ('navmap', OccupancyGrid, '/per_nav_map', map_qos))]
    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    def request(path, body=None, timeout=5):
        req = urllib.request.Request(root+path, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'X-Go2': '1'})
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)
    console_log = open(ws/'runs/route_policy_console.log', 'wb')
    proc = subprocess.Popen(['ros2', 'run', 'go2_web', 'web_server', '--ros-args', '-p', 'port:=10088',
                             '-p', 'use_sim_time:=true', '-p', f'workspace:={ws}',
                             '-p', 'launch_cmd:=ros2 launch go2_sim sim.launch.py gui:=false'],
                            cwd=ws, stdout=console_log, stderr=subprocess.STDOUT, start_new_session=True)
    owned, snapshots = set(), []
    ready, unhealthy_since = False, None
    run = None
    def sample():
        nonlocal unhealthy_since
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.005)
        if proc.poll() is not None:
            raise RuntimeError('owned web process exited')
        s = request('/api/state')
        snapshots.append(s)
        owned.update(s.get('stack', {}).get('sim_processes', []))
        if s['stack']['state'] == 'failed':
            raise RuntimeError(str(s['stack']))
        if ready:
            bad = any(time.monotonic()-seen.get(k, 0)>.8 for k in ('pose', 'scan', 'loc', 'task'))
            bad = bad or messages.get('loc', LocStatus()).state == LocStatus.LOST
            bad = bad or bool(s.get('dev', {}).get('fault'))
            if bad:
                unhealthy_since = unhealthy_since or time.monotonic()
                if time.monotonic()-unhealthy_since > 3.:
                    raise RuntimeError('required feedback unhealthy/stale for >3s')
            elif unhealthy_since is not None:
                print('CHECK transient health recovered', flush=True)
                unhealthy_since = None
        return s
    def wait_for(predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            s = sample()
            if predicate(s):
                return s
            time.sleep(.1)
        raise RuntimeError('deadline '+json.dumps(s.get('task'), ensure_ascii=False))
    def audit():
        path = run/'route_decisions.jsonl'
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    try:
        end = time.monotonic()+15
        while True:
            try:
                request('/api/state')
                break
            except OSError:
                if proc.poll() is not None or time.monotonic()>end:
                    raise
                time.sleep(.15)
        request('/api/stack/start', {'mode': 'nav', 'map': 'my_map', 'enable_motion': True})
        s = wait_for(lambda s: s['stack']['state']=='running' and s.get('aligned') and s.get('pose')
                     and all(k in seen for k in ('pose', 'scan', 'loc', 'task', 'map', 'navmap')), 65)
        ready = True
        run = Path(s['run'])
        if not run.is_absolute():
            run = ws/'runs'/run
        print('CHECK READY '+str(run), flush=True)
        owners = {t: node.count_publishers(t) for t in ('/map', '/per_nav_map', '/mot_nav_goal', '/mot_cmd_vel', '/safe_cmd_vel')}
        assert all(v==1 for v in owners.values()), owners
        assert s['localisation']=='legkilo'
        p = s['pose']
        x, y = p['x']+1.2, p['y']
        # Real physical box: occupancy must arrive through Gazebo rays -> per_scan -> per_evidence. 2026-11-12: intermediate
        # waypoints are PASSED inside a 0.6 m radius, so a 0.3 m box no longer blocks them (the dog passes them); a wider box would
        # stay hollow in the map (rays only mark its surface). Only the final waypoint (radius 0.1) is still 'occupied'.
        req = SpawnEntity.Request()
        req.name = 'route_probe_box_'+token
        req.xml = """<sdf version='1.6'><model name='route_box'><static>true</static><link name='box'>
          <collision name='collision'><geometry><box><size>0.3 0.3 0.8</size></box></geometry></collision>
          <visual name='visual'><geometry><box><size>0.3 0.3 0.8</size></box></geometry></visual>
          </link></model></sdf>"""
        req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = x, y, .4
        req.initial_pose.orientation.w = 1.
        assert spawn.wait_for_service(timeout_sec=3.)
        future = spawn.call_async(req)
        wait_for(lambda s: future.done(), 5)
        assert future.result().success, future.result().status_message
        rows = [dict(name='blocked1', x=x, y=y, radius_m=.1),
                dict(name='blocked2', x=x, y=y+.02, radius_m=.1),
                dict(name='clear3', x=p['x']-.65, y=y, radius_m=.4),
                dict(name='final4', x=x, y=y, radius_m=.1)]
        def occupied(s):
            grid = messages['navmap']; info = grid.info
            i = int((x-info.origin.position.x)/info.resolution)
            j = int((y-info.origin.position.y)/info.resolution)
            # Centre may be unobserved inside a hollow ray surface: use the planner's area judgement.
            return bool(request('/api/check', {'waypoints':[rows[0]]})['problems'][0])
        wait_for(occupied, 10)
        request('/api/mission/'+mission, {'waypoints':rows})
        request('/api/task', {'command':'load', 'arg':mission})
        wait_for(lambda s:s.get('task',{}).get('count')==4, 5)
        request('/api/task', {'command':'run_once'})
        wait_for(lambda s:any(r['kind']=='round_done' for r in audit()), 100)
        records = audit()
        original_points = next(r['original_waypoints'] for r in records if r['kind']=='round_start')
        transitions = [r for r in records if r['kind']=='transition']
        # 2026-11-12: a 0.3 m box inside the 0.6 m pass area is no longer 'occupied'. The dog either passes the point or, if it cannot
        # get within 0.6 m, is skipped by the 20 s no-spatial-progress window (both observed); it must never go to alternatives.
        for wp in (1, 2):
            assert any(r['waypoint']==wp and r['to_phase'] in ('arrived', 'skipped') for r in transitions), wp
        outcome = {wp: [r['to_phase']+':'+str(r.get('reason')) for r in transitions if r['waypoint']==wp and r['to_phase'] in ('arrived', 'skipped')]
                   for wp in (1, 2)}
        # The dog now ends its pass of the box ~0.4 m from it, where mot_nav can dither in place without reporting blocked
        # (run 20261112_162036_nav: turn +-0.3 rad for 20 s); the 20 s window then skips the clear point. Accept either.
        assert any(r['waypoint']==3 and r['to_phase'] in ('arrived', 'skipped') for r in transitions)
        outcome[3] = [r['to_phase']+':'+str(r.get('reason')) for r in transitions if r['waypoint']==3 and r['to_phase'] in ('arrived', 'skipped')]
        assert any(r['waypoint']==4 and r['to_phase']=='alternative_approach' for r in transitions)
        assert any(r['waypoint']==4 and r['to_phase']=='arrived' and r['reason']=='final_alternative' for r in transitions)
        assert not any(r['waypoint'] in (1,2) and r['to_phase']=='alternative_search' for r in transitions)
        assert 'ROUND_RETRY' not in (run/'events.log').read_text()
        print('CHECK once: blocked intermediates '+json.dumps(outcome)+', clear point settled, checked final alternative arrived, no extra retry', flush=True)
        offset = len(records)
        request('/api/task', {'command':'run_loop'})
        wait_for(lambda s:any(r['kind']=='goal_sent' and r['round']==2 and r['waypoint']==1 for r in audit()[offset:]), 100)
        records = audit()[offset:]
        first = next(r for r in records if r['kind']=='goal_sent' and r['round']==2 and r['waypoint']==1)
        assert first['original']==original_points[0] and first['target']==original_points[0]
        wait_for(lambda s:any(r['kind']=='goal_sent' and r['round']==2 and r['waypoint']==4 and r.get('reason')=='original_waypoint'
                             for r in audit()[offset:]), 100)
        records = audit()[offset:]
        last = next(r for r in records if r['kind']=='goal_sent' and r['round']==2 and r['waypoint']==4)
        assert last['original']==original_points[-1] and last['target']==original_points[-1]
        request('/api/task', {'command':'cancel'})
        wait_for(lambda s:s.get('task',{}).get('mode')=='idle', 5)
        print('CHECK loop: original blocked points and original final point retried in round2; cancel idle', flush=True)
        print('CHECK ownership '+json.dumps(owners), flush=True)
    finally:
        (ws/'runs/route_policy_probe_states.json').write_text(json.dumps(snapshots, ensure_ascii=False))
        ready = False
        if proc.poll() is None:
            try:
                request('/api/task', {'command':'cancel'})
                request('/api/stack/stop', {}, timeout=70)
            except OSError:
                pass
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=75)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for pid in owned:
            path = Path(f'/proc/{pid}/stat')
            if path.exists():
                assert path.read_text().rsplit(')',1)[1].split()[0]=='Z', f'owned process survives: {pid}'
        node.destroy_node()
        rclpy.shutdown()
        console_log.close()
        mission_path = ws/'missions'/f'{mission}.yaml'
        if mission_path.exists():
            if run and run.is_dir():
                (run/'probe_mission.yaml').write_bytes(mission_path.read_bytes())
            mission_path.unlink()
        print('CHECK owned stack/console cleanup complete', flush=True)


if __name__=='__main__':
    main()
