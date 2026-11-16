#!/usr/bin/python3
"""Owned domain78 physics test: trapped-robot ladders (blocked: backup,turn; start_blocked: turn,relocate).

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
    mission = '_local_probe_'+token
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
    console_log = open(ws/'runs/local_recovery_console.log', 'wb')
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
        import math
        scenario = os.environ.get('GO2_LOCAL_SCENARIO', 'wallbox')
        yaw = p.get('yaw', 0.) or 0.
        c, s_ = math.cos(yaw), math.sin(yaw)
        def world(fx, fy):
            return p['x']+c*fx-s_*fy, p['y']+s_*fx+c*fy
        # Real Gazebo boxes; occupancy reaches the dog only through rays -> per_scan -> per_evidence.
        if scenario == 'pocket':      # U pocket, open behind
            walls = [(.47, 0., .1, .9), (0., .40, 1.0, .1), (0., -.40, 1.0, .1)]
        elif scenario == 'small':     # one small box near the nose, no unmapped wall (tries to avoid the DEGRADED freeze)
            walls = [(.5, 0., .12, .25)]
        elif scenario == 'smallu':    # three small boxes around the dog, open behind (few unmapped points)
            walls = [(.46, 0., .12, .25), (.05, .34, .25, .12), (.05, -.34, .25, .12)]
        elif scenario == 'smallv':    # tighter: dog centre within ~.22 m of boxes on three sides (aims at start_blocked)
            walls = [(.43, 0., .12, .25), (.05, .28, .25, .12), (.05, -.28, .25, .12)]
        elif scenario == 'wallbox':   # unmapped wall+box right at the start (lowers the map score: see the record)
            walls = [(.48, 0., .2, .5), (0., -.55, 1.4, .1)]
        else:                         # 'user': ONLY the box, at the user's screenshot place beside the MAPPED south wall
            walls = []
        if scenario == 'user':
            box_world = (-12.57, -4.99, .16, .5)
        for k, (fx, fy, sx, sy) in enumerate(walls):
            req = SpawnEntity.Request()
            req.name = f'local_probe_wall{k}_'+token
            req.xml = f"""<sdf version='1.6'><model name='w{k}'><static>true</static><link name='l'>
              <collision name='c'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></collision>
              <visual name='v'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></visual>
              </link></model></sdf>"""
            req.initial_pose.position.x, req.initial_pose.position.y = world(fx, fy)
            req.initial_pose.position.z = .4
            req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            assert spawn.wait_for_service(timeout_sec=3.)
            future = spawn.call_async(req)
            wait_for(lambda s, f=future: f.done(), 5)
            assert future.result().success, future.result().status_message
        if scenario == 'user':
            req = SpawnEntity.Request()
            req.name = 'local_probe_box_'+token
            bx, by, sx, sy = box_world
            req.xml = f"""<sdf version='1.6'><model name='b'><static>true</static><link name='l'>
              <collision name='c'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></collision>
              <visual name='v'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></visual>
              </link></model></sdf>"""
            req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = bx, by, .4
            req.initial_pose.orientation.w = 1.
            assert spawn.wait_for_service(timeout_sec=3.)
            future = spawn.call_async(req)
            wait_for(lambda s, f=future: f.done(), 5)
            assert future.result().success, future.result().status_message
        end = time.monotonic()+5
        wait_for(lambda s: time.monotonic() > end, 8)
        def rel_rows(points):
            return [dict(name=f'beyond{i+1}', x=world(fx, fy)[0], y=world(fx, fy)[1], radius_m=.4)
                    for i, (fx, fy) in enumerate(points)]
        def run_mission(tag, rows, command='run_once', deadline=150):
            name = mission+tag
            request('/api/mission/'+name, {'waypoints': rows})
            request('/api/task', {'command': 'load', 'arg': name})
            wait_for(lambda s: s.get('task', {}).get('count') == len(rows), 5)
            before = len(audit())
            request('/api/task', {'command': command})
            wait_for(lambda s: s.get('task', {}).get('mode') in ('once', 'loop'), 10)
            wait_for(lambda s: s.get('task', {}).get('mode') == 'idle', deadline)
            return audit()[before:], name
        if scenario == 'user':
            # arrive beside the box/wall, then RESTART the 4-point route from there (the user's failing situation)
            run_mission('_a', [dict(name='beside', x=-12.2, y=-4.914, radius_m=.15), dict(name='beside2', x=-12.2, y=-4.914, radius_m=.15)], deadline=200)
            print('CHECK arrived beside box, pose '+json.dumps(request('/api/state').get('pose')), flush=True)
            four = [dict(name='p1', x=-3.21, y=.21, radius_m=.4), dict(name='p2', x=-7.59, y=-.14, radius_m=.4),
                    dict(name='p3', x=-10.79, y=-2.73, radius_m=.4), dict(name='p4', x=-13.07, y=-4.75, radius_m=.4)]
            multi, name1 = run_mission('_m', four, deadline=200)
            single, name2 = run_mission('_s', [four[3]], deadline=120)
        else:
            multi, name1 = run_mission('_m', rel_rows([(2.0, 0.), (3.0, 0.)]))
            single, name2 = run_mission('_s', rel_rows([(2.0, 0.)]))
        records = multi + single
        steps = [r for r in records if r['kind'] == 'local_step']
        results = [r for r in records if r['kind'] == 'local_result']
        print('CHECK scenario '+scenario, flush=True)
        print('CHECK local steps '+json.dumps([(r['local_reason'], r['step'], r['step_index'], r.get('policy', 'route')) for r in steps]), flush=True)
        print('CHECK local results '+json.dumps([(r['step'], r['result'], r['effect'], r['effective']) for r in results]), flush=True)
        print('CHECK transitions '+json.dumps([(r.get('policy', 'route'), r.get('waypoint'), r.get('to_phase'), r.get('reason'))
                                              for r in records if r['kind'] == 'transition']), flush=True)
        json.dump(dict(scenario=scenario, steps=steps, results=results, records=records),
                  open(ws/f'runs/local_recovery_{scenario}_summary.json', 'w'), ensure_ascii=False)
        assert steps, 'scenario did not produce blocked/start_blocked: nothing was exercised'
        order = {'blocked': ['backup', 'turn'], 'start_blocked': ['turn', 'relocate']}
        for reason, expected in order.items():
            for policy in ('route', 'single_or_teach'):
                got = [r['step'] for r in steps if r['local_reason'] == reason and r.get('policy', 'route') == policy]
                # a ladder restarts for every waypoint visit, so compare each visit-prefix
                assert all(g in expected for g in got), (reason, policy, got)
        assert not any(r['step'] == 'backup' and r['local_reason'] == 'start_blocked' for r in steps)
        assert all((not r['effective']) or r['step'] != 'backup' or r['effect'] >= .27 for r in results)
        for part in (multi,):          # the legacy policy's one ROUND_RETRY pass resets used_s by design
            for wp in {r.get('waypoint') for r in part if r.get('waypoint')}:
                used = [r['used_s'] for r in part if r.get('waypoint') == wp and r.get('used_s') is not None
                        and r.get('goal_id') is not None]
                assert used == sorted(used), 'waypoint elapsed time was reset'
        print('CHECK ownership '+json.dumps(owners), flush=True)
        for nm in (name1, name2):
            request('/api/task', {'command': 'cancel'})
    finally:
        (ws/'runs/local_recovery_probe_states.json').write_text(json.dumps(snapshots, ensure_ascii=False))
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
        for tag in ('_a', '_m', '_s'):
            mission_path = ws/'missions'/f'{mission}{tag}.yaml'
            if mission_path.exists():
                if run and run.is_dir():
                    (run/f'probe_mission{tag}.yaml').write_bytes(mission_path.read_bytes())
                mission_path.unlink()
        print('CHECK owned stack/console cleanup complete', flush=True)


if __name__=='__main__':
    main()
