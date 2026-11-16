#!/usr/bin/python3
"""Owned domain78 physics test: mot_recover really moves the dog (diagnosis of zero-displacement recover steps).

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
from geometry_msgs.msg import Twist
from go2_msgs.msg import RecoverCmd, SafeStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def main():
    ws = Path(os.environ['GO2_WS'])
    root = 'http://127.0.0.1:10088'
    for port in (10088, 11348):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))
    token = uuid.uuid4().hex[:8]
    mission = '_recover_probe_'+token
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
    console_log = open(ws/'runs/recover_motion_console.log', 'wb')
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
        scenario = os.environ.get('GO2_LOCAL_SCENARIO', 'open')
        log = []
        t0 = time.monotonic()
        def rec(name):
            return lambda m: log.append((round(time.monotonic()-t0, 2), name,
                                         [round(m.linear.x, 3), round(m.linear.y, 3), round(m.angular.z, 3)]
                                         if hasattr(m, 'linear') else
                                         dict(lim=m.limiting, reason=m.reason, dir=m.direction, gap=round(m.gap_m, 2) if m.gap_m == m.gap_m else None,
                                              cin=[round(v, 3) for v in m.cmd_in], out=[round(v, 3) for v in m.cmd_out])))
        keep = []
        for name in ('/mot_nav_cmd_vel', '/mot_recover_cmd_vel', '/mot_cmd_vel', '/safe_cmd_vel'):
            keep.append(node.create_subscription(Twist, name, rec(name), 10))
        keep.append(node.create_subscription(SafeStatus, '/safe_status', rec('/safe_status'), 10))
        pub = node.create_publisher(RecoverCmd, '/mot_recover_cmd', 10)
        yaw0 = p.get('yaw', 0.) or 0.
        if scenario == 'wallbox':
            c, s_ = math.cos(yaw0), math.sin(yaw0)
            for k, (fx, fy, sx, sy) in enumerate([(.48, 0., .2, .5), (0., -.55, 1.4, .1)]):
                req = SpawnEntity.Request()
                req.name = f'recover_probe_wall{k}_'+token
                req.xml = f"""<sdf version='1.6'><model name='w{k}'><static>true</static><link name='l'>
                  <collision name='c'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></collision>
                  <visual name='v'><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></visual>
                  </link></model></sdf>"""
                req.initial_pose.position.x = p['x']+c*fx-s_*fy
                req.initial_pose.position.y = p['y']+s_*fx+c*fy
                req.initial_pose.position.z = .4
                req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw0/2), math.cos(yaw0/2)
                assert spawn.wait_for_service(timeout_sec=3.)
                future = spawn.call_async(req)
                wait_for(lambda s, f=future: f.done(), 5)
                assert future.result().success
            end = time.monotonic()+5
            wait_for(lambda s: time.monotonic() > end, 8)
        results = []
        for action, value in (('backup', .3), ('turn', 1.5708)):
            before = dict(messages['pose'].pose.pose.position.__dict__) if False else None
            x0, y0 = messages['pose'].pose.pose.position.x, messages['pose'].pose.pose.position.y
            q = messages['pose'].pose.pose.orientation
            yaw_a = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            m = RecoverCmd(id='diag_'+action, action=action, value=value)
            pub.publish(m)
            mark = len(log)
            end = time.monotonic()+18
            wait_for(lambda s: time.monotonic() > end, 22)
            q = messages['pose'].pose.pose.orientation
            yaw_b = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            dx = messages['pose'].pose.pose.position.x-x0
            dy = messages['pose'].pose.pose.position.y-y0
            results.append(dict(action=action, dx=round(dx, 3), dy=round(dy, 3), dyaw=round(math.atan2(math.sin(yaw_b-yaw_a), math.cos(yaw_b-yaw_a)), 3),
                                cmds=len(log)-mark))
            print('CHECK '+json.dumps(results[-1]), flush=True)
        json.dump(dict(scenario=scenario, results=results, log=log), open(ws/f'runs/recover_motion_{scenario}.json', 'w'))
    finally:
        (ws/'runs/recover_motion_states.json').write_text(json.dumps(snapshots, ensure_ascii=False))
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
        print('CHECK owned stack/console cleanup complete', flush=True)


if __name__=='__main__':
    main()
