#!/usr/bin/python3
"""Owned domain78 physics test: hand teleop before/after localisation DEGRADED (which stage drops it).

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
    mission = '_teleop_probe_'+token
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
    console_log = open(ws/'runs/teleop_degraded_console.log', 'wb')
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
        log = []
        t0 = time.monotonic()
        def twist(name):
            return lambda m: log.append((round(time.monotonic()-t0, 2), name, [round(m.linear.x, 3), round(m.linear.y, 3), round(m.angular.z, 3)]))
        keep = [node.create_subscription(Twist, '/dev_teleop_cmd_vel', twist('teleop'), 10),
                node.create_subscription(Twist, '/mot_cmd_vel', twist('mux'), 10),
                node.create_subscription(Twist, '/safe_cmd_vel', twist('safe'), 10),
                node.create_subscription(Twist, '/sim/cmd_vel', twist('sim_cmd'), 10),
                node.create_subscription(SafeStatus, '/safe_status', lambda m: log.append((round(time.monotonic()-t0, 2), 'safe_status',
                    dict(lim=m.limiting, reason=m.reason, out=[round(v, 3) for v in m.cmd_out]))), 10)]
        def pose():
            m = messages['pose'].pose.pose
            return m.position.x, m.position.y
        def drive(vx, wz, seconds):
            x0, y0 = pose()
            mark = len(log)
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                request('/api/teleop', dict(vx=vx, vy=0, wz=wz))
                wait_for(lambda s: True, 1)
                time.sleep(.08)
            request('/api/teleop', dict(vx=0, vy=0, wz=0))
            x1, y1 = pose()
            seg = log[mark:]
            def peak(name):
                vals = [abs(v[0]) + abs(v[2]) for _, n, v in seg if n == name and isinstance(v, list)]
                return max(vals) if vals else 0.
            return dict(moved_m=round(math.hypot(x1-x0, y1-y0), 3), teleop_peak=peak('teleop'), mux_peak=peak('mux'),
                        safe_peak=peak('safe'), sim_cmd_peak=peak('sim_cmd'),
                        loc_state=int(messages['loc'].state))
        res = {}
        res['ok_state'] = drive(.3, 0., 3.)
        print('CHECK before '+json.dumps(res['ok_state']), flush=True)
        # many unmapped obstacles around the dog (real boxes -> rays -> per_scan): drives the map score down
        px, py = pose()
        count = 0
        for ring, n in ((.8, 10), (1.3, 14)):
            for k in range(n):
                a_ = 2*math.pi*k/n
                req = SpawnEntity.Request()
                req.name = f'teleop_probe_box{count}_'+token
                req.xml = """<sdf version='1.6'><model name='b'><static>true</static><link name='l'>
                  <collision name='c'><geometry><box><size>0.3 0.3 0.8</size></box></geometry></collision>
                  <visual name='v'><geometry><box><size>0.3 0.3 0.8</size></box></geometry></visual>
                  </link></model></sdf>"""
                req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = px+ring*math.cos(a_), py+ring*math.sin(a_), .4
                req.initial_pose.orientation.w = 1.
                future = spawn.call_async(req)
                wait_for(lambda s, f=future: f.done(), 5)
                count += 1
        wait_for(lambda s: int(messages['loc'].state) != LocStatus.OK, 30)
        end = time.monotonic()+3
        wait_for(lambda s: time.monotonic() > end, 6)
        print('CHECK loc state after obstacles '+str(int(messages['loc'].state))+' (OK=%d)' % LocStatus.OK, flush=True)
        res['degraded'] = drive(.3, 0., 3.)
        print('CHECK degraded '+json.dumps(res['degraded']), flush=True)
        res['degraded_turn'] = drive(0., .4, 3.)
        print('CHECK degraded_turn '+json.dumps(res['degraded_turn']), flush=True)
        json.dump(dict(results=res, log=log), open(ws/'runs/teleop_degraded.json', 'w'))
    finally:
        (ws/'runs/teleop_degraded_states.json').write_text(json.dumps(snapshots, ensure_ascii=False))
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
