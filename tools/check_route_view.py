#!/usr/bin/python3
"""Owned domain78 session for LOOKING at the multi-waypoint web display (later-leg preview, heading arrows).

Starts its own web console on 10088 (not the user's 10087), starts the navigation stack, loads a 5-point mission with headings and
runs it in loop mode, then holds until the file named by $GO2_VIEW_DONE exists (or GO2_VIEW_MAX_S, default 240 s) so a person or
the browser pane can inspect http://127.0.0.1:10088. Shuts down only the processes it started. Run under run_bounded.py with
GO2_WATCH_STOP_SECONDS=85 and an outer deadline of at least GO2_VIEW_MAX_S + 200 s.
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


POINTS = [(-3.39, .15, -2.987), (-6.25, -.07, -2.94), (-8.68, -.5, -1.76), (-6.29, -4.69, 1.45), (-12.54, -4.38, .98)]


def main():
    ws = Path(os.environ['GO2_WS'])
    root = 'http://127.0.0.1:10088'
    for port in (10088, 11348):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))
    done_file = Path(os.environ.get('GO2_VIEW_DONE', '/nonexistent'))
    hold_s = float(os.environ.get('GO2_VIEW_MAX_S', '240'))
    mission = '_route_view_'+uuid.uuid4().hex[:8]

    def request(path, body=None, timeout=5):
        req = urllib.request.Request(root+path, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'X-Go2': '1'})
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)
    console_log = open(ws/'runs/route_view_console.log', 'wb')
    proc = subprocess.Popen(['ros2', 'run', 'go2_web', 'web_server', '--ros-args', '-p', 'port:=10088',
                             '-p', 'use_sim_time:=true', '-p', f'workspace:={ws}',
                             '-p', 'launch_cmd:=ros2 launch go2_sim sim.launch.py gui:=false'],
                            cwd=ws, stdout=console_log, stderr=subprocess.STDOUT, start_new_session=True)
    owned = set()
    run = None

    def sample():
        if proc.poll() is not None:
            raise RuntimeError('owned web process exited')
        s = request('/api/state')
        owned.update(s.get('stack', {}).get('sim_processes', []))
        if s['stack']['state'] == 'failed':
            raise RuntimeError(str(s['stack']))
        return s

    def wait_for(predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            s = sample()
            if predicate(s):
                return s
            time.sleep(.2)
        raise RuntimeError('deadline')
    try:
        end = time.monotonic()+15
        while True:
            try:
                request('/api/state')
                break
            except OSError:
                if proc.poll() is not None or time.monotonic() > end:
                    raise
                time.sleep(.15)
        request('/api/stack/start', {'mode': 'nav', 'map': 'my_map', 'enable_motion': True})
        s = wait_for(lambda s: s['stack']['state'] == 'running' and s.get('aligned') and s.get('pose'), 65)
        run = Path(s['run'])
        print('VIEW READY '+str(run)+' url='+root, flush=True)
        rows = [dict(name=f'点{i+1}', x=x, y=y, yaw=yaw, radius_m=.4) for i, (x, y, yaw) in enumerate(POINTS)]
        request('/api/mission/'+mission, {'waypoints': rows})
        request('/api/task', {'command': 'load', 'arg': mission})
        wait_for(lambda s: s.get('task', {}).get('count') == len(rows), 5)
        request('/api/task', {'command': 'run_loop'})
        print('VIEW RUNNING loop with %d points; hold up to %.0f s (touch %s to finish)' % (len(rows), hold_s, done_file), flush=True)
        t0 = time.monotonic()
        last = 0.
        while time.monotonic()-t0 < hold_s and not done_file.exists():
            s = sample()
            if time.monotonic()-last >= 30.:
                last = time.monotonic()
                t = s.get('task') or {}
                print('VIEW alive %.0fs task=%s index=%s preview_legs=%d' % (
                    time.monotonic()-t0, t.get('mode'), t.get('index'), len(s.get('route_preview') or [])), flush=True)
            time.sleep(1.)
        print('VIEW hold over', flush=True)
    finally:
        if proc.poll() is None:
            try:
                request('/api/task', {'command': 'cancel'})
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
                assert path.read_text().rsplit(')', 1)[1].split()[0] == 'Z', f'owned process survives: {pid}'
        console_log.close()
        mission_path = ws/'missions'/f'{mission}.yaml'
        if mission_path.exists():
            if run and run.is_dir():
                (run/'view_mission.yaml').write_bytes(mission_path.read_bytes())
            mission_path.unlink()
        print('VIEW owned stack/console cleanup complete', flush=True)


if __name__ == '__main__':
    main()
