#!/usr/bin/python3
"""Owned domain78 physics test for the single-point policy (2026-11-12).

1. A clicked goal that can never be planned (outside the map -> mot_nav goal_blocked before any path existed): no special case and
   no popup; checked alternative landing spots all fail, SKIP (goal_blocked_recovery_exhausted), the one ROUND_RETRY, SKIP again.
2. A normal nearby goal still arrives (no regression of the changed single-point policy).
Starts its own web console on 10088, stops only what it started. Run under run_bounded.py (GO2_WATCH_STOP_SECONDS=85, >=300 s).
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request


def main():
    ws = Path(os.environ['GO2_WS'])
    root = 'http://127.0.0.1:10088'
    for port in (10088, 11348):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))

    def request(path, body=None, timeout=5):
        req = urllib.request.Request(root+path, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'X-Go2': '1'})
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)
    console_log = open(ws/'runs/single_goal_console.log', 'wb')
    proc = subprocess.Popen(['ros2', 'run', 'go2_web', 'web_server', '--ros-args', '-p', 'port:=10088',
                             '-p', 'use_sim_time:=true', '-p', f'workspace:={ws}',
                             '-p', 'launch_cmd:=ros2 launch go2_sim sim.launch.py gui:=false'],
                            cwd=ws, stdout=console_log, stderr=subprocess.STDOUT, start_new_session=True)
    owned, run = set(), None

    def sample():
        if proc.poll() is not None:
            raise RuntimeError('owned web process exited')
        s = request('/api/state')
        owned.update(s.get('stack', {}).get('sim_processes', []))
        if s['stack']['state'] == 'failed':
            raise RuntimeError(str(s['stack']))
        return s

    def wait_for(predicate, seconds, what):
        end = time.monotonic()+seconds
        s = None
        while time.monotonic() < end:
            s = sample()
            if predicate(s):
                return s
            time.sleep(.2)
        raise RuntimeError('deadline: '+what+' '+json.dumps(s.get('task') if s else None, ensure_ascii=False))

    def events():
        return (run/'events.log').read_text() if run and (run/'events.log').exists() else ''
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
        s = wait_for(lambda s: s['stack']['state'] == 'running' and s.get('aligned') and s.get('pose'), 65, 'ready')
        run = Path(s['run'])
        if not run.is_absolute():
            run = ws/'runs'/run
        print('CHECK READY '+str(run), flush=True)
        request('/api/goal', {'x': 60., 'y': 0.})
        # 2026-11-12: no start-type special case and no popup: goal_blocked -> checked alternative landing spots (all outside the
        # map here, so none passes) -> the usual SKIP + the one ROUND_RETRY.
        s = wait_for(lambda s: events().count('SKIP wp=1') >= 2, 40, 'SKIP x2 (first try + the one retry)')
        text = events()
        assert 'reason=goal_blocked_recovery_exhausted' in text and 'ROUND_RETRY' in text
        assert 'GOAL_REJECT' not in text and 'goal_blocked_at_start' not in text
        assert not any('换一个点' in a['text'] for a in s.get('alerts', [])), 'no popup for goal_blocked'
        print('CHECK unplannable goal: checked alternatives fail, skipped, one retry, no popup', flush=True)
        time.sleep(1.)
        request('/api/goal', {'x': -0.934, 'y': 0.152})
        wait_for(lambda s: 'ARRIVED wp=1 name=goal' in events(), 60, 'ARRIVED')
        print('CHECK nearby single goal still arrives', flush=True)
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
        print('CHECK owned stack/console cleanup complete', flush=True)


if __name__ == '__main__':
    main()
