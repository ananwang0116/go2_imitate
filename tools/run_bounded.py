#!/usr/bin/python3
"""Owned process group + deadline + heartbeat, for build/test checks only."""
import os
import signal
import subprocess
import sys
import time

seconds = float(sys.argv[1])
stop_seconds = float(os.environ.get('GO2_WATCH_STOP_SECONDS', '45'))
command = sys.argv[2:]
p = subprocess.Popen(command, start_new_session=True)
print(f'WATCH start pid={p.pid} deadline_s={seconds} stop_budget_s={stop_seconds}', flush=True)
start = time.monotonic()
try:
    while p.poll() is None:
        if time.monotonic() - start >= seconds:
            raise TimeoutError('deadline exceeded')
        try:
            p.wait(timeout=min(25, max(.1, seconds-(time.monotonic()-start))))
        except subprocess.TimeoutExpired:
            print(f'WATCH alive pid={p.pid} elapsed_s={time.monotonic()-start:.0f}', flush=True)
except (TimeoutError, KeyboardInterrupt) as exc:
    print(f'WATCH stopping: {exc}', flush=True)
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=stop_seconds)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()
    sys.exit(124)
print(f'WATCH exit pid={p.pid} code={p.returncode} elapsed_s={time.monotonic()-start:.1f}', flush=True)
sys.exit(p.returncode)
