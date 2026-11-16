"""Manage an owned Gazebo launch, including its server, GUI and ROS child processes."""
import os
import signal
import subprocess
import threading
import time
import uuid
import json
from pathlib import Path

MODES = ('map', 'nav', 'realtime')
TAIL_LINES = 8


class StackManager:
    def __init__(self, ws, launch_cmd=None, env=None, kill_after_s=20.0):
        self.ws = Path(ws)
        self.launch_cmd = launch_cmd or ['ros2', 'launch', 'go2_sim', 'sim.launch.py']
        self.env = dict(env if env is not None else os.environ)
        self.kill_after_s = kill_after_s
        self.proc = None
        self.mode = self.map = ''
        self.enable_motion = False
        self.log = None
        self.started = 0.0
        self.stopping = False
        self.killed = False
        self.lock = threading.Lock()
        self.sim_session = ''
        self.sim_members = []

    @staticmethod
    def _members(token):
        """Identify only descendants carrying our per-launch ownership token, never by name."""
        found = []
        marker = f'GO2_SIM_SESSION={token}'.encode()
        for entry in Path('/proc').glob('[0-9]*'):
            try:
                if marker not in (entry / 'environ').read_bytes().split(b'\0'):
                    continue
                stat = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if stat[0] != 'Z':
                    found.append((int(entry.name), stat[19]))  # start time guards PID reuse
            except (OSError, IndexError):
                continue
        return found

    @classmethod
    def _signal_members(cls, token, sig):
        for pid, started in cls._members(token):
            try:
                stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if stat[19] == started:
                    os.kill(pid, sig)
            except (OSError, IndexError):
                pass

    def _supervise_sim(self, proc, token):
        while proc.poll() is None:
            if self.sim_session == token:
                self.sim_members = self._members(token)
            time.sleep(.3)
        # ros2 launch normally waits for its children. If its root exits unexpectedly,
        # still close this launch's GUI/server and nodes, including separate process groups.
        for sig, seconds in [(signal.SIGINT, 45), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
            members = self._members(token)
            if not members:
                break
            if sig == signal.SIGKILL:
                self.killed = True
            self._signal_members(token, sig)
            deadline = time.monotonic() + seconds
            while self._members(token) and time.monotonic() < deadline:
                time.sleep(.2)
        if self.sim_session == token:
            self.sim_members = self._members(token)

    def alive(self):
        return (self.proc is not None and self.proc.poll() is None) or bool(self.sim_members)

    def _save_session(self, launch_pid=None):
        sessions = self.ws / 'runs' / '_sessions'
        sessions.mkdir(parents=True, exist_ok=True)
        owner_stat = Path(f'/proc/{os.getpid()}/stat').read_text().rsplit(')', 1)[1].split()
        record = {'workspace': str(self.ws.resolve()), 'token': self.sim_session,
                  'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  'web_pid': os.getpid(), 'web_start': owner_stat[19], 'launch_pid': launch_pid}
        temporary = sessions / f'{self.sim_session}.tmp'
        temporary.write_text(json.dumps(record))
        temporary.replace(sessions / f'{self.sim_session}.json')

    def start(self, mode, map_name, enable_motion, offline=False):
        with self.lock:
            if self.alive():
                raise PermissionError('the stack is already running (stop it first)')
            if mode not in MODES:
                raise ValueError(f'mode {mode!r} must be one of {MODES}')
            if mode in ('nav', 'realtime') and not (self.ws / 'maps' / map_name / 'map.yaml').is_file():
                raise ValueError(f'map {map_name!r} not found')
            logdir = self.ws / 'runs' / '_web'
            logdir.mkdir(parents=True, exist_ok=True)
            self.log = logdir / f'launch_{time.strftime("%Y%m%d_%H%M%S")}_{mode}.log'
            cmd = self.launch_cmd + [f'mode:={mode}', f'enable_motion:={"true" if enable_motion else "false"}', 'web:=false']
            if mode in ('nav', 'realtime'):
                cmd.append(f'map:={map_name}')
            if offline:
                cmd.append('offline:=true')
            launch_env = dict(self.env)
            if 'go2_sim' in self.launch_cmd:
                self.sim_session = uuid.uuid4().hex
                launch_env['GO2_SIM_SESSION'] = self.sim_session
                self._save_session()  # durable ownership exists before any child can become orphaned
            with open(self.log, 'wb') as stream:
                self.proc = subprocess.Popen(cmd, cwd=self.ws, env=launch_env, stdin=subprocess.DEVNULL, stdout=stream,
                                             stderr=subprocess.STDOUT, start_new_session=True)
            self.mode, self.map, self.enable_motion = mode, map_name if mode in ('nav', 'realtime') else '', bool(enable_motion)
            self.started, self.stopping, self.killed = time.time(), False, False
            if 'go2_sim' in self.launch_cmd:
                try:
                    self._save_session(self.proc.pid)
                except OSError:
                    pass  # the pre-spawn ownership record is already sufficient for recovery
                threading.Thread(target=self._supervise_sim, args=(self.proc, self.sim_session), daemon=True).start()
                threading.Thread(target=self._watch_sim_start, args=(self.proc,), daemon=True).start()
            return self.proc.pid

    def _watch_sim_start(self, proc):
        deadline = time.monotonic() + 60
        while proc.poll() is None and time.monotonic() < deadline:
            if self.stopping or self.log_has('SIM_READY'):
                return
            time.sleep(.5)
        with self.lock:
            if self.proc is proc and proc.poll() is None and not self.stopping:
                with open(self.log, 'a') as stream:
                    stream.write('\nSIM_START_TIMEOUT: Gazebo did not become ready within 60 seconds\n')
                # stop() takes this lock itself; defer until after releasing it.
                timed_out = True
            else:
                timed_out = False
        if timed_out:
            self.stop()

    def stop(self):
        """SIGINT to the group now; a thread escalates to SIGKILL. Returns immediately."""
        with self.lock:
            if not self.alive():
                return False
            pgid = self.proc.pid
            if pgid == os.getpgrp():
                raise RuntimeError('refusing to signal our own process group')
            if self.stopping:
                return True
            self.stopping = True
            try:
                if self.sim_session:
                    os.kill(pgid, signal.SIGINT)  # launch forwards once to each child; avoid double Ctrl-C
                else:
                    os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                if self.sim_session:
                    self._signal_members(self.sim_session, signal.SIGINT)
        threading.Thread(target=self._escalate, args=(self.proc, pgid), daemon=True).start()
        return True

    def _escalate(self, proc, pgid):
        try:
            proc.wait(timeout=self.kill_after_s)
        except subprocess.TimeoutExpired:
            self.killed = True
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            if self.sim_session:
                self._signal_members(self.sim_session, signal.SIGKILL)

    def wait_stopped(self, timeout_s=30):
        """Keep the console alive until this launch's server, GUI and nodes are gone."""
        deadline = time.monotonic() + timeout_s
        while self.alive() and time.monotonic() < deadline:
            time.sleep(.1)
        if self.sim_session:
            remaining = self._members(self.sim_session)
            if remaining:
                self.killed = True
                self._signal_members(self.sim_session, signal.SIGKILL)
                deadline = time.monotonic() + 3
                while self._members(self.sim_session) and time.monotonic() < deadline:
                    time.sleep(.1)
            self.sim_members = self._members(self.sim_session)
        return not self.alive()

    def tail(self):
        if not self.log or not self.log.is_file():
            return []
        lines = self.log.read_text(errors='replace').splitlines()
        return [ln[:220] for ln in lines if ln.strip()][-TAIL_LINES:]

    def log_has(self, text):
        return bool(self.log and self.log.is_file() and text in self.log.read_text(errors='replace'))

    def status(self):
        alive = self.alive()
        if alive:
            state = 'stopping' if self.stopping else 'starting'
        elif self.proc is None:
            state = 'idle'
        elif self.stopping or self.proc.returncode == 0:
            state = 'idle'
        else:
            state = 'failed'
        return {'session_id': self.sim_session if alive else '', 'state': state, 'mode': self.mode if alive else '', 'map': self.map if alive else '',
                'enable_motion': self.enable_motion if alive else False, 'pid': self.proc.pid if alive else 0,
                'since': self.started, 'returncode': None if alive or self.proc is None else self.proc.returncode,
                'killed': self.killed, 'tail': self.tail() if (state != 'idle' or self.killed) else [],
                'log': str(self.log) if self.log else '',
                'sim_processes': [pid for pid, _ in self.sim_members]}

    def run_dir(self):
        """runs/<time>_<mode> made by this start (the newest one after it)."""
        found = sorted(p for p in (self.ws / 'runs').glob(f'*_{self.mode}') if p.stat().st_mtime >= self.started - 2) if self.mode else []
        return found[-1] if found else None
