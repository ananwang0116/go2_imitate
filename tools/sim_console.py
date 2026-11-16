#!/usr/bin/python3
"""Restart only this workspace's previous console/simulation before a fresh console.

No process-name kills. Live socket ownership, workspace/domain, PID birth time
and persisted per-launch tokens identify exactly what can be stopped.
"""
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import urllib.request

from go2_web.stack_manager import StackManager


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, IndexError):
        return None


def console_owned(pid, ws):
    try:
        args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        env = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        expected = str(ws/'install/go2_web/lib/go2_web/web_server').encode()
        return (expected in args and f'GO2_WS={ws}'.encode() in env and b'ROS_DOMAIN_ID=78' in env)
    except OSError:
        return False


def listeners(port):
    inodes = set()
    for name in ['/proc/net/tcp', '/proc/net/tcp6']:
        for line in Path(name).read_text().splitlines()[1:]:
            fields = line.split()
            if fields[3] == '0A' and int(fields[1].split(':')[1], 16) == port:
                inodes.add(fields[9])
    owners = set()
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            for fd in (directory/'fd').iterdir():
                try:
                    target = os.readlink(fd)
                    if target.startswith('socket:[') and target[8:-1] in inodes:
                        owners.add(int(directory.name))
                except OSError:
                    continue
        except OSError:
            continue
    if inodes and not owners:
        raise RuntimeError(f'端口 {port} 已占用但无法核实进程归属；未停止任何未知进程。')
    return owners


def request(port, path, body=None):
    req = urllib.request.Request(f'http://127.0.0.1:{port}{path}',
                                 data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type':'application/json','X-Go2':'1'})
    with urllib.request.urlopen(req, timeout=2) as response:
        return json.load(response)


def stop_console(pid):
    birth = identity(pid)
    if birth is None:
        return
    print(f'正在停止上一轮 v4 网页（PID {pid}），等待它关闭仿真…', flush=True)
    for sig, seconds in [(signal.SIGINT, 75), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        if identity(pid) != birth:
            return
        os.kill(pid, sig)
        deadline = time.monotonic()+seconds
        last_notice = time.monotonic()
        while identity(pid) == birth and time.monotonic() < deadline:
            if time.monotonic()-last_notice >= 5:
                print('  仍在等待旧网页及其 Gazebo/ROS 进程收尾…', flush=True)
                last_notice = time.monotonic()
            time.sleep(.1)
    if identity(pid) == birth:
        raise RuntimeError(f'上一轮网页 PID {pid} 未能退出，本次不继续启动。')


def session_records(ws):
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    result = []
    for path in (ws/'runs/_sessions').glob('*.json'):
        data = json.loads(path.read_text())
        if data.get('workspace') != str(ws) or not re.fullmatch('[0-9a-f]{32}', data.get('token','')):
            raise RuntimeError(f'无法核实会话记录 {path.name}，本次不继续启动。')
        if data.get('boot_id') != boot:
            path.unlink()  # stale runtime ownership metadata, not user data
        else:
            result.append((path, data))
    return result


def cleanup_token(token):
    if not StackManager._members(token):
        return
    print('正在清理上一轮已记录的 Gazebo/ROS 残留…', flush=True)
    for sig, seconds in [(signal.SIGINT, 45), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        StackManager._signal_members(token, sig)
        deadline = time.monotonic()+seconds
        while StackManager._members(token) and time.monotonic() < deadline:
            time.sleep(.1)
        if not StackManager._members(token):
            return
    raise RuntimeError('上一轮仿真残留未能退出，本次不继续启动。')


def prepare(ws, port=10087):
    records = session_records(ws)
    owners = listeners(port)
    for pid in owners:
        if not console_owned(pid, ws):
            raise RuntimeError(f'端口 {port} 被其他程序占用（PID {pid}）；没有结束它。')
    # Capture the current manager's token even for consoles predating persisted records.
    live_tokens = set()
    if owners:
        try:
            state = request(port, '/api/state')
            if state.get('simulation'):
                for pid in state.get('stack',{}).get('sim_processes',[]):
                    try:
                        entries = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                        for entry in entries:
                            if entry.startswith(b'GO2_SIM_SESSION='):
                                token = entry.split(b'=',1)[1].decode()
                                if re.fullmatch('[0-9a-f]{32}', token): live_tokens.add(token)
                    except OSError:
                        continue
                if state.get('stack',{}).get('pid'):
                    request(port, '/api/stack/stop', {})
        except (OSError, ValueError):
            print('旧网页未正常响应，将按已核实的 PID 和会话归属收尾。', flush=True)
    for pid in owners:
        stop_console(pid)
    # Include a launch that was created while the old console was shutting down.
    records = session_records(ws)
    for path, data in records:
        pid = data.get('web_pid')
        if pid and identity(pid) == data.get('web_start') and console_owned(pid, ws):
            stop_console(pid)
        cleanup_token(data['token'])
        if StackManager._members(data['token']):
            raise RuntimeError('会话尚未完全退出，拒绝继续启动。')
        path.unlink(missing_ok=True)
    for token in live_tokens:
        cleanup_token(token)
    if listeners(port):
        raise RuntimeError(f'端口 {port} 尚未释放，拒绝启动重复网页。')
    # An untracked master belongs to an unknown launch; never silently attach to it.
    with socket.socket() as sock:
        sock.settimeout(.5)
        if sock.connect_ex(('127.0.0.1',11348)) == 0:
            raise RuntimeError('Gazebo 11348 端口仍占用且不属于已记录会话；未结束未知进程，本次不启动。')
    print('上一轮网页、Gazebo 服务端/窗口及已记录 ROS 会话均已退出。', flush=True)


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    ws = Path(os.environ['GO2_WS']).resolve()
    if ws != Path('/home/an/go2_v4') or os.environ.get('ROS_DOMAIN_ID') != '78':
        raise RuntimeError('仅用于 go2_v4 的独立仿真环境')
    (ws/'runs').mkdir(exist_ok=True)
    lock = open(ws/'runs/.web_start.lock','w')
    deadline = time.monotonic()+10
    while True:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB); break
        except BlockingIOError:
            if time.monotonic() >= deadline:
                raise RuntimeError('另一个启动/清理流程正在运行，请稍后重试。')
            time.sleep(.1)
    proc = None
    try:
        prepare(ws)
        executable = ws/'install/go2_web/lib/go2_web/web_server'
        cmd = [str(executable),'--ros-args','-p','port:=10087','-p','use_sim_time:=true',
               '-p',f'workspace:={ws}','-p','offline:=true',
               '-p','launch_cmd:=ros2 launch go2_sim sim.launch.py']
        proc = subprocess.Popen(cmd, cwd=ws, start_new_session=True)
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            if proc.poll() is not None:
                raise RuntimeError(f'新网页启动失败，退出码 {proc.returncode}')
            try:
                state=request(10087,'/api/state')
                if state.get('simulation') and console_owned(proc.pid,ws): break
            except (OSError,ValueError):
                pass
            time.sleep(.2)
        else:
            raise RuntimeError('新网页 15 秒内未就绪')
        fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
        print('新一轮网页已就绪：http://127.0.0.1:10087/',flush=True)
        print('选择地图并打开 Gazebo；再次运行此脚本会先完整停止这一轮。Ctrl-C 关闭本轮网页和仿真。',flush=True)
        return proc.wait()
    finally:
        if not lock.closed: lock.close()
        if proc is not None and proc.poll() is None:
            os.kill(proc.pid,signal.SIGINT)
            try: proc.wait(timeout=75)
            except subprocess.TimeoutExpired:
                os.kill(proc.pid,signal.SIGKILL);proc.wait()


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'启动失败：{exc}',file=sys.stderr,flush=True)
        raise SystemExit(1)
