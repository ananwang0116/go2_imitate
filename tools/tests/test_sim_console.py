import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

import pytest

SPEC = importlib.util.spec_from_file_location('sim_console', Path(__file__).resolve().parents[1]/'sim_console.py')
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def server(tmp_path, owned=True):
    # IPv4 bind(0) may select a port already used on IPv6 (e.g. Gazebo's
    # dynamic listeners). The guard intentionally checks both families.
    for _ in range(32):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
            if not guard.listeners(port):break
    else:raise AssertionError('no isolated test listener port found')
    path=tmp_path/'install/go2_web/lib/go2_web/web_server' if owned else tmp_path/'foreign.py'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text('''from http.server import BaseHTTPRequestHandler,HTTPServer
import json,sys
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  data=json.dumps({'simulation':True,'stack':{'pid':0,'sim_processes':[]}}).encode()
  self.send_response(200);self.end_headers();self.wfile.write(data)
 def log_message(self,*args):pass
HTTPServer(('127.0.0.1',int(sys.argv[1])),Handler).serve_forever()
''')
    env=dict(os.environ,GO2_WS=str(tmp_path),ROS_DOMAIN_ID='78')
    proc=subprocess.Popen([sys.executable,str(path),str(port)],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        if proc.pid in guard.listeners(port):return proc,port
        time.sleep(.05)
    proc.kill();proc.wait();raise AssertionError('test server did not start')


def dispose(proc):
    if proc.poll() is None:proc.kill()
    proc.wait()


def test_restarts_only_the_owned_listener(tmp_path,monkeypatch):
    proc,port=server(tmp_path)
    # Test port ownership independently of any real Gazebo master on this desktop.
    class NoMaster:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def settimeout(self,value):pass
        def connect_ex(self,address):return 111
    try:
        assert guard.console_owned(proc.pid,tmp_path)
        monkeypatch.setattr(guard.socket,'socket',lambda *a,**k:NoMaster())
        # Avoid mocking urllib's sockets; stop_console exercises the same owned PID path.
        monkeypatch.setattr(guard,'request',lambda *a,**k:{'simulation':True,'stack':{'pid':0,'sim_processes':[]}})
        guard.prepare(tmp_path,port)
        proc.wait(timeout=3)
        assert not guard.listeners(port)
    finally:dispose(proc)


def test_foreign_listener_is_preserved(tmp_path):
    proc,port=server(tmp_path,owned=False)
    try:
        with pytest.raises(RuntimeError,match='其他程序'):
            guard.prepare(tmp_path,port)
        assert proc.poll() is None
    finally:dispose(proc)


def test_orphan_session_cleanup_does_not_signal_unrelated_process(tmp_path):
    token=uuid.uuid4().hex
    env=dict(os.environ,GO2_SIM_SESSION=token)
    owned=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],env=env,start_new_session=True)
    other=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True)
    try:
        guard.cleanup_token(token)
        owned.wait(timeout=3)
        assert other.poll() is None
    finally:
        dispose(owned);dispose(other)


def test_record_owner_pid_reuse_is_not_trusted(tmp_path,monkeypatch):
    proc,port=server(tmp_path)
    records=tmp_path/'runs/_sessions';records.mkdir(parents=True)
    data={'workspace':str(tmp_path),'token':uuid.uuid4().hex,
          'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
          'web_pid':proc.pid,'web_start':'wrong-birth-time'}
    (records/'session.json').write_text(json.dumps(data))
    class NoMaster:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def settimeout(self,value):pass
        def connect_ex(self,address):return 111
    try:
        monkeypatch.setattr(guard.socket,'socket',lambda *a,**k:NoMaster())
        guard.prepare(tmp_path,0)
        assert proc.poll() is None  # recorded PID matches, but birth time does not: never signal it
    finally:dispose(proc)
