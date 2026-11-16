#!/usr/bin/python3
"""Reproduce fixed-map movement/alignment -> restart -> realtime in ONE web process.

Run with run_bounded.py in domain78. Owns port10088 and its manager only.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request


def main():
    ws=Path(os.environ['GO2_WS']);root='http://127.0.0.1:10088'
    for port in (10088,11348):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
    def request(path,body=None):
        req=urllib.request.Request(root+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json','X-Go2':'1'})
        with urllib.request.urlopen(req,timeout=5) as response:return json.load(response)
    log=open(ws/'runs/mode_session_console.log','wb')
    proc=subprocess.Popen(['ros2','run','go2_web','web_server','--ros-args','-p','port:=10088','-p','use_sim_time:=true',
        '-p',f'workspace:={ws}','-p','launch_cmd:=ros2 launch go2_sim sim.launch.py gui:=false'],
        cwd=ws,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    snapshots=[];owned=set()
    def sample():
        s=request('/api/state');snapshots.append(s);owned.update(s.get('stack',{}).get('sim_processes',[]))
        if s['stack']['state']=='failed':raise RuntimeError(str(s['stack']))
        return s
    def wait_for(predicate,seconds):
        deadline=time.monotonic()+seconds;s=None
        while time.monotonic()<deadline:
            s=sample()
            if predicate(s):return s
            time.sleep(.12)
        raise RuntimeError('timeout '+json.dumps({k:s.get(k) for k in ('session_id','mode','pose','loc','alignment','realtime')},ensure_ascii=False))
    try:
        deadline=time.monotonic()+15
        while True:
            try:request('/api/state');break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.2)
        request('/api/stack/start',{'mode':'nav','map':'my_map','enable_motion':True})
        s=wait_for(lambda s:s['stack']['state']=='running' and s.get('aligned') and s.get('pose'),65)
        s=wait_for(lambda s:s.get('loc',{}).get('map_score') is not None,5)
        assert 0<=s['loc']['map_score']<=1,s['loc']
        print('CHECK actual fixed-map match score '+str(s['loc']['map_score']),flush=True)
        session=s['session_id'];start=s['pose']
        print('CHECK fixed session READY '+session,flush=True)
        goal={'x':start['x']+1.2,'y':start['y']}
        checked=request('/api/check',{'waypoints':[dict(name='session-check',**goal)]})
        assert not any(checked['problems']),checked
        request('/api/goal',goal)
        s=wait_for(lambda s:s.get('pose') and math.hypot(s['pose']['x']-start['x'],s['pose']['y']-start['y'])>.4 and any(
            e['name']=='ROUND_DONE' and 'arrived=1' in e['text'] for e in s.get('events',[])),45)
        s=wait_for(lambda s:s.get('task',{}).get('mode')=='idle',5)
        track=request('/api/track');assert track['session_id']==session and len(track['path'])>5,track
        moved=s['pose']
        print('CHECK fixed navigation moved '+json.dumps(moved),flush=True)
        # Deliberately change the map alignment while idle. The prior transform
        # must not leak into the new mapping origin, even when old TF stamps are future.
        reply=request('/api/initialpose',{'x':moved['x']+3.,'y':moved['y']+2.,'yaw':moved['th']})
        seen=set()
        def aligned(s):
            a=s['alignment'];seen.add(a['state'])
            return a['request_id']==reply['request_id'] and a['state'] in ('ok','weak') and s.get('pose')
        s=wait_for(aligned,10)
        assert 'pending' in seen,seen
        assert math.hypot(s['pose']['x'],s['pose']['y'])>2.,s['pose']
        old_pose=s['pose'];old_run=s['run']
        print('CHECK initial pose feedback '+str(sorted(seen))+' new map pose '+json.dumps(old_pose),flush=True)
        request('/api/stack/stop',{})
        s=wait_for(lambda s:s['stack']['state']=='idle',65)
        assert s['pose'] is None and not s['scan'] and not s['nav_path'] and s['loc'] is None
        request('/api/stack/start',{'mode':'realtime','map':'my_map','enable_motion':True})
        deadline=time.monotonic()+65;early_empty=0;origin_pose=None
        while time.monotonic()<deadline:
            s=sample()
            assert s['session_id']!=session
            if s['pose'] is None:early_empty+=1
            else:
                assert math.hypot(s['pose']['x'],s['pose']['y'])<.4,('old fixed-map coordinates survived',old_pose,s['pose'])
                origin_pose=origin_pose or s['pose']
            if s['stack']['state']=='running' and s.get('realtime') and s['realtime']['healthy'] and s['pose']:
                break
            time.sleep(.12)
        else:raise RuntimeError('new realtime did not become healthy')
        assert early_empty>0 and origin_pose
        assert s['run']!=old_run and s['localisation']=='legkilo'
        assert s['alignment']['request_id']==reply['request_id'] and s['realtime']['mode']=='manual'
        print(f'CHECK new session {s["session_id"]} had {early_empty} waiting samples; new origin '+json.dumps(origin_pose),flush=True)
        request('/api/stack/stop',{})
        s=wait_for(lambda s:s['stack']['state']=='idle',65)
        assert s['pose'] is None and s['realtime'] is None and s['live_map'] is None
        print('CHECK same console fixed->realtime restart and cleanup passed',flush=True)
    finally:
        (ws/'runs/mode_session_states.json').write_text(json.dumps(snapshots,ensure_ascii=False))
        if proc.poll() is None:
            try:request('/api/stack/stop',{})
            except OSError:pass
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=75)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for pid in owned:
            path=Path(f'/proc/{pid}/stat')
            if path.exists():assert path.read_text().rsplit(')',1)[1].split()[0]=='Z',f'owned process survives: {pid}'
        log.close()


if __name__=='__main__':main()
