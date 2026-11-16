#!/usr/bin/python3
"""Bounded realtime integration probe. Run under run_bounded.py, in domain78.
Owns its port10088 console and session only; refuses a competing Gazebo master.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request
import uuid

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview-seconds',type=float,default=0)
    parser.add_argument('--slam-exit-probe',action='store_true')
    args=parser.parse_args()
    ws=Path(os.environ['GO2_WS']);root='http://127.0.0.1:10088'
    for port in (10088,11348):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
    client='probe-'+uuid.uuid4().hex
    def request(path,body=None):
        if path.startswith('/api/realtime') and body is not None:
            body=dict(body);body.setdefault('client',client)
        req=urllib.request.Request(root+path,data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type':'application/json','X-Go2':'1'})
        with urllib.request.urlopen(req,timeout=5) as response:return json.load(response)
    rclpy.init();node=rclpy.create_node('realtime_probe')
    maps={};inputs={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def remember(key,m):maps[key]=m
    subs=[node.create_subscription(OccupancyGrid,t,lambda m,k=t:remember(k,m),qos)
          for t in ('/map','/per_nav_map')]
    subs.append(node.create_subscription(String,'/sim/kilo_inputs',lambda m:inputs.update(json.loads(m.data)),10))
    def spin():
        for _ in range(4):rclpy.spin_once(node,timeout_sec=.01)
    log=open(ws/'runs/realtime_console_test.log','wb')
    proc=subprocess.Popen(['ros2','run','go2_web','web_server','--ros-args','-p','port:=10088',
        '-p','use_sim_time:=true','-p',f'workspace:={ws}',
        '-p','launch_cmd:=ros2 launch go2_sim sim.launch.py gui:=false'],
        cwd=ws,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    snapshots=[];last_beat=-1e9;owned=[];ready=False;bad_since=None
    def sample(heartbeat=True):
        nonlocal last_beat,owned,bad_since
        spin()
        if heartbeat and time.monotonic()-last_beat>.7:
            request('/api/realtime/heartbeat',{});last_beat=time.monotonic()
        if not heartbeat:
            request('/api/realtime/heartbeat',{'client':'observer'})
        s=request('/api/state');snapshots.append(s)
        owned=s.get('stack',{}).get('sim_processes') or owned
        if s['stack']['state']=='failed':raise RuntimeError(str(s['stack']))
        if ready and (not s.get('loc') or s['loc']['state']!='OK' or s.get('dev',{}).get('fault')):
            if bad_since is None:bad_since=time.monotonic();print('CHECK transient unhealthy',flush=True)
            if time.monotonic()-bad_since>3.:raise RuntimeError('unhealthy over3s')
        elif bad_since is not None:
            print('CHECK health recovered',flush=True);bad_since=None
        return s
    def wait_for(predicate,seconds,heartbeat=True):
        end=time.monotonic()+seconds;s=None
        while time.monotonic()<end:
            s=sample(heartbeat)
            if predicate(s):return s
            time.sleep(.15)
        raise RuntimeError('condition timed out: '+json.dumps({k:s.get(k) for k in ('mode','run','realtime','loc','nav','task')},ensure_ascii=False))
    try:
        end=time.monotonic()+15
        while True:
            try:request('/api/state');break
            except OSError:
                if time.monotonic()>end:raise
                time.sleep(.2)
        request('/api/stack/start',{'mode':'realtime','map':'my_map','enable_motion':True})
        s=wait_for(lambda s:s['stack']['state']=='running' and s.get('realtime',{}).get('healthy') and s.get('live_map'),65)
        ready=True
        print('CHECK realtime READY '+json.dumps(s['realtime'],ensure_ascii=False),flush=True)
        assert s['cloud_n']==0 and s['mode']=='realtime'
        assert s['localisation']=='legkilo' and s['realtime']['localisation']=='legkilo'
        end=time.monotonic()+5
        while len(maps)<2 and time.monotonic()<end:spin()
        assert len(maps)==2
        observed=np.asarray(maps['/map'].data,np.int8).reshape(600,600)
        nav=np.asarray(maps['/per_nav_map'].data,np.int8).reshape(600,600)
        assert (observed<0).sum()>observed.size*.5,'world was revealed instead of scanned'
        assert (nav[observed<0]>=65).all(),'unknown must be blocked'
        owners={t:node.count_publishers(t) for t in ('/map','/per_nav_map','/mot_nav_goal','/mot_cmd_vel','/safe_cmd_vel','/sim/cmd_vel','/loc_status','/loc_odom','/kilo_odom')}
        assert all(n==1 for n in owners.values()),owners
        assert node.get_publishers_info_by_topic('/loc_status')[0].node_name=='loc_pose'
        assert inputs.get('imu_count',0)>=50 and inputs.get('cloud_count',0)>0,inputs
        print('CHECK Leg-KILO real sensor input '+json.dumps(inputs),flush=True)
        print('CHECK unique control owners '+json.dumps(owners),flush=True)
        for endpoint in ('/api/live-map.png','/api/evidence.png'):
            with urllib.request.urlopen(root+endpoint,timeout=5) as r:
                im=cv2.imdecode(np.frombuffer(r.read(),np.uint8),cv2.IMREAD_UNCHANGED)
            assert im is not None
            if endpoint.endswith('evidence.png'):
                keep=im[:,:,3]>0;assert keep.any();assert np.all(im[keep,:3]==[212,188,0])
        request('/api/realtime',{'action':'point','x':14.,'y':14.})
        s=wait_for(lambda s:bool(s.get('realtime',{}).get('error')),3)
        assert s['realtime']['goal'] is None
        print('CHECK unseen point rejected',flush=True)
        # Find a known-free 1.2m point and ask the existing planner before using it.
        pose=s['pose'];target=None
        from go2_sim.realtime_logic import known_goal
        meta=maps['/map'].info
        origin=(meta.origin.position.x,meta.origin.position.y)
        for a in np.linspace(0,2*math.pi,24,endpoint=False):
            xy=(pose['x']+1.2*math.cos(a),pose['y']+1.2*math.sin(a))
            if known_goal(nav,.05,origin,xy,.51):
                check=request('/api/check',{'waypoints':[{'name':'realtime-test','x':xy[0],'y':xy[1]}]})
                if not any(check['problems']):target=xy;break
        assert target is not None,'no known-free reachable point from initial scan'
        request('/api/realtime',{'action':'point','x':target[0],'y':target[1],'yaw':.7})
        s=wait_for(lambda s:len(s.get('nav_path',[]))>1,5)
        print('CHECK active goal shows blue planned path',flush=True)
        s=wait_for(lambda s:s.get('realtime',{}).get('mode')=='complete',45)
        assert not s.get('nav_path'), 'completed goal retained planned path'
        assert abs(math.atan2(math.sin(s['pose']['th']-.7),math.cos(s['pose']['th']-.7)))<.35,s['pose']
        track=request('/api/track');assert track['session_id']==s['session_id'] and len(track['path'])>5,track
        print('CHECK dragged final heading + actual track',flush=True)
        moved=math.hypot(s['pose']['x']-pose['x'],s['pose']['y']-pose['y'])
        assert moved>.4
        print(f'CHECK known point arrived displacement={moved:.3f}',flush=True)
        known_before=s['realtime']['known_cells'];start=s['pose']
        request('/api/realtime',{'action':'explore'})
        s=wait_for(lambda s:s.get('realtime',{}).get('phase')=='navigating',15)
        print('CHECK frontier dispatched '+json.dumps(s['realtime']['goal']),flush=True)
        s=wait_for(lambda s:math.hypot(s['pose']['x']-start['x'],s['pose']['y']-start['y'])>.2,25)
        s=wait_for(lambda s:s.get('realtime',{}).get('known_cells',0)>known_before,10)
        print(f'CHECK exploration moved and map grew {known_before}->{s["realtime"]["known_cells"]}',flush=True)
        # Passive state polling is not a control heartbeat.
        s=wait_for(lambda s:s.get('realtime',{}).get('reason')=='web_lease_expired',6,heartbeat=False)
        assert s['realtime']['mode']=='paused' and s['realtime']['goal'] is None
        print('CHECK operator lease expiry cancels autonomy even with observer heartbeat',flush=True)
        request('/api/realtime',{'action':'manual'})
        s=wait_for(lambda s:s['realtime']['mode']=='manual' and s.get('task',{}).get('mode')=='idle',5)
        request('/api/teleop',{'vx':.1,'vy':0,'wz':0})
        request('/api/teleop',{'vx':0,'vy':0,'wz':0})
        request('/api/estop',{})
        s=wait_for(lambda s:s['realtime']['mode']=='paused',3)
        print('CHECK manual mode + teleop + estop',flush=True)
        request('/api/realtime',{'action':'save'})
        s=wait_for(lambda s:bool(s['realtime'].get('saved')),5)
        assert Path(s['realtime']['saved']).is_file()
        print('CHECK observed map saved '+s['realtime']['saved'],flush=True)
        end=time.monotonic()+args.preview_seconds
        while time.monotonic()<end:sample();time.sleep(.3)
        if args.slam_exit_probe:
            sample()
            request('/api/realtime',{'action':'point','x':target[0],'y':target[1],'yaw':.7})
            s=wait_for(lambda s:s.get('realtime',{}).get('phase')=='navigating',8)
            expected=(ws/'install/legkilo/lib/legkilo/legkilo_node').resolve()
            pids=[]
            for pid in owned:
                try:
                    argv=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
                    if argv and Path(os.fsdecode(argv[0])).resolve()==expected:
                        env=Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                        assert b'ROS_DOMAIN_ID=78' in env
                        pids.append(pid)
                except FileNotFoundError:pass
            assert len(pids)==1,pids
            ready=False
            os.kill(pids[0],signal.SIGINT)
            s=wait_for(lambda s:s['stack']['state'] in ('idle','failed'),65)
            spin()
            assert node.count_publishers('/loc_status')==0
            print('CHECK active-mission Leg-KILO exit stops whole owned stack; no ground-truth fallback',flush=True)
        else:
            ready=False
            request('/api/stack/stop',{})
            s=wait_for(lambda s:s['stack']['state']=='idle',65)
        print('CHECK all owned simulation processes stopped',flush=True)
    finally:
        (ws/'runs/realtime_test_states.json').write_text(json.dumps(snapshots,ensure_ascii=False))
        if proc.poll() is None:
            try:request('/api/stack/stop',{})
            except OSError:pass
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=75)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for pid in owned:
            p=Path(f'/proc/{pid}/stat')
            if p.exists():
                st=p.read_text().rsplit(')',1)[1].split()
                assert st[0]=='Z',f'owned process left alive: {pid}'
        node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
