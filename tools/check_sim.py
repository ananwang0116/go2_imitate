#!/usr/bin/python3
"""Bounded end-to-end test of the web -> Gazebo -> original planner chain.

Run after sourcing go2_env.sh. Owns only its web process and that web's stack.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--single-cycle', action='store_true', help='navigation and web-stop check only')
    args = parser.parse_args()
    ws = Path(os.environ['GO2_WS'])
    root = 'http://127.0.0.1:10088'
    def request(path, body=None):
        req = urllib.request.Request(root+path, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'X-Go2': '1'})
        with urllib.request.urlopen(req, timeout=5) as response:
            return json.load(response)
    # Never reuse or stop a web server that this test did not start.
    import socket
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', 10088))
    log = open(ws/'runs/check_sim_web.log', 'wb')
    proc = subprocess.Popen(['ros2','run','go2_web','web_server','--ros-args','-p','port:=10088',
                             '-p','use_sim_time:=true','-p',f'workspace:={ws}',
                             '-p',f'launch_cmd:=ros2 launch go2_sim sim.launch.py gui:={str(args.gui).lower()}'],
                            cwd=ws, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stack_pid = None
    snapshots = []
    try:
        deadline = time.monotonic()+20
        while True:
            if proc.poll() is not None:
                raise RuntimeError('test web exited; inspect runs/check_sim_web.log')
            try:
                maps = request('/api/maps'); break
            except OSError:
                if time.monotonic() > deadline: raise
                time.sleep(.3)
        assert any(m['name']=='my_map' and not m['cloud'] for m in maps['maps']), maps
        stack_pid = request('/api/stack/start', {'mode':'nav','map':'my_map','enable_motion':True})['pid']
        print(f'CHECK web up; requested Gazebo pid={stack_pid} gui={args.gui}', flush=True)
        deadline = time.monotonic()+65
        while time.monotonic() < deadline:
            state=request('/api/state'); snapshots.append(state)
            if state.get('stack',{}).get('state')=='failed':
                raise RuntimeError(str(state['stack']))
            if state.get('pose') and state.get('loc',{}).get('state')=='OK' and state.get('aligned') and state.get('scan') and state.get('task'):
                break
            time.sleep(.5)
        else:
            raise RuntimeError('simulation not ready: '+json.dumps(state))
        assert state['cloud_n']==0
        deadline = time.monotonic()+10
        while not state.get('evidence') and time.monotonic()<deadline:
            time.sleep(.2); state=request('/api/state')
        assert state.get('evidence'), 'obstacle evidence map missing'
        import cv2
        import numpy as np
        with urllib.request.urlopen(root+'/api/evidence.png', timeout=5) as response:
            overlay=cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_UNCHANGED)
        assert overlay is not None and overlay.shape[2]==4
        occupied=overlay[:,:,3]>0
        assert occupied.any(), 'simulated lidar produced no confirmed obstacle cells'
        assert np.all(overlay[occupied,:3]==[212,188,0]), 'overlay must be cyan BGRA'
        print(f'CHECK cyan evidence cells={int(occupied.sum())} revision={state["evidence"]["rev"]}', flush=True)
        print('CHECK ready pose='+json.dumps(state['pose'])+' cloud_n=0', flush=True)
        # Choose a clear 1 m segment from the spawn in the supplied map.
        import cv2
        import numpy as np
        from go2_sim.world import map_data
        grid,res,origin=map_data(ws/'maps/my_map/map.yaml')
        clearance=cv2.distanceTransform(np.pad((grid==0).astype(np.uint8),1),cv2.DIST_L2,5)[1:-1,1:-1]*res
        x,y=state['pose']['x'],state['pose']['y']
        goal=None
        for angle in np.linspace(0,2*math.pi,24,endpoint=False):
            points=[(x+t*math.cos(angle),y+t*math.sin(angle)) for t in np.linspace(0,1.2,30)]
            if all(0<=int((py-origin[1])/res)<grid.shape[0] and 0<=int((px-origin[0])/res)<grid.shape[1]
                   and clearance[int((py-origin[1])/res),int((px-origin[0])/res)]>.5 for px,py in points):
                goal=points[-1];break
        if goal is None: raise RuntimeError('no clear local navigation test segment')
        request('/api/goal', {'x':goal[0],'y':goal[1]})
        print('CHECK navigation goal='+str(goal),flush=True)
        deadline=time.monotonic()+55
        moved=False
        while time.monotonic()<deadline:
            state=request('/api/state');snapshots.append(state)
            pose=state.get('pose')
            if pose and math.hypot(pose['x']-x,pose['y']-y)>.4:moved=True
            arrived = state.get('nav',{}).get('state')=='ARRIVED' or any(
                e.get('layer')=='task' and e.get('name')=='ROUND_DONE' and 'arrived=1' in e.get('text','')
                for e in state.get('events', []))
            if moved and arrived:break
            time.sleep(.5)
        else:raise RuntimeError('goal did not arrive: '+json.dumps(state))
        assert state['cloud_n']==0
        print('CHECK original planner arrived; simulated displacement='+str(math.hypot(pose['x']-x,pose['y']-y)),flush=True)
        run=ws/'runs'/state['run']
        stages=[json.loads(line) for line in (run/'startup.jsonl').read_text().splitlines()]
        expected=['MODEL_SPAWN','JOINT_BROADCASTER_LOADED','STAND_CONTROLLER_LOADED','EFFORT_CONTROLLER_LOADED','STAND_TRAJECTORY_SENT']
        assert [row['stage'] for row in stages]==expected and all(row['returncode']==0 for row in stages), stages
        import csv
        rows=[r for r in csv.DictReader((run/'body_motion.csv').open()) if int(r['gait_ready']) and int(r['gait_phase']) in (1,3)]
        assert {int(r['gait_phase']) for r in rows}=={1,3}, 'both diagonal swing phases must occur'
        assert all(float(r['gait_fault'])==0 for r in rows), 'gait fault during short navigation'
        planned={tuple(int(r['plan_'+leg]) for leg in ['FL','FR','RL','RR']) for r in rows}
        actual={tuple(int(r['contact_'+leg]) for leg in ['FL','FR','RL','RR']) for r in rows}
        assert {(0,1,1,0),(1,0,0,1)} <= planned, 'both planned diagonal support pairs required'
        assert {(0,1,1,0),(1,0,0,1)} <= actual, 'both actual diagonal contact pairs required'
        import yaml
        geometry=yaml.safe_load((ws/'src/go2_bringup/config/geometry.yaml').read_text())
        heights=[float(r['z'])-math.sin(float(r['pitch']))*geometry['mid360_x_m']+math.cos(float(r['pitch']))*math.sin(float(r['roll']))*geometry['mid360_y_m']+math.cos(float(r['pitch']))*math.cos(float(r['roll']))*geometry['mid360_z_m'] for r in rows]
        assert max(heights)-min(heights)>.0001, 'lidar must follow actual body motion'
        print('CHECK staged startup + articulated diagonal trot + body-driven lidar motion verified',flush=True)

        request('/api/stack/stop',{})
        deadline=time.monotonic()+65
        while time.monotonic()<deadline:
            state=request('/api/state')
            if state['stack']['state']=='idle' and not state['stack']['pid']:break
            time.sleep(.5)
        else:raise RuntimeError('owned simulation did not stop')
        assert not state['stack'].get('sim_processes'), state['stack']
        print('CHECK stopped through web API; all owned processes gone (server + GUI + ROS nodes)',flush=True)
        if args.single_cycle:
            return
        # Restart the same console. Closing its owned Gazebo client must also stop the server.
        stack_pid=request('/api/stack/start', {'mode':'nav','map':'my_map','enable_motion':True})['pid']
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            state=request('/api/state')
            if state.get('pose') and state.get('aligned') and state['stack']['state']=='running':break
            time.sleep(.5)
        else:raise RuntimeError('restart did not become ready')
        if args.gui:
            client=None
            for pid in state['stack']['sim_processes']:
                try:
                    argv=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
                    if Path(argv[0].decode()).name=='gzclient':client=pid;break
                except OSError:pass
            if client is None:raise RuntimeError('owned Gazebo GUI missing after restart')
            os.kill(client,signal.SIGTERM)
            deadline=time.monotonic()+65
            while time.monotonic()<deadline:
                state=request('/api/state')
                if not state['stack']['pid'] and not state['stack']['sim_processes']:break
                time.sleep(.3)
            else:raise RuntimeError('GUI exit left simulator processes running')
            print('CHECK restart + GUI exit stopped both Gazebo processes and all ROS nodes',flush=True)
        else:
            request('/api/stack/stop',{})
        # Console shutdown is a separate ownership path: start once more and close the console.
        if args.gui:
            stack_pid=request('/api/stack/start', {'mode':'nav','map':'my_map','enable_motion':True})['pid']
            deadline=time.monotonic()+60
            while time.monotonic()<deadline:
                state=request('/api/state')
                if state.get('pose') and state.get('aligned') and state['stack']['state']=='running':break
                time.sleep(.5)
            else:raise RuntimeError('console-shutdown test did not become ready')
            owned=state['stack']['sim_processes']
            os.killpg(proc.pid,signal.SIGINT)
            proc.wait(timeout=75)
            alive=[]
            for pid in owned:
                try:
                    stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                    if stat[0]!='Z':alive.append(pid)
                except OSError:pass
            assert not alive, f'console exit left owned processes: {alive}'
            print('CHECK console exit stopped server + GUI + ROS nodes',flush=True)
    finally:
        (ws/'runs/check_sim_states.json').write_text(json.dumps(snapshots,ensure_ascii=False,indent=2))
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=75)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        # The launch has its own session. Clean it only when its recorded ownership and domain match.
        if stack_pid:
            try:
                env=Path(f'/proc/{stack_pid}/environ').read_bytes()
                if b'ROS_DOMAIN_ID=78\0' in env:
                    os.killpg(stack_pid,signal.SIGINT)
            except (OSError,ProcessLookupError):pass
        log.close()


if __name__=='__main__':main()
