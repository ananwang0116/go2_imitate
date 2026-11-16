"""Gazebo fixed-map or observed-only realtime navigation with the original planner."""
import json
import os
import fcntl
import re
import time
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from go2_bringup import run_dir
from go2_sim.world import build_world

_lock_handle = None


def start(context):
    global _lock_handle
    cfg = context.launch_configurations
    ws = Path(os.environ['GO2_WS']).resolve()
    if ws.name != 'go2_v4' or os.environ.get('ROS_DOMAIN_ID') != '78':
        raise RuntimeError('simulation requires go2_v4 workspace and isolated ROS_DOMAIN_ID=78')
    (ws / 'runs').mkdir(exist_ok=True)
    _lock_handle = open(ws / 'runs/.simulation.lock', 'w')
    try:
        fcntl.flock(_lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('another go2_v4 simulation is running; stop it from its owning web console')
    name = cfg['map']
    if not re.fullmatch(r'[0-9A-Za-z_\-一-鿿]{1,40}', name):
        raise ValueError('invalid map name')
    geometry = Path(get_package_share_directory('go2_bringup')) / 'config/geometry.yaml'
    speeds = geometry.parent / 'speeds.yaml'
    map_yaml = ws / 'maps' / name / 'map.yaml'
    if cfg['mode'] not in ('nav', 'realtime'):
        raise RuntimeError('simulation supports nav or realtime')
    realtime = cfg['mode'] == 'realtime'
    slam = cfg['localisation'] == 'legkilo'
    if cfg['localisation'] not in ('legkilo','ground_truth'):
        raise ValueError('localisation must be legkilo or ground_truth')
    run = run_dir.create(ws, cfg['mode'], dict(cfg, simulation='gazebo', localisation=cfg['localisation']), geometry, name)
    world = run / 'simulation.world'
    info = build_world(map_yaml, geometry, world)
    info['localisation']=cfg['localisation']
    (run / 'simulation.json').write_text(json.dumps(info, indent=2))
    EVIDENCE_TTL = {'dynamic_ttl_s': 5.0, 'static_ttl_s': 90.0, 'static_min_s': 3.0}     # obstacle lifetimes, see go2_per.evidence_logic
    def node(package, executable, extra=None):
        return Node(package=package, executable=executable, name=executable.replace('.py', ''), output='screen',
                    parameters=[{'use_sim_time': True}, *(extra or [])])
    sim_share = Path(get_package_share_directory('go2_sim'))
    robot_xml = Path(info['robot_description']).read_text()
    server = ExecuteProcess(cmd=['gzserver', '--pause', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                                 str(world), '--ros-args', '--params-file', str(sim_share/'config/gazebo.yaml'), '--'], output='screen')
    bridge = node('go2_sim', 'sim_bridge', [{'enable_motion': cfg['enable_motion'] == 'true','localisation':cfg['localisation']}])
    planner = node('go2_nav', 'mot_nav', [{'geometry_file': str(geometry), 'speeds_file': str(speeds)}])
    per_map = (node('go2_sim', 'sim_online_mapper') if realtime else
               node('go2_per', 'per_map', [{'yaml_filename': str(map_yaml)}]))
    scan = Node(package='go2_per', executable='per_scan', name='per_scan', output='screen',
                parameters=[{'use_sim_time': True}], remappings=[('/dev_lidar_cloud', '/sim/mid360_points')])
    evidence = (node('go2_sim', 'sim_realtime_evidence', [EVIDENCE_TTL]) if realtime else
                node('go2_per', 'per_evidence', [EVIDENCE_TTL]))
    explorer = node('go2_sim', 'sim_realtime',[{'localisation':cfg['localisation']}]) if realtime else None
    mux = node('go2_mot', 'mot_cmd_mux'); recover = node('go2_mot', 'mot_recover')
    safety = node('go2_safe', 'safe_gate', [{'mode': 'nav'}])
    task = node('go2_task', 'task_executor', [{'missions_dir': str(ws / 'missions')}])
    state = node('robot_state_publisher', 'robot_state_publisher', [{'robot_description':robot_xml,'publish_frequency':30.}])
    slam_nodes=[]
    if slam:
        config=run_dir.write_kilo_config(run,Path(get_package_share_directory('legkilo')),geometry)
        params=yaml.safe_load(config.read_text())
        params.update(lidar_topic='/sim/kilo_points',imu_topic='/sim/kilo_imu',redundancy=False,
                      lidar_type=1,time_scale=1,lidar_time_offset=0.,temp_result_save_folder='go2_sim_run')
        config.write_text(yaml.safe_dump(params,sort_keys=False))
        kilo=Node(package='legkilo',executable='legkilo_node',name='legkilo_node',output='screen',
                  parameters=[{'use_sim_time':True}],sigterm_timeout='40',sigkill_timeout='5',
                  arguments=['--config_file',str(config),'--stderrthreshold=1','--colorlogtostderr=false'])
        slam_nodes=[node('go2_sim','sim_slam_inputs'),kilo,node('go2_loc','loc_pose'),
                    node('go2_sim','sim_slam_frames',[{'mapping':realtime,'initial_x':info['spawn_x'],
                          'initial_y':info['spawn_y'],'initial_yaw':0.,'initial_floor_height_m':.35}])]
        if not realtime:slam_nodes.extend([node('go2_loc','loc_align'),node('go2_loc','loc_map_check')])
        started_s=time.time()
        def collect(event,context):
            result=run_dir.collect_kilo(run,Path(get_package_share_directory('legkilo')),started_s,temp_folder='go2_sim_run')
            (run/'kilo'/'collection_result.txt').write_text(str(result))
            return []
        slam_nodes.append(RegisterEventHandler(OnProcessExit(target_action=kilo,on_exit=collect)))
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', prefix='/usr/bin/python3', output='screen',
                 arguments=['-file',info['robot_description'],'-entity','go2','-x',str(info['spawn_x']),'-y',str(info['spawn_y']),
                            '-z',str(info['spawn_z']),'-timeout','30'])
    stand=node('go2_sim','sim_startup',[{'startup':True,'auto_stand':True,'duration':5.}])
    gait=Node(package='go2_gait_controller',executable='reference_trot_controller',name='go2_champ_controller',output='screen',
              parameters=[str(Path(get_package_share_directory('go2_gait_controller'))/'config/reference_trot.yaml'),{'use_sim_time':True}],
              remappings=[('/cmd_vel','/sim/cmd_vel')])
    startup_at=time.monotonic()
    def after_success(stage, next_actions):
        def callback(event, context):
            if context.is_shutdown:
                return []
            with open(run/'startup.jsonl','a') as stream:
                stream.write(json.dumps({'stage':stage,'returncode':event.returncode,'elapsed_s':round(time.monotonic()-startup_at,3),'wall_s':time.time()})+'\n')
            if event.returncode != 0:
                return [LogInfo(msg=f'[sim] START_FAILED stage={stage} returncode={event.returncode}'),
                        EmitEvent(event=Shutdown(reason=f'{stage} failed; no later startup stages will run'))]
            return [LogInfo(msg=f'[sim] STAGE_OK stage={stage}'),*next_actions]
        return callback
    handlers=[RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_success('MODEL_SPAWN',[stand]))),
              RegisterEventHandler(OnProcessExit(target_action=stand,on_exit=after_success('STAND_TRAJECTORY_SENT',[gait])))]
    actions=[SetEnvironmentVariable('GO2_GEOMETRY_FILE',str(geometry)),SetEnvironmentVariable('GO2_SPEEDS_FILE',str(speeds)),
             SetEnvironmentVariable('GO2_RUN_DIR',str(run)),*handlers,server,state,spawn,bridge,*slam_nodes,per_map,scan,evidence,mux,recover,safety,task,planner]
    if explorer is not None:
        actions.append(explorer)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=explorer,on_exit=[EmitEvent(event=Shutdown(reason='explorer exited'))])))
    for proc in [server,state,bridge,planner,per_map,scan,evidence,mux,recover,safety,task,gait]+[p for p in slam_nodes if isinstance(p,Node)]:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=proc,on_exit=[EmitEvent(event=Shutdown(reason='simulation core exited'))])))
    if cfg['gui']=='true':
        client=ExecuteProcess(cmd=['gzclient'],output='screen',additional_env={'QT_X11_NO_MITSHM':'1'})
        actions += [client,RegisterEventHandler(OnProcessExit(target_action=client,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))]))]

    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in
                              {'mode': 'nav', 'map': 'my_map', 'localisation':'legkilo', 'enable_motion': 'true', 'gui': 'true', 'web': 'false', 'offline': 'true'}.items()] + [OpaqueFunction(function=start)])
