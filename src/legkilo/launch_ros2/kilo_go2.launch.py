"""Leg-KILO (LIO mode) on the real Go2. Normally started by `ros2 launch go2_bridge go2_real.launch.py mode:=mapping kilo:=true`."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution(
            [FindPackageShare('legkilo'), 'config', 'go2_mid360.yaml'])),
        # glog: every level always goes to the log files in <legkilo>/logs; only >= stderrthreshold (0 INFO, 1 WARNING, 2 ERROR) reaches the terminal.
        DeclareLaunchArgument('stderrthreshold', default_value='0'),
        Node(package='legkilo', executable='legkilo_node', name='legkilo_node', output='screen',
             arguments=['--config_file', config, ['--stderrthreshold=', LaunchConfiguration('stderrthreshold')],
                        '--colorlogtostderr=false']),   # go2_v2: plain white terminal (user request 2026-10-25)
    ])
