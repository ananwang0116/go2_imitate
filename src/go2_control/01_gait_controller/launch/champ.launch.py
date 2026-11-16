from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = str(Path(get_package_share_directory('go2_gait_controller')) / 'config/reference_trot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('gait_controller', default_value='reference_trot_controller'),
        Node(package='go2_gait_controller', executable=LaunchConfiguration('gait_controller'),
             name='go2_champ_controller', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}])
    ])
