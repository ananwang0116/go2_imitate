# source ./go2_env.sh -- go2_v4 desktop simulation only
_go2_v4_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E 'miniconda|anaconda' | paste -sd:)"
source /opt/ros/humble/setup.bash
[ ! -f "$_go2_v4_ws/install/local_setup.bash" ] || source "$_go2_v4_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11348
export GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GO2_WS="$_go2_v4_ws"
export RCUTILS_COLORIZED_OUTPUT=0
unset GO2_DDS_CONFIG CYCLONEDDS_URI
unset _go2_v4_ws
