#!/bin/bash
# Desktop only: starts the web console; its Start button opens Gazebo.
set -eo pipefail
cd "$(dirname "$0")/.."
source ./go2_env.sh
if [ ! -f install/go2_sim/share/go2_sim/package.xml ]; then
  echo '请先编译：source ./go2_env.sh && colcon build --symlink-install --base-paths src'
  exit 2
fi
exec /usr/bin/python3 tools/sim_console.py
