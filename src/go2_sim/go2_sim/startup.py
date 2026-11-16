"""Sequential simulation controller setup, then the original standing command.

One ROS process avoids repeated Python/DDS discovery between dependent stages.
The inherited standing trajectory, activation and feedback checks are unchanged.
"""
import importlib.util
import json
import os
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import LoadController, ConfigureController

_path = Path(get_package_prefix('go2_joint_command_controller'))/'lib/go2_joint_command_controller/stand_command.py'
_spec = importlib.util.spec_from_file_location('go2_original_stand_command', _path)
_stand = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_stand)

CONTROLLERS = (
    ('joint_state_broadcaster', 'JOINT_BROADCASTER_LOADED'),
    ('go2_joint_tracking_controller', 'STAND_CONTROLLER_LOADED'),
    ('go2_balance_effort_controller', 'EFFORT_CONTROLLER_LOADED'),
)


class SimStartup(_stand.StandCommand):
    def __init__(self):
        super().__init__()
        self.load = self.create_client(LoadController, '/controller_manager/load_controller')
        self.configure = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        self.index, self.configuring, self.future = 0, False, None

    def tick(self):
        if self.index >= len(CONTROLLERS):
            return super().tick()
        name, stage = CONTROLLERS[self.index]
        if time.monotonic()-self.started_at > 30:
            self.get_logger().error('Controller setup deadline exceeded: '+name)
            self.failed = self.finished = True
            return
        client = self.configure if self.configuring else self.load
        if self.future is None:
            if not client.service_is_ready():
                return
            req = ConfigureController.Request() if self.configuring else LoadController.Request()
            req.name = name
            self.future = client.call_async(req)
            return
        if not self.future.done():
            return
        try:
            if not self.future.result().ok:
                raise RuntimeError(('configure ' if self.configuring else 'load ')+name+' rejected')
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.failed = self.finished = True
            return
        self.future = None
        if self.configuring:
            with (Path(os.environ['GO2_RUN_DIR'])/'startup.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(stage=stage,returncode=0,wall_s=time.time(),
                    setup_elapsed_s=round(time.monotonic()-self.started_at,3)))+'\n')
            self.get_logger().info('STAGE_OK stage='+stage)
            self.index += 1
        self.configuring = not self.configuring


def main():
    rclpy.init()
    node = SimStartup()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.failed = True
    finally:
        failed = node.failed
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0
