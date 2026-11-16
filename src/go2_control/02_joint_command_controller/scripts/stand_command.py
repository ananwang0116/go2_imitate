#!/usr/bin/python3
"""Start Gazebo controllers together and send a smooth standing trajectory."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, ListControllers
from std_srvs.srv import Empty


class StandCommand(Node):
    def __init__(self):
        super().__init__('go2_stand_command')
        self.joints = [f'{leg}_{part}_joint' for leg in ('FL', 'FR', 'RL', 'RR') for part in ('hip', 'thigh', 'calf')]
        self.declare_parameter('duration', 5.0)
        self.declare_parameter('startup', False)
        self.declare_parameter('auto_stand', True)
        self.duration = float(self.get_parameter('duration').value)
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('duration must be finite and positive')
        self.startup = self.get_parameter('startup').value
        self.auto_stand = self.get_parameter('auto_stand').value
        self.positions = None
        self.sent = False
        self.finished = False
        self.failed = False
        self.started_at = time.monotonic()
        self.sent_at = None
        self.switch_future = None
        self.list_future = None
        self.switch_started = None
        self.unpause_future = None
        self.publisher = self.create_publisher(JointTrajectory, '/go2_joint_tracking_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.unpause = self.create_client(Empty, '/unpause_physics')
        self.list_controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        # A steady clock keeps startup polling alive while simulation time is paused.
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def feedback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values and math.isfinite(values[j]) for j in self.joints):
            self.positions = [values[j] for j in self.joints]

    def tick(self):
        if time.monotonic() - self.started_at > 60 and not self.sent:
            self.get_logger().error('Timed out waiting for Gazebo controllers and twelve-joint feedback.')
            self.failed = True
            self.finished = True
            return
        if self.startup:
            if self.switch_future is None:
                if not self.switch.service_is_ready() or not self.unpause.service_is_ready():
                    return
                request = SwitchController.Request()
                request.activate_controllers = ['joint_state_broadcaster', 'go2_joint_tracking_controller']
                request.strictness = SwitchController.Request.STRICT
                request.timeout.sec = 10
                self.switch_started = time.monotonic()
                self.switch_future = self.switch.call_async(request)
                # Switch completes in a physics update: queue activation before unpausing.
                self.unpause_future = self.unpause.call_async(Empty.Request())
                return
            if not self.switch_future.done() or not self.unpause_future.done():
                # DDS can lose a service response under startup load. Verify the
                # actual controller state and running physics instead of hanging.
                if time.monotonic() - self.switch_started < 2 or self.positions is None:
                    return
                if self.list_future is None:
                    if self.list_controllers.service_is_ready():
                        self.list_future = self.list_controllers.call_async(ListControllers.Request())
                    return
                if not self.list_future.done():
                    return
                response = self.list_future.result()
                states = {c.name: c.state for c in response.controller}
                self.list_future = None
                if all(states.get(c) == 'active' for c in ('joint_state_broadcaster', 'go2_joint_tracking_controller')):
                    self.get_logger().info('Verified active controllers and live feedback after delayed startup response.')
                    self.startup = False
                else:
                    return
            try:
                if self.startup:
                    if not self.switch_future.result().ok:
                        raise RuntimeError('Controller activation was rejected')
                    self.unpause_future.result()
            except Exception as exc:
                self.get_logger().error(str(exc))
                self.failed = True
                self.finished = True
                return
            self.startup = False
        if self.sent:
            if time.monotonic() - self.sent_at > 1:
                self.finished = True
            return
        if self.positions is None or self.publisher.get_subscription_count() == 0:
            return
        msg = JointTrajectory()
        msg.joint_names = self.joints
        start = JointTrajectoryPoint()
        start.positions = self.positions
        start.velocities = [0.0] * 12
        start.accelerations = [0.0] * 12
        end = JointTrajectoryPoint()
        end.positions = [0.0, 0.65, -1.3] * 4 if self.auto_stand else self.positions
        end.velocities = [0.0] * 12
        end.accelerations = [0.0] * 12
        end.time_from_start.sec = int(self.duration)
        end.time_from_start.nanosec = int((self.duration % 1) * 1e9)
        msg.points = [start, end]
        self.publisher.publish(msg)
        self.sent = True
        self.sent_at = time.monotonic()
        self.get_logger().info(
            f'Smooth {self.duration:g} s standing trajectory sent; final target remains active.'
            if self.auto_stand else 'Crouched posture target sent; run stand_command.py to stand up.')


def main():
    rclpy.init()
    node = StandCommand()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        failed = node.failed
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
