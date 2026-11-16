"""Articulated Gazebo adapter: true body motion owns the mounted lidar pose."""
import csv
import math
import os
import time
from pathlib import Path
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from gazebo_msgs.msg import ModelStates
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float32, Float64MultiArray, UInt8MultiArray
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from go2_msgs.msg import DevState, LocStatus, Event, SafeStatus
from go2_common.node import Go2Node, CMD_QOS, SENSOR_QOS, EVENT_QOS, spin


class SimBridge(Go2Node):
    def __init__(self):
        super().__init__("sim_bridge", "dev")
        self.localisation = self.declare_parameter('localisation','legkilo').value
        self.slam = self.localisation == 'legkilo'
        self.loc_at=-1e9;self.loc_ok_since=None;self.gait_announced=False
        self.enabled = bool(self.declare_parameter("enable_motion", True).value)
        self.last_cmd = self.last_balance = self.last_query = -1e9
        self.balance = None
        self.controller_active = False
        self.controller_ready_after = float('inf')
        self.future = None
        self.ready = False
        self.map_seen = self.scan_seen = False
        self.last_align = -1e9
        self.phase = -1
        self.planned_contacts = [-1]*4
        self.actual_contacts = [-1]*4
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dev_pub = self.create_publisher(DevState, "/dev_state", CMD_QOS)
        self.odom_pub = self.create_publisher(Odometry, "/sim/ground_truth_odom" if self.slam else "/loc_odom", SENSOR_QOS)
        self.loc_pub = None if self.slam else self.create_publisher(LocStatus, "/loc_status", CMD_QOS)
        self.score_pub = None if self.slam else self.create_publisher(Float32, "/loc_map_score", CMD_QOS)
        if self.slam:self.create_subscription(LocStatus,"/loc_status",self.on_loc,CMD_QOS)
        self.cmd_pub = self.create_publisher(Twist, "/sim/cmd_vel", CMD_QOS)
        self.align_pub = self.create_publisher(Event, "/loc_event", EVENT_QOS)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(LaserScan, "/per_obstacle_scan", self.on_scan, SENSOR_QOS)
        self.create_subscription(ModelStates, "/sim/model_states", self.on_state, SENSOR_QOS)
        self.create_subscription(Float64MultiArray, "/go2/balance_status", self.on_balance, SENSOR_QOS)
        self.create_subscription(UInt8MultiArray, '/go2/gait_contacts', lambda m: self.keep_contacts(m, True), SENSOR_QOS)
        self.create_subscription(UInt8MultiArray, '/go2/actual_contacts', lambda m: self.keep_contacts(m, False), SENSOR_QOS)
        self.create_subscription(Twist, "/safe_cmd_vel", self.on_cmd, CMD_QOS)
        self.create_service(Trigger, "/dev_stop", self.stop)
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.create_timer(.05, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        t = TransformStamped(); t.header.frame_id = "map"; t.child_frame_id = "odom"; t.transform.rotation.w = 1.
        if not self.slam:self.static_tf.sendTransform(t)
        self.motion_log = None
        run = os.environ.get("GO2_RUN_DIR")
        if run:
            self.motion_log = open(Path(run)/"body_motion.csv", "w", buffering=1)
            self.motion_writer = csv.writer(self.motion_log)
            self.motion_writer.writerow(["sim_s","x","y","z","roll","pitch","yaw","gait_phase","gait_fault","gait_ready",
                                         'plan_FL','plan_FR','plan_RL','plan_RR','contact_FL','contact_FR','contact_RL','contact_RR'])
        self.cmd_log = None
        if run:
            # Per-run velocity-command trace (log only, no control): who commanded what at each stage of the chain.
            self.cmd_log = open(Path(run)/"cmd_trace.csv", "w", buffering=1)
            self.cmd_writer = csv.writer(self.cmd_log)
            self.cmd_writer.writerow(["sim_s", "topic", "vx", "vy", "wz", "limiting", "reason"])
            for topic in ("/dev_teleop_cmd_vel", "/mot_nav_cmd_vel", "/mot_recover_cmd_vel", "/mot_cmd_vel", "/safe_cmd_vel", "/sim/cmd_vel"):
                self.create_subscription(Twist, topic, lambda m, t=topic: self.trace(t, m.linear.x, m.linear.y, m.angular.z), CMD_QOS)
            self.create_subscription(SafeStatus, "/safe_status", lambda m: self.trace(
                "/safe_status(out)", m.cmd_out[0], m.cmd_out[1], m.cmd_out[2], int(m.limiting), m.reason), CMD_QOS)
        self.event("START", source="gazebo_physics", localisation=self.localisation, cloud="web_none", motion=self.enabled)

    def on_loc(self,m):
        self.loc_at=time.monotonic()
        if m.state == LocStatus.OK:
            if self.loc_ok_since is None:self.loc_ok_since=self.loc_at
        else:self.loc_ok_since=None

    def localisation_ready(self):
        # Start-up readiness only (SIM_READY below, latched); never used to gate velocity commands.
        return (not self.slam or (self.loc_ok_since is not None and
                time.monotonic()-self.loc_ok_since>=2. and time.monotonic()-self.loc_at<.6))

    def on_map(self, msg):
        self.map_seen = True

    def on_scan(self, msg):
        self.scan_seen = bool(msg.ranges)

    def on_balance(self, msg):
        if len(msg.data) >= 8:
            self.balance = list(msg.data); self.last_balance = time.monotonic(); self.phase = int(msg.data[0])

    def keep_contacts(self, msg, planned):
        if len(msg.data) == 4:
            if planned: self.planned_contacts = list(msg.data)
            else: self.actual_contacts = list(msg.data)

    def gait_ready(self):
        return (self.controller_active and self.balance is not None and
                self.get_clock().now().nanoseconds*1e-9 >= self.controller_ready_after and
                time.monotonic()-self.last_balance < .4 and self.balance[7] == 0.)

    def on_cmd(self, msg):
        self.last_cmd = time.monotonic()
        # Localisation state is NOT a motion gate (removed 2026-11-11): DEGRADED/LOST must not freeze hand driving or recovery.
        # safe_gate alone limits automatic commands (LOST); hand driving stays unrestricted.
        self.cmd_pub.publish(msg if self.enabled and self.gait_ready() else Twist())

    def watchdog(self):
        now = time.monotonic()
        if now-self.last_cmd > .3 or not self.gait_ready():
            self.cmd_pub.publish(Twist())
        if self.future is not None and self.future.done():
            try:
                states = {c.name:c.state for c in self.future.result().controller}
                active = states.get("go2_balance_effort_controller") == "active" and states.get("go2_joint_tracking_controller") == "inactive"
                if active and not self.controller_active:
                    self.controller_ready_after = self.get_clock().now().nanoseconds*1e-9 + 1.
                self.controller_active = active
            except Exception:
                self.controller_active = False
            self.future = None
        if self.future is None and now-self.last_query > .5 and self.controllers.service_is_ready():
            self.future = self.controllers.call_async(ListControllers.Request()); self.last_query = now

    def stop(self, request, response):
        self.last_cmd = -1e9; self.cmd_pub.publish(Twist())
        response.success = True; response.message = "simulated gait stop requested"
        return response

    def on_state(self, msg):
        if "go2" not in msg.name: return
        i = msg.name.index("go2"); pose, velocity = msg.pose[i], msg.twist[i]
        q = pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        stamp = self.get_clock().now().to_msg()
        c,s = math.cos(yaw),math.sin(yaw)
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = "odom"; o.child_frame_id = "base_footprint"
        o.pose.pose.position.x = pose.position.x; o.pose.pose.position.y = pose.position.y
        o.pose.pose.orientation.z = math.sin(yaw/2); o.pose.pose.orientation.w = math.cos(yaw/2)
        o.twist.twist.linear.x = c*velocity.linear.x+s*velocity.linear.y
        o.twist.twist.linear.y = -s*velocity.linear.x+c*velocity.linear.y
        o.twist.twist.angular.z = velocity.angular.z
        if self.slam:
            o.header.frame_id='sim_world';o.child_frame_id='sim_base'
        self.odom_pub.publish(o)
        footprint = TransformStamped(); footprint.header = o.header; footprint.child_frame_id = "base_footprint"
        footprint.transform.translation.x = pose.position.x; footprint.transform.translation.y = pose.position.y
        footprint.transform.rotation = o.pose.pose.orientation
        body = TransformStamped(); body.header.stamp = stamp; body.header.frame_id = "base_footprint"; body.child_frame_id = "base"
        body.transform.translation.z = pose.position.z
        cy,sy = math.cos(yaw/2),math.sin(yaw/2)
        body.transform.rotation.x = cy*q.x+sy*q.y; body.transform.rotation.y = cy*q.y-sy*q.x
        body.transform.rotation.z = cy*q.z-sy*q.w; body.transform.rotation.w = cy*q.w+sy*q.z
        if not self.slam:self.tf.sendTransform([footprint, body])
        ready = self.gait_ready()
        fallen = abs(roll) > .6 or abs(pitch) > .6 or pose.position.z < .12
        state = DevState(); state.header = o.header; state.error_code = 100
        state.body_height_m = pose.position.z; state.roll_rad = roll; state.pitch_rad = pitch; state.yaw_rad = yaw
        state.position_m = [pose.position.x,pose.position.y,pose.position.z]
        state.velocity_mps = [o.twist.twist.linear.x,o.twist.twist.linear.y,velocity.linear.z]
        state.yaw_speed_radps = velocity.angular.z
        state.fault = fallen or bool(self.balance is not None and self.balance[7])
        state.fault_reason = "fall_or_gait_fault" if state.fault else ""
        self.dev_pub.publish(state)
        if not self.slam:
            loc = LocStatus(); loc.header = o.header; loc.state = LocStatus.OK if ready and not fallen else LocStatus.LOST
            loc.reason = "gazebo_ground_truth" if loc.state == LocStatus.OK else "waiting_for_standing_and_trot_controller"
            loc.map_score = 1.; self.loc_pub.publish(loc); self.score_pub.publish(Float32(data=1.))
            if ready and not fallen and time.monotonic()-self.last_align > 2.:
                event = Event(); event.header = o.header; event.layer = "loc"; event.name = "ALIGN_OK"
                self.align_pub.publish(event); self.last_align = time.monotonic()
        if ready and not fallen and not self.gait_announced:
            self.event('SIM_GAIT_READY',source='controller_and_physical_feedback')
            self.gait_announced=True
        if ready and self.localisation_ready() and self.map_seen and self.scan_seen and not fallen and not self.ready:
            self.event("SIM_READY", source="articulated_reference_trot", controllers=True, map=True, scan=True, localisation=self.localisation)
            self.ready = True
        if self.motion_log:
            self.motion_writer.writerow([stamp.sec+stamp.nanosec*1e-9,pose.position.x,pose.position.y,pose.position.z,
                                         roll,pitch,yaw,self.phase,0 if self.balance is None else self.balance[7],int(ready),
                                         *self.planned_contacts,*self.actual_contacts])

    def trace(self, topic, vx, vy, wz, limiting="", reason=""):
        if self.cmd_log:
            self.cmd_writer.writerow([round(self.get_clock().now().nanoseconds*1e-9, 3), topic, round(vx, 4), round(vy, 4),
                                      round(wz, 4), limiting, reason])

    def on_shutdown(self):
        if self.motion_log: self.motion_log.close()
        if self.cmd_log: self.cmd_log.close()


def main():
    spin(SimBridge)
