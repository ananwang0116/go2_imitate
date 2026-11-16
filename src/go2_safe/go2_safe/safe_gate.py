#!/usr/bin/env python3
"""safe_gate: the single judge of "will moving this way now hit something" (spec 2 P2, L4).

/mot_cmd_vel + /per_obstacle_scan + /dev_state + /loc_status -> /safe_cmd_vel, /safe_status
Only limits, never creates motion (rules in safe_logic.py).
Every command: clamp to config/speeds.yaml, zero on a dog fault; silence lets dev_bridge stop on its own timeout.
Which further checks apply (safe_logic.checks_for; user decisions 2026-10-25, confirmed 2026-10-26):
- HAND commands, in ANY mode (teleop fresh on /dev_teleop_cmd_vel; teleop has the highest priority in mot_cmd_mux, so
  /mot_cmd_vel IS the keyboard / web console), and everything in mapping (launch mode map) or with manual:=true: nothing
  else - pure hand driving, also when the user takes over during navigation to fix a problem ("遥控是不受限制的").
- AUTONOMOUS commands (mot_nav, mot_recover; navigation only): linear speed limited so that braking leaves
  safe_stop_gap_m before the nearest point in the body's swept strip; a pure in-place turn is zeroed when a scan point would
  enter the turning body rectangle (safe_logic.rotation_blocked); zero when the scan is stale or localisation is LOST.
Event CONTROL to=hand|auto obstacle_limit=.. when this changes.
Publishes only while /mot_cmd_vel is fresh (one zero at the end): silence lets dev_bridge stop the dog by its own timeout.
Events: CONTROL (hand/auto), LIMIT (limiting starts / its reason changes), CLEAR (no limiting for clear_hold_s). /safe_status carries the
numbers at 20 Hz. Obstacles = the union of the scans of the last scan_memory_s (one frame of the Go2 lidar has holes:
the gap read inf for single frames at a wall and the speed jumped back to full, 2026-10-25). Speed limits:
config/speeds.yaml (back separately); allowed speeds below min_move_mps become 0 (see safe_logic).
Latency: the command is processed in its own callback (no waiting for the 20 Hz tick); the scan points are kept as
a numpy array, so one decision costs ~0.1 ms for 720 points.
"""
import collections
import math

import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from go2_msgs.msg import DevState, LocStatus, SafeStatus

from go2_common import names, speeds
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, spin
from go2_safe.safe_logic import checks_for, clamp, limit, scan_points


class SafeGate(Go2Node):
    def __init__(self):
        super().__init__('safe_gate', 'safe')
        p = self.declare_parameter
        self.manual = bool(p('manual', False).value)
        self.mode = p('mode', 'nav').value                 # launch mode: map = only hand driving exists, never limited
        self.scan_timeout_s = float(p('scan_timeout_s', 0.4).value)
        self.cmd_timeout_s = float(p('cmd_timeout_s', 0.3).value)
        self.state_timeout_s = float(p('state_timeout_s', 0.3).value)
        s = speeds.load()
        side = s.manual_side_mps if self.manual else s.side_mps
        self.max_v = (float(p('max_vx_mps', s.forward_mps).value), float(p('max_vy_mps', side).value),
                      float(p('max_wz_radps', s.turn_radps).value), float(p('max_back_mps', s.back_mps).value))
        self.min_move_mps = float(p('min_move_mps', 0.12).value)
        self.scan_memory_s = float(p('scan_memory_s', 0.3).value)
        self.clear_hold_s = float(p('clear_hold_s', 1.0).value)
        self.scans = collections.deque()          # (time, points): the union of the last scan_memory_s
        self.points, self.scan_s = None, -1e9
        self.limit_off_since = None
        self.fault, self.state_s = True, -1e9
        self.loc_state, self.loc_s = LocStatus.LOST, -1e9
        self.cmd, self.cmd_s = None, -1e9
        self.reason = None
        self.active = False
        self.create_subscription(Twist, names.MOT_CMD_VEL, self.on_cmd, CMD_QOS)
        self.hand_timeout_s = float(p('hand_timeout_s', 0.5).value)       # = mot_cmd_mux source_timeout_s
        self.teleop_s, self.control = -1e9, None
        self.create_subscription(Twist, names.DEV_TELEOP_CMD_VEL, self.on_teleop, CMD_QOS)
        self.create_subscription(LaserScan, names.PER_OBSTACLE_SCAN, self.on_scan, SENSOR_QOS)
        self.create_subscription(DevState, names.DEV_STATE, self.on_dev_state, CMD_QOS)
        self.create_subscription(LocStatus, names.LOC_STATUS, self.on_loc_status, CMD_QOS)
        self.cmd_pub = self.create_publisher(Twist, names.SAFE_CMD_VEL, CMD_QOS)
        self.status_pub = self.create_publisher(SafeStatus, names.SAFE_STATUS, CMD_QOS)
        self.create_timer(0.05, self.tick)
        g = self.geometry
        self.event('START', mode=self.mode, manual=self.manual, safe_stop_gap_m=g.safe_stop_gap_m, brake_m_at_0_4mps=g.brake_distance_m(0.4))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        now = self.now_s()
        self.scans.append((now, scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)))
        while self.scans and now - self.scans[0][0] > self.scan_memory_s:
            self.scans.popleft()
        self.points = np.vstack([s[1] for s in self.scans])     # one frame alone has holes (gap=inf seen at a wall)
        self.scan_s = now

    def on_teleop(self, msg):
        self.teleop_s = self.now_s()

    def on_dev_state(self, msg):
        self.fault, self.state_s = bool(msg.fault), self.now_s()

    def on_loc_status(self, msg):
        self.loc_state, self.loc_s = msg.state, self.now_s()

    def on_cmd(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            return
        self.cmd, self.cmd_s = msg, self.now_s()
        self.decide()

    def tick(self):
        if self.active and self.now_s() - self.cmd_s >= self.cmd_timeout_s:
            self.cmd_pub.publish(Twist())
            self.active = False
            self.publish_status(True, 'all', 'no_input', math.nan, 0.0, (0, 0, 0), (0, 0, 0))

    def decide(self):
        now = self.now_s()
        c = self.cmd
        cin = (c.linear.x, c.linear.y, c.angular.z)
        g = self.geometry
        control, obstacle_limit, loc_check = checks_for(self.mode, self.manual, now - self.teleop_s < self.hand_timeout_s)
        if control != self.control:
            self.event('CONTROL', to=control, obstacle_limit=obstacle_limit)
            self.control = control
        block = None
        if self.fault or now - self.state_s > self.state_timeout_s:
            block = 'dev_fault'
        elif loc_check and (self.loc_state == LocStatus.LOST or now - self.loc_s > 1.0):
            block = 'loc_lost'
        elif obstacle_limit and now - self.scan_s > self.scan_timeout_s:
            block = 'stale_scan'
        if block:
            out, gap, direction = (0.0, 0.0, 0.0), math.nan, 'all'
        elif not obstacle_limit:
            out = clamp(cin, self.max_v)
            gap, direction = math.nan, ''
        else:
            vx, vy, wz, gap, direction = limit(cin, self.points, g, self.max_v, self.min_move_mps)
            out = (vx, vy, wz)
        lin_in, lin_out = math.hypot(cin[0], cin[1]), math.hypot(out[0], out[1])
        # A held in-place turn counts as limiting too (mot_recover reports `blocked` from it).
        turn_held = abs(out[2]) < abs(clamp(cin, self.max_v)[2]) - 0.01
        limiting = bool(block) or lin_out < lin_in - 0.01 or turn_held
        reason = block or ('obstacle' if limiting else None)
        # events with hysteresis: LIMIT at once (or when the reason changes), CLEAR only after clear_hold_s without
        # limiting (approaching a wall flipped LIMIT/CLEAR every frame, 2026-10-25)
        if reason:
            self.limit_off_since = None
            if reason != self.reason:
                self.event('LIMIT', reason=reason, dir=direction or 'all', gap_m=gap, v_in=lin_in, v_out=lin_out)
                if block:
                    self.warn_throttled(block, f'LIMIT {block}: command zeroed')
                self.reason = reason
        elif self.reason:
            self.limit_off_since = self.limit_off_since or now
            if now - self.limit_off_since >= self.clear_hold_s:
                self.event('CLEAR', after=self.reason)
                self.reason, self.limit_off_since = None, None
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = out
        self.cmd_pub.publish(t)
        self.active = True
        self.publish_status(limiting, direction, reason or '', gap, g.brake_distance_m(lin_in), cin, out)

    def publish_status(self, limiting, direction, reason, gap, brake, cin, cout):
        s = SafeStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.limiting, s.direction, s.reason = limiting, direction, reason
        s.gap_m, s.brake_distance_m = float(gap), float(brake)
        s.cmd_in, s.cmd_out = [float(v) for v in cin], [float(v) for v in cout]
        self.status_pub.publish(s)


def main():
    spin(SafeGate)


if __name__ == '__main__':
    main()
