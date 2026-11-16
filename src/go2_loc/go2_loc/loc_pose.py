#!/usr/bin/env python3
"""loc_pose: the single judge of "is the localisation trustworthy" (spec 2 P2, spec 4).

/kilo_odom (Leg-KILO, kilo_world -> kilo_base, ~15 Hz)
  -> TF odom -> base_footprint   planar part (x, y, yaw); height/roll/pitch are dev_bridge's base_footprint -> base
  -> TF odom -> kilo_world       static identity: odom IS Leg-KILO's world (origin = where the dog stood at launch)
  -> /loc_odom                   nav_msgs/Odometry in odom, body-frame twist from the pose difference over twist_window_s
  -> /loc_status (5 Hz)          OK | LOST (+ reason); events STATE from=.. to=.. reason=.. on every change, JUMP on a glitch
LOST = no /kilo_odom for kilo_timeout_s, or a pose jump faster than the dog can move (a single glitch is dropped;
a new place that stays for accept_after samples becomes the baseline). JUMP events: at most one per 5 s with the count
(a diverged Leg-KILO jumped every frame: 328 events in 23 s, 2026-10-25); > 10 in 5 s -> WARN "restart". Ported from ~/go2_kilo loc_kilo_pose.py.
DEGRADED: /loc_map_score below degraded_score continuously for degraded_hold_s (reason map_mismatch); back to OK only
after it stayed >= degraded_score + recover_margin for degraded_hold_s (hysteresis, spec P5). LOST always wins.
"""
import collections
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from go2_msgs.msg import LocStatus

from go2_common import names
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, spin

KILO_WORLD = 'kilo_world'     # Leg-KILO's own frame names (external, not renamed)
STATE_NAMES = {LocStatus.OK: 'OK', LocStatus.DEGRADED: 'DEGRADED', LocStatus.LOST: 'LOST'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseFilter:
    """Jump rejection without ROS (tested offline). feed() returns 'ok', 'glitch' (dropped) or 'rebase' (accepted new place)."""

    def __init__(self, max_speed_mps, max_yaw_rate_radps, accept_after):
        self.max_speed_mps, self.max_yaw_rate_radps, self.accept_after = max_speed_mps, max_yaw_rate_radps, accept_after
        self.last = None          # (t, x, y, yaw) last accepted
        self.candidate = None     # (t, x, y, yaw, count)

    def feed(self, t, x, y, yaw):
        if self.last is None:
            self.last = (t, x, y, yaw)
            return 'ok'
        if t <= self.last[0]:
            return 'old'
        dt = max(t - self.last[0], 1e-3)
        jump = (math.hypot(x - self.last[1], y - self.last[2]) / dt > self.max_speed_mps
                or abs(wrap(yaw - self.last[3])) / dt > self.max_yaw_rate_radps)
        if not jump:
            self.candidate = None
            self.last = (t, x, y, yaw)
            return 'ok'
        c = self.candidate
        if c is not None and math.hypot(x - c[1], y - c[2]) < 0.3 and abs(wrap(yaw - c[3])) < 0.3:
            self.candidate = (t, x, y, yaw, c[4] + 1)
        else:
            self.candidate = (t, x, y, yaw, 1)
        if self.candidate[4] < self.accept_after:
            return 'glitch'
        self.candidate = None
        self.last = (t, x, y, yaw)
        return 'rebase'


class Hysteresis:
    """A flag that turns on after `bad` held hold_s and off after `good` held hold_s (spec P5)."""

    def __init__(self, hold_s):
        self.hold_s, self.on, self.since = hold_s, False, None

    def update(self, now, bad, good):
        toggle = good if self.on else bad          # the condition that would change the current state
        if toggle:
            self.since = now if self.since is None else self.since
            if now - self.since >= self.hold_s:
                self.on, self.since = not self.on, None
        else:
            self.since = None
        return self.on


class LocPose(Go2Node):
    def __init__(self):
        super().__init__('loc_pose', 'loc')
        p = self.declare_parameter
        self.kilo_timeout_s = float(p('kilo_timeout_s', 0.5).value)
        self.max_speed_mps = float(p('max_speed_mps', 2.0).value)          # the dog does <= 0.4; faster = a glitch
        self.max_yaw_rate_radps = float(p('max_yaw_rate_radps', 6.0).value)
        self.accept_after = int(p('accept_after', 3).value)
        self.jump_lost_s = float(p('jump_lost_s', 2.0).value)              # LOST for this long after a jump
        self.twist_window_s = float(p('twist_window_s', 0.3).value)
        self.degraded_score = float(p('degraded_score', 0.45).value)   # aligned real-dog office: 0.50-0.89 (measured once)
        self.recover_margin = float(p('recover_margin', 0.05).value)
        self.degraded_hold_s = float(p('degraded_hold_s', 3.0).value)
        self.score_s = None
        self.jumps, self.last_jump_event_s = 0, -1e9
        self.hyst = Hysteresis(self.degraded_hold_s)
        self.filter = PoseFilter(self.max_speed_mps, self.max_yaw_rate_radps, self.accept_after)
        self.hist = collections.deque()
        self.last_rx_s = None
        self.lost_until_s = 0.0
        self.map_score = float('nan')
        self.state, self.reason = LocStatus.LOST, 'no /kilo_odom yet'

        self.create_subscription(Odometry, names.KILO_ODOM, self.on_kilo, SENSOR_QOS)
        self.create_subscription(Float32, names.LOC_MAP_SCORE, self.on_score, CMD_QOS)
        self.odom_pub = self.create_publisher(Odometry, names.LOC_ODOM, SENSOR_QOS)
        self.status_pub = self.create_publisher(LocStatus, names.LOC_STATUS, CMD_QOS)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = names.FRAME_ODOM, KILO_WORLD
        t.transform.rotation.w = 1.0
        self.tf_static.sendTransform(t)
        self.create_timer(0.2, self.publish_status)
        self.event('START', kilo_in=names.KILO_ODOM, tf='odom->base_footprint', state='LOST', reason=self.reason)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_score(self, msg):
        self.map_score, self.score_s = float(msg.data), self.now_s()

    def on_kilo(self, m):
        now = self.now_s()
        self.last_rx_s = now
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        prev = self.filter.last
        result = self.filter.feed(t, x, y, yaw)
        if result in ('glitch', 'rebase'):
            self.lost_until_s = now + self.jump_lost_s
            self.jumps += 1
            if now - self.last_jump_event_s >= 5.0:      # one event per 5 s: a diverging Leg-KILO jumps every frame
                self.event('JUMP', result=result, count=self.jumps, x_m=x, y_m=y, yaw_deg=math.degrees(yaw),
                           from_x_m=prev[1], from_y_m=prev[2], from_yaw_deg=math.degrees(prev[3]))
                if self.jumps > 10:
                    self.get_logger().warn('[loc] Leg-KILO keeps jumping (diverged?): stop, keep the dog still and restart the launch')
                self.last_jump_event_s, self.jumps = now, 0
            if result == 'glitch':
                return
            self.hist.clear()
        elif result == 'old':
            return
        self.hist.append((t, x, y, yaw))
        while len(self.hist) > 2 and t - self.hist[0][0] > self.twist_window_s:
            self.hist.popleft()
        self.publish_pose(m.header.stamp, x, y, yaw)

    def publish_pose(self, stamp, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, names.FRAME_ODOM, names.FRAME_BASE_FOOTPRINT
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(tf)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, names.FRAME_ODOM, names.FRAME_BASE_FOOTPRINT
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        first, last = self.hist[0], self.hist[-1]
        dt = last[0] - first[0]
        if len(self.hist) >= 2 and dt > 0.05:
            vx, vy, wz = (last[1] - first[1]) / dt, (last[2] - first[2]) / dt, wrap(last[3] - first[3]) / dt
            c, s = math.cos(yaw), math.sin(yaw)
            o.twist.twist.linear.x, o.twist.twist.linear.y = c * vx + s * vy, -s * vx + c * vy
            o.twist.twist.angular.z = wz
        self.odom_pub.publish(o)

    def publish_status(self):
        now = self.now_s()
        if self.last_rx_s is None:
            state, reason = LocStatus.LOST, 'no /kilo_odom yet'
        elif now - self.last_rx_s > self.kilo_timeout_s:
            state, reason = LocStatus.LOST, 'kilo_stale'
        elif now < self.lost_until_s:
            state, reason = LocStatus.LOST, 'pose_jump'
        else:
            fresh = self.score_s is not None and now - self.score_s < 3.0
            bad = fresh and self.map_score < self.degraded_score
            good = (not fresh) or self.map_score >= self.degraded_score + self.recover_margin
            degraded = self.hyst.update(now, bad, good)
            state, reason = (LocStatus.DEGRADED, 'map_mismatch') if degraded else (LocStatus.OK, '')
        if state != self.state or (state != LocStatus.OK and reason != self.reason):
            self.event('STATE', from_state=STATE_NAMES[self.state], to=STATE_NAMES[state], reason=reason or 'fresh')
            if state == LocStatus.LOST:
                self.warn_throttled('lost', f'LOST ({reason})')
        self.state, self.reason = state, reason
        msg = LocStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.reason = state, reason
        msg.kilo_age_s = float('nan') if self.last_rx_s is None else now - self.last_rx_s
        msg.map_score = self.map_score
        self.status_pub.publish(msg)


def main():
    spin(LocPose)


if __name__ == '__main__':
    main()
