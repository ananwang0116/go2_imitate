#!/usr/bin/env python3
"""loc_align: start-up alignment (spec 4.1). Owns TF map -> odom (static).

At start: map -> odom = identity (the dog stood at the map origin, facing x, when Leg-KILO started), event
ALIGN_DEFAULT; without any map -> odom the frame `map` does not exist and nothing can be drawn in it.
The initial position hint (/initialpose, a ROUGH hint: within search_xy_m / search_yaw_rad): the newest /per_map_scan is
matched against the /map walls around the hint (map_match.search, ~0.3-1 s). Then
  score >= min_score and not ambiguous -> map -> odom from the best pose, event ALIGN_OK (score, shift from the hint)
  otherwise                            -> the hint is used AS IS, event ALIGN_WEAK (score, reason) and a WARN: the user
                                          sees the dog jump to the click and can click again (decision 2026-10-25:
                                          a click must always have a visible effect)
map -> odom = pose_in_map * (odom -> base_footprint at the scan's stamp)^-1.
Decision 2026-10-25: no automatic correction while running (spec 4.4); loc_map_check / loc_pose only report.
"""
import math
import time

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener, TransformException

from go2_common import names
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, spin
from go2_loc.map_match import DistanceField, search


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def map_to_odom_from(px, py, pyaw, ox, oy, oyaw):
    """map -> odom such that base_footprint (at ox, oy, oyaw in odom) lands at px, py, pyaw in map."""
    yaw = math.atan2(math.sin(pyaw - oyaw), math.cos(pyaw - oyaw))
    c, s = math.cos(yaw), math.sin(yaw)
    return px - (c * ox - s * oy), py - (s * ox + c * oy), yaw


def scan_body_points(msg, max_points):
    r = np.asarray(msg.ranges, dtype=np.float64)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    keep = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    pts = np.column_stack((r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])))
    if len(pts) > max_points:
        pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(int)]
    return pts


MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LocAlign(Go2Node):
    def __init__(self):
        super().__init__('loc_align', 'loc')
        p = self.declare_parameter
        self.search_xy_m = float(p('search_xy_m', 1.0).value)
        self.search_yaw_rad = float(p('search_yaw_rad', 0.52).value)
        self.min_score = float(p('min_score', 0.6).value)
        self.max_second_ratio = float(p('max_second_ratio', 0.95).value)   # second-best place this close = ambiguous
        self.wall_tolerance_m = float(p('wall_tolerance_m', 0.10).value)
        self.field, self.scan, self.odom_pose = None, None, None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, names.INITIALPOSE, self.on_initialpose, CMD_QOS)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(LaserScan, names.PER_MAP_SCAN, self.on_scan, SENSOR_QOS)
        self.create_subscription(Odometry, names.LOC_ODOM, self.on_loc_odom, SENSOR_QOS)
        self.tf_static = StaticTransformBroadcaster(self)
        self.event('START', hint_in=names.INITIALPOSE, tf='map->odom', min_score=self.min_score)
        self.publish_map_to_odom(0.0, 0.0, 0.0)
        self.event('ALIGN_DEFAULT', x_m=0.0, y_m=0.0, yaw_rad=0.0, reason='not_aligned_yet')

    def publish_map_to_odom(self, x_m, y_m, yaw_rad):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = names.FRAME_MAP, names.FRAME_ODOM
        t.transform.translation.x, t.transform.translation.y = float(x_m), float(y_m)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)
        self.tf_static.sendTransform(t)

    def on_map(self, msg):
        i = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(i.height, i.width)
        self.field = DistanceField(grid, i.resolution, (i.origin.position.x, i.origin.position.y))

    def on_scan(self, msg):
        self.scan = msg

    def on_loc_odom(self, msg):
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation))

    def odom_pose_at(self, stamp):
        try:
            t = self.tf_buffer.lookup_transform(names.FRAME_ODOM, names.FRAME_BASE_FOOTPRINT, Time.from_msg(stamp)).transform
            return (t.translation.x, t.translation.y, yaw_of(t.rotation))
        except TransformException:
            return self.odom_pose

    def on_initialpose(self, msg):
        request_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        hint = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation))
        if msg.header.frame_id != names.FRAME_MAP:
            self.event('ALIGN_FAIL', request_stamp_ns=request_ns, source='initialpose', reason=f'frame_{msg.header.frame_id}_not_map')
            return
        if self.odom_pose is None:
            self.event('ALIGN_FAIL', request_stamp_ns=request_ns, source='initialpose', reason='no_loc_odom')
            self.get_logger().warn('[loc] ALIGN_FAIL: no /loc_odom yet (Leg-KILO not running / LOST)')
            return
        result, reason, odom_pose, ms = None, '', self.odom_pose, 0.0
        if self.field is None:
            reason = 'no_map'
        elif self.scan is None:
            reason = 'no_scan'
        else:
            scan = self.scan
            odom_pose = self.odom_pose_at(scan.header.stamp)
            t0 = time.perf_counter()
            result = search(self.field, scan_body_points(scan, 400), hint, self.search_xy_m, self.search_yaw_rad,
                            self.wall_tolerance_m)
            ms = (time.perf_counter() - t0) * 1e3
            if result is None:
                reason = 'too_few_scan_points'
            elif result['score'] < self.min_score:
                reason = 'low_score'
            elif result['second'] >= self.max_second_ratio * result['score']:
                reason = 'ambiguous'
        if result is not None and not reason:
            pose = result['pose']
            self.publish_map_to_odom(*map_to_odom_from(*pose, *odom_pose))
            self.event('ALIGN_OK', request_stamp_ns=request_ns, score=result['score'], second=result['second'], x_m=pose[0], y_m=pose[1],
                       yaw_deg=math.degrees(pose[2]), shift_m=math.hypot(pose[0] - hint[0], pose[1] - hint[1]),
                       turn_deg=math.degrees(math.atan2(math.sin(pose[2] - hint[2]), math.cos(pose[2] - hint[2]))), ms=ms)
            return
        self.publish_map_to_odom(*map_to_odom_from(*hint, *odom_pose))
        fields = dict(request_stamp_ns=request_ns, reason=reason, x_m=hint[0], y_m=hint[1], yaw_deg=math.degrees(hint[2]), ms=ms)
        if result is not None:
            fields.update(score=result['score'], second=result['second'])
        self.event('ALIGN_WEAK', **fields)
        self.get_logger().warn(f'[loc] 2D Pose Estimate used AS IS ({reason}): check that the laser points lie on the walls; click again if not')


def main():
    spin(LocAlign)


if __name__ == '__main__':
    main()
