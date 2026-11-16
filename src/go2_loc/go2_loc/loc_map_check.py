#!/usr/bin/env python3
"""loc_map_check: running laser-vs-map consistency (spec 4.2). It only measures; loc_pose decides OK / DEGRADED.

Every period_s: the newest /per_map_scan (wall height band) is put into the map with TF map -> base_footprint at
the scan's stamp; /loc_map_score = share of points within wall_tolerance_m of a /map wall (map_match.py).
Points are capped (max_points) so one check costs ~1 ms. Events SCORE_LOW / SCORE_OK with hysteresis: low after
event_hold_s below low_score, ok after event_hold_s at or above low_score + margin (a score wandering around the
threshold printed one event per second, 2026-10-25). The numbers are on the topic and in the bag.
low_score 0.45: aligned on the real dog in an office the score was 0.50-0.89 (furniture not in the map), measured once.
"""
import math
from collections import deque

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformException

from go2_common import names
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, spin
from go2_loc.loc_align import MAP_QOS, scan_body_points, yaw_of
from go2_loc.map_match import DistanceField, score, to_map


class LocMapCheck(Go2Node):
    def __init__(self):
        super().__init__('loc_map_check', 'loc')
        p = self.declare_parameter
        self.period_s = float(p('period_s', 1.0).value)
        self.wall_tolerance_m = float(p('wall_tolerance_m', 0.10).value)
        self.low_score = float(p('low_score', 0.45).value)
        self.margin = float(p('margin', 0.05).value)
        self.event_hold_s = float(p('event_hold_s', 3.0).value)
        self.since = None
        self.max_points = int(p('max_points', 360).value)
        self.field, self.scan, self.low = None, None, None
        self.scans = deque(maxlen=12)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(LaserScan, names.PER_MAP_SCAN, self.on_scan, SENSOR_QOS)
        self.score_pub = self.create_publisher(Float32, names.LOC_MAP_SCORE, CMD_QOS)
        self.create_timer(self.period_s, self.check)
        self.event('START', out=names.LOC_MAP_SCORE, tol_m=self.wall_tolerance_m)

    def on_map(self, msg):
        i = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(i.height, i.width)
        self.field = DistanceField(grid, i.resolution, (i.origin.position.x, i.origin.position.y))

    def on_scan(self, msg):
        self.scan = msg
        self.scans.append(msg)

    def check(self):
        if self.field is None or self.scan is None:
            return
        # LIO follows the ray callback. Use the newest resolvable scan within
        # one second, with its own TF; never move an old scan by latest pose.
        now_ns = self.get_clock().now().nanoseconds
        for s in reversed(self.scans):
            stamp_ns = s.header.stamp.sec*10**9+s.header.stamp.nanosec
            if not 0 <= now_ns-stamp_ns <= 10**9:
                continue
            try:
                t = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_BASE_FOOTPRINT,
                                                    Time.from_msg(s.header.stamp)).transform
                break
            except TransformException:
                continue
        else:
            return
        pose = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        value = score(self.field, to_map(scan_body_points(s, self.max_points), pose), self.wall_tolerance_m)
        if math.isnan(value):
            return
        self.score_pub.publish(Float32(data=value))
        now = self.get_clock().now().nanoseconds * 1e-9
        flip = (value >= self.low_score + self.margin) if self.low else (value < self.low_score)
        if self.low is None:
            flip = True
        if flip:
            self.since = now if self.since is None else self.since
            if self.low is None or now - self.since >= self.event_hold_s:
                self.low = not self.low if self.low is not None else value < self.low_score
                self.event('SCORE_LOW' if self.low else 'SCORE_OK', score=value, threshold=self.low_score)
                self.since = None
        else:
            self.since = None


def main():
    spin(LocMapCheck)


if __name__ == '__main__':
    main()
