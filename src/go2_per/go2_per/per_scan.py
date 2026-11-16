#!/usr/bin/env python3
"""per_scan: /dev_lidar_cloud -> /per_obstacle_scan, /per_map_scan, /per_free_scan (frame base_footprint, the cloud's stamp).

Rules in per_logic.py are unchanged. Up to eight clouds wait at most one
second for their OWN timestamp's body attitude. Process the newest resolvable
cloud, dropping expired/older frames; never project through latest body TF.
The ground plane is fitted in the current base_footprint frame. A missing fit
uses that frame's zero-height reference instead of retaining an older frame's
slope. This avoids painting a remembered body tilt into the floor after a stop.
WARN SLOW when one cloud takes longer than slow_ms (throttled).
"""
from collections import deque
import array
import math
import time

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException

from go2_common import names
from go2_common.node import Go2Node, SENSOR_QOS, spin
from go2_per import per_logic

LATEST_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PerScan(Go2Node):
    def __init__(self):
        super().__init__('per_scan', 'per')
        self.cfg = dict(per_logic.DEFAULT_CFG)
        for k, v in per_logic.DEFAULT_CFG.items():
            self.cfg[k] = type(v)(self.declare_parameter(k, v).value)
        self.slow_ms = float(self.declare_parameter('slow_ms', 30.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = deque(maxlen=8)
        self.create_timer(.02, self.drain, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_subscription(PointCloud2, names.DEV_LIDAR_CLOUD, self.on_cloud, LATEST_QOS)
        self.obstacle_pub = self.create_publisher(LaserScan, names.PER_OBSTACLE_SCAN, SENSOR_QOS)
        self.map_pub = self.create_publisher(LaserScan, names.PER_MAP_SCAN, SENSOR_QOS)
        self.free_pub = self.create_publisher(LaserScan, names.PER_FREE_SCAN, SENSOR_QOS)
        self.plane = np.zeros(3)
        self.no_tf_reported = False
        self.event('START', cloud_in=names.DEV_LIDAR_CLOUD, obstacle_band_m=f"{self.cfg['obstacle_min_height_m']}-{self.cfg['obstacle_max_height_m']}")

    def on_cloud(self, msg):
        self.pending.append(msg)
        self.drain()

    def drain(self):
        now = self.get_clock().now().nanoseconds
        for msg in reversed(self.pending):
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if not 0 <= now-stamp <= 10**9:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(names.FRAME_BASE_FOOTPRINT, msg.header.frame_id,
                                                    Time.from_msg(msg.header.stamp)).transform
            except TransformException:
                continue
            if self.no_tf_reported:
                self.event('TF_OK',frame=msg.header.frame_id)
                self.no_tf_reported=False
            self.process_cloud(msg, tf)
            while self.pending:
                old = self.pending[0].header.stamp
                if old.sec*10**9+old.nanosec > stamp:
                    break
                self.pending.popleft()
            return
        if self.pending and not self.no_tf_reported:
            old=self.pending[0].header.stamp
            if now-(old.sec*10**9+old.nanosec) > 500_000_000:
                self.event('NO_TF',frame=self.pending[0].header.frame_id,reason='scan_timestamp_tf_wait')
                self.no_tf_reported=True
        while self.pending:
            old = self.pending[0].header.stamp
            if now-(old.sec*10**9+old.nanosec) <= 10**9:
                break
            self.pending.popleft()

    def process_cloud(self, msg, tf):
        started = time.perf_counter()
        offsets = {f.name:f.offset for f in msg.fields}
        if not all(k in offsets for k in 'xyz') or msg.is_bigendian:
            return
        pts = per_logic.cloud_xyz(bytes(msg.data), msg.point_step, [offsets[k] for k in 'xyz'], msg.width*msg.height)
        t = tf.translation
        pts = per_logic.transform(pts, quat_to_rot(tf.rotation), (t.x,t.y,t.z))
        # A zero result means insufficient CURRENT floor support. The previous
        # plane belongs to a different body pose and must not be reused here.
        self.plane = per_logic.ground_plane(pts)
        rel = per_logic.relative_to_plane(pts, self.plane)
        obstacle, mapping, free = per_logic.scans(rel, self.geometry, self.cfg)
        for pub, ranges, rmin in ((self.obstacle_pub,obstacle,self.cfg['obstacle_min_range_m']),
                                  (self.map_pub,mapping,self.cfg['map_min_range_m']),
                                  (self.free_pub,free,0.)):
            pub.publish(self.scan_msg(msg.header.stamp,ranges,rmin))
        ms = (time.perf_counter()-started)*1000
        if ms > self.slow_ms:
            self.warn_throttled('slow', f'SLOW cloud took {ms:.1f} ms (> {self.slow_ms} ms)')


    def scan_msg(self, stamp, ranges, range_min):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = stamp, names.FRAME_BASE_FOOTPRINT
        n = len(ranges)
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / n
        s.angle_max = s.angle_min + (n - 1) * s.angle_increment
        s.scan_time = 1.0 / 15.0
        s.range_min, s.range_max = float(range_min), float(self.cfg['max_range_m'])
        s.ranges = array.array('f', ranges.astype(np.float32).tobytes())
        return s


def main():
    spin(PerScan)


if __name__ == '__main__':
    main()
