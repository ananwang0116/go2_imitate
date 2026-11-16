#!/usr/bin/env python3
"""per_evidence: the single judge of "is there an obstacle here" (spec 2 P2, P4). Rules in evidence_logic.py.

/map (prior) + /per_hard_map (hand-drawn walls, always 100) + /per_obstacle_scan + /per_free_scan + TF map -> base_footprint
  -> /per_evidence_map (-1 unseen / 0 proven free / 100 proven occupied)
  -> /per_nav_map      (prior fused with evidence; mot_nav reads this, never /map directly)
Streaming: scans are paired by stamp and only the newest pair whose pose is already in TF is used (Leg-KILO's pose
reaches TF ~1 frame after its cloud); older pairs are dropped. Grid updates are incremental (only the rays' cells).
The maps are published every publish_period_s, only when something changed (and once at start), transient-local.
A /map with another size or origin than the first one is ignored (it used to reset the costmaps).
"""
import array
import json
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from go2_common import names
from go2_common.node import CMD_QOS, Go2Node, SENSOR_QOS, spin
from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG, frame_changed

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Live-adjustable lifetimes (seconds) and their bounds. There are no parameter services on Go2Nodes, so the web sends a String JSON.
TTL_LIMITS = {'dynamic_ttl_s': (0.5, 60.0), 'static_ttl_s': (1.0, 3600.0), 'static_min_s': (0.5, 30.0)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PerEvidence(Go2Node):
    def __init__(self):
        super().__init__('per_evidence', 'per')
        self.cfg = {k: type(v)(self.declare_parameter(k, v).value) for k, v in DEFAULT_CFG.items()}
        self.publish_period_s = float(self.declare_parameter('publish_period_s', 0.5).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg, self.prior, self.grid, self.map_key, self.hard = None, None, None, None, None
        self.map_odom = None      # (x, y, yaw) of map -> odom the evidence was written with
        self.obstacle = {}        # stamp ns -> LaserScan
        self.free = {}
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, names.PER_HARD_MAP, self.on_hard, MAP_QOS)
        self.create_subscription(LaserScan, names.PER_OBSTACLE_SCAN, lambda m: self.on_scan(self.obstacle, m), SENSOR_QOS)
        self.create_subscription(LaserScan, names.PER_FREE_SCAN, lambda m: self.on_scan(self.free, m), SENSOR_QOS)
        self.ev_pub = self.create_publisher(OccupancyGrid, names.PER_EVIDENCE_MAP, MAP_QOS)
        self.ttl_pub = self.create_publisher(String, names.PER_EVIDENCE_TTL, MAP_QOS)
        self.create_subscription(String, names.PER_EVIDENCE_TTL_CMD, self.on_ttl_cmd, CMD_QOS)
        self.nav_pub = self.create_publisher(OccupancyGrid, names.PER_NAV_MAP, MAP_QOS)
        self.create_timer(0.05, self.process)
        self.create_timer(self.publish_period_s, self.publish)
        self.event('START', out=names.PER_NAV_MAP, prior_wall_cost=self.cfg['prior_wall_cost'],
                   dynamic_ttl_s=self.cfg['dynamic_ttl_s'], static_ttl_s=self.cfg['static_ttl_s'],
                   static_min_s=self.cfg['static_min_s'])
        self.publish_ttl()

    def publish_ttl(self):
        self.ttl_pub.publish(String(data=json.dumps({k: float(self.cfg[k]) for k in TTL_LIMITS})))

    def on_ttl_cmd(self, msg):
        """The web layer panel changes how long dynamic / static obstacles stay. Validated, applied to the running grid (the same cfg
        dict), echoed on /per_evidence_ttl. Session-only: the launch defaults come back on the next start."""
        try:
            want = json.loads(msg.data)
            new = {k: float(want[k]) for k in TTL_LIMITS if k in want}
        except (ValueError, TypeError):
            self.warn_throttled('ttl_cmd', f'ignoring malformed {names.PER_EVIDENCE_TTL_CMD}: {msg.data[:80]!r}')
            return
        for k, v in new.items():
            lo, hi = TTL_LIMITS[k]
            if not math.isfinite(v) or not lo <= v <= hi:
                self.warn_throttled('ttl_cmd', f'ignoring {k}={v}: outside {lo}..{hi}')
                return
        if new:
            self.cfg.update(new)
            if self.grid is not None:
                self.grid.changed = True              # recolour / re-expire on the next publish
            self.event('EVIDENCE_TTL', **{k: self.cfg[k] for k in TTL_LIMITS})
        self.publish_ttl()

    def on_map(self, msg):
        i = msg.info
        key = (i.width, i.height, round(i.resolution, 4), round(i.origin.position.x, 3), round(i.origin.position.y, 3))
        if self.map_key is not None and key != self.map_key:
            self.warn_throttled('map_size', f'ignoring /map with another size/origin {key}, keeping {self.map_key}')
            return
        first = self.map_key is None
        self.map_key, self.map_msg = key, msg
        self.prior = np.asarray(msg.data, dtype=np.int8).reshape(i.height, i.width)
        if first:
            self.grid = EvidenceGrid(i.width, i.height, i.resolution, (i.origin.position.x, i.origin.position.y), self.cfg)
            self.grid.changed = True
            self.event('MAP', width=i.width, height=i.height, resolution_m=i.resolution)

    def on_hard(self, msg):
        i = msg.info
        self.hard = np.asarray(msg.data, dtype=np.int8).reshape(i.height, i.width) >= 65
        if self.grid is not None:
            self.grid.changed = True
        self.event('HARD_WALLS', cells=int(self.hard.sum()))

    @staticmethod
    def stamp_ns(m):
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

    def on_scan(self, store, msg):
        store[self.stamp_ns(msg)] = msg
        while len(store) > 6:
            store.pop(min(store))

    def check_alignment(self):
        """map -> odom is static but NOT constant: it is the identity until the user's initial pose is aligned (loc_align, ALIGN_OK /
        ALIGN_WEAK, typically 10-30 s after the start), then the real place. Evidence written before that sits where the room is NOT:
        in the 2026-11-09 nav run (`..._081759_nav`) 22 s of scans were painted around map (0, 0) facing x while the dog really stood at
        (1.43, 3.90) facing 150 deg, and nothing ever erased them: a ghost room of 'obstacles' in free space around the dog. So the
        evidence is cleared whenever map -> odom changes."""
        try:
            t = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_ODOM, Time()).transform
        except TransformException:
            return
        cur = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        if self.map_odom is not None:
            changed, shift, turn = frame_changed(self.map_odom, cur)
            if changed:
                cleared = self.grid.reset()
                self.obstacle.clear()
                self.free.clear()
                self.event('EVIDENCE_RESET', reason='map_to_odom_changed', shift_m=shift, turn_deg=math.degrees(turn), cells_cleared=cleared)
        self.map_odom = cur

    def process(self):
        if self.grid is None:
            return
        self.check_alignment()
        common = sorted(set(self.obstacle) & set(self.free), reverse=True)
        for st in common:
            o = self.obstacle[st]
            try:
                tf = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_BASE_FOOTPRINT, Time.from_msg(o.header.stamp))
            except TransformException:
                continue
            f = self.free[st]
            n = len(o.ranges)
            angles = o.angle_min + o.angle_increment * np.arange(n)
            t = tf.transform
            self.grid.update((t.translation.x, t.translation.y, yaw_of(t.rotation)), angles,
                             np.asarray(o.ranges, dtype=np.float64), np.asarray(f.ranges, dtype=np.float64))
            for store in (self.obstacle, self.free):              # this pair and everything older is done
                for k in [k for k in store if k <= st]:
                    store.pop(k)
            return

    def body_cells(self):
        try:
            t = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_BASE_FOOTPRINT, Time()).transform
        except TransformException:
            return None
        g = self.geometry
        yaw = yaw_of(t.rotation)
        xs = np.arange(-g.body_half_length_m, g.body_half_length_m + 1e-9, self.grid.res / 2)
        ys = np.arange(-g.body_half_width_m, g.body_half_width_m + 1e-9, self.grid.res / 2)
        bx, by = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        px = t.translation.x + c * bx.ravel() - s * by.ravel()
        py = t.translation.y + s * bx.ravel() + c * by.ravel()
        return self.grid.cells(np.column_stack((px, py)))

    def publish(self):
        if self.grid is not None:
            self.grid.expire()             # runs even when no new scans arrive; never converts old evidence to proven free
        if self.grid is None or not self.grid.changed:
            return
        self.grid.changed = False
        stamp = self.get_clock().now().to_msg()
        hard = self.hard if self.hard is not None and self.hard.shape == self.prior.shape else None
        for pub, data in ((self.ev_pub, self.grid.evidence()), (self.nav_pub, self.grid.nav(self.prior, self.body_cells(), hard))):
            m = OccupancyGrid()
            m.header.frame_id, m.header.stamp = names.FRAME_MAP, stamp
            m.info = self.map_msg.info
            m.data = array.array('b', data.tobytes())
            pub.publish(m)


def main():
    spin(PerEvidence)


if __name__ == '__main__':
    main()
