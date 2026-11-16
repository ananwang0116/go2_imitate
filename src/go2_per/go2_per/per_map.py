#!/usr/bin/env python3
"""per_map: publishes the saved 2D map as /map (nav_msgs/OccupancyGrid, frame map, transient-local, once). Replaces Nav2's
map_server + lifecycle_manager (2026-10-26). Rules in map_logic.py. Parameter yaml_filename = maps/<name>/map.yaml.
/map is read by per_evidence (the prior of /per_nav_map), loc_align and loc_map_check.
/per_hard_map (same grid, 100 = hard, 0 = none; always published, empty without maps/<name>/hard.pgm): the walls drawn by hand
in the web map editor; per_evidence keeps them at 100 whatever the laser says (glass is never seen).
"""
import array
import time

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from go2_common import names
from go2_common.node import Go2Node, spin
import numpy as np

from go2_per.map_logic import load_hard, load_map

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PerMap(Go2Node):
    def __init__(self):
        super().__init__('per_map', 'per', needs_geometry=False)
        path = self.declare_parameter('yaml_filename', '').value
        if not path:
            raise RuntimeError('per_map: parameter yaml_filename is required')
        data, res, origin = load_map(path)
        m = OccupancyGrid()
        m.header.frame_id = names.FRAME_MAP
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = res
        m.info.height, m.info.width = int(data.shape[0]), int(data.shape[1])
        m.info.origin = Pose()
        m.info.origin.position.x, m.info.origin.position.y = origin
        m.info.origin.orientation.w = 1.0
        m.data = array.array('b', data.tobytes())
        self.pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.pub.publish(m)
        hard = load_hard(path, data.shape)
        h = OccupancyGrid()
        h.header, h.info = m.header, m.info
        h.data = array.array('b', np.where(hard, 100, 0).astype(np.int8).tobytes())
        self.hard_pub = self.create_publisher(OccupancyGrid, names.PER_HARD_MAP, MAP_QOS)
        self.hard_pub.publish(h)
        self.event('MAP_PUBLISHED', path=path, width=m.info.width, height=m.info.height, resolution_m=res,
                   hard_cells=int(hard.sum()))


def main():
    spin(PerMap)


if __name__ == '__main__':
    main()
