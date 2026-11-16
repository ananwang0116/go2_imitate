"""Go2Node: the base class of every go2_v2 node.

Gives each node the same three things: the shared geometry (one file, spec 3), events (spec 8: INFO line +
/<layer>_event + events.log), and WARN throttled per reason (>= 10 s).
"""
import os
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from diagnostic_msgs.msg import KeyValue
from go2_msgs.msg import Event

from . import geometry as geometry_mod
from .events import format_event, format_value, append_event_line
from .names import event_topic

GEOMETRY_ENV = 'GO2_GEOMETRY_FILE'   # set by go2.launch.py; the geometry_file parameter overrides it

# Sensor streams (dog, Leg-KILO): Best Effort. A Reliable subscriber never matches a Best Effort publisher.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
# Commands and status: Reliable, shallow.
CMD_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
# Events: Reliable and transient-local so a late subscriber (the web console, ros2 topic echo) still sees the last ones.
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
# /task_cmd: task_cli is a short-lived process. Transient-local keeps the command until task_executor has matched
# and acknowledged it (with volatile durability a command sent before the reader side matched was lost, 2026-10-25).
TASK_CMD_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
WARN_THROTTLE_S = 10.0


class Go2Node(Node):
    def __init__(self, name: str, layer: str, needs_geometry: bool = True):
        # no parameter services: 6 more entities in the executor's wait set that rclpy rebuilds for EVERY message (measured, docs/02 session 34);
        # launch / command-line parameters (-p) are still applied at start, only `ros2 param get/set` on a running node is gone
        super().__init__(name, start_parameter_services=False)
        self.layer = layer
        self._event_pub = self.create_publisher(Event, event_topic(layer), EVENT_QOS)
        self._last_warn = {}
        self.geometry = None
        if needs_geometry:
            path = self.declare_parameter('geometry_file', os.environ.get(GEOMETRY_ENV, '')).value
            if not path:
                raise RuntimeError(f'{name}: no geometry file (parameter geometry_file or env {GEOMETRY_ENV})')
            self.geometry = geometry_mod.load(path)
            problems = geometry_mod.check(self.geometry)
            if problems:
                raise RuntimeError(f'{name}: {path} violates: ' + '; '.join(problems))

    def event(self, name: str, /, **fields):
        """One decision or state change. Never call this periodically. `name` is positional-only, so an event may carry
        a field called name= (a task_executor crash on 2026-10-25: 'got multiple values for argument name')."""
        text = format_event(self.layer, name, fields)
        self.get_logger().info(text)
        msg = Event()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.layer, msg.name = self.layer, name
        msg.fields = [KeyValue(key=k, value=format_value(v)) for k, v in fields.items()]
        self._event_pub.publish(msg)
        append_event_line(text, self.get_clock().now().nanoseconds * 1e-9)

    def warn_throttled(self, reason: str, text: str):
        """WARN, at most once per WARN_THROTTLE_S for the same reason."""
        now = time.monotonic()
        if now - self._last_warn.get(reason, -1e9) >= WARN_THROTTLE_S:
            self._last_warn[reason] = now
            self.get_logger().warn(f'[{self.layer}] {text}')


def spin(node_cls):
    """Standard main(): spin until Ctrl-C, then destroy quietly (shutdown noise is not an error)."""
    rclpy.init()
    node = None
    try:
        node = node_cls()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # Ctrl-C shuts the context down while a timer callback may still be publishing: that raises "context is invalid"
        # and used to end the node with exit code 1 (board, 2026-11-04: dev_bridge, dev_mid360, safe_gate). Only after
        # the shutdown is it noise; while the context is alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            if hasattr(node, 'on_shutdown'):
                node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
