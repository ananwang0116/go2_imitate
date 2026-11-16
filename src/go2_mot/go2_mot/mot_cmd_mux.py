#!/usr/bin/env python3
"""mot_cmd_mux: choose ONE velocity source by priority: teleop > recover > nav -> /mot_cmd_vel (spec 2, L3).

A source counts while its last command is younger than source_timeout_s. Output at 20 Hz while any source is fresh;
when none is, one zero command and then silence (dev_bridge stops the dog when /safe_cmd_vel goes quiet).
Event SOURCE from=.. to=.. when the chosen source changes; "none" only after none_hold_s without any source (short
gaps while mot_nav swaps paths or a recovery step starts used to flap nav<->none several times per second, 2026-10-25).
"""
from geometry_msgs.msg import Twist

from go2_common import names
from go2_common.node import Go2Node, CMD_QOS, spin

SOURCES = (('teleop', names.DEV_TELEOP_CMD_VEL), ('recover', names.MOT_RECOVER_CMD_VEL), ('nav', names.MOT_NAV_CMD_VEL))


def choose(last, now_s, timeout_s):
    """Highest-priority fresh source name, or None. last: {name: (time_s, Twist)}."""
    for name, _ in SOURCES:
        t = last.get(name)
        if t is not None and now_s - t[0] < timeout_s:
            return name
    return None


class MotCmdMux(Go2Node):
    def __init__(self):
        super().__init__('mot_cmd_mux', 'mot', needs_geometry=False)
        self.source_timeout_s = float(self.declare_parameter('source_timeout_s', 0.5).value)
        self.none_hold_s = float(self.declare_parameter('none_hold_s', 2.0).value)
        self.reported = None
        self.none_since = None
        self.last = {}
        self.current = None
        for name, topic in SOURCES:
            self.create_subscription(Twist, topic, lambda msg, n=name: self.on_cmd(n, msg), CMD_QOS)
        self.pub = self.create_publisher(Twist, names.MOT_CMD_VEL, CMD_QOS)
        self.create_timer(0.05, self.tick)
        self.event('START', priority='teleop>recover>nav', out=names.MOT_CMD_VEL)

    def on_cmd(self, source, msg):
        self.last[source] = (self.get_clock().now().nanoseconds * 1e-9, msg)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        chosen = choose(self.last, now, self.source_timeout_s)
        if chosen != self.current:
            if chosen is None:
                self.pub.publish(Twist())          # one explicit zero when the last source goes quiet
                self.none_since = now
            self.current = chosen
        report = chosen
        if chosen is None and (self.none_since is None or now - self.none_since < self.none_hold_s):
            report = self.reported               # a short gap: keep reporting the previous source
        if report != self.reported:
            self.event('SOURCE', from_source=self.reported or 'none', to=report or 'none')
            self.reported = report
        if chosen is not None:
            self.pub.publish(self.last[chosen][1])


def main():
    spin(MotCmdMux)


if __name__ == '__main__':
    main()
