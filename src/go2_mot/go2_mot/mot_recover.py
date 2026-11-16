#!/usr/bin/env python3
"""mot_recover: recovery motions, ONLY when task_executor asks (spec 6.3). /mot_recover_cmd (go2_msgs/RecoverCmd):
  backup <m>      straight back at backup_speed_mps until <m> travelled (by /loc_odom)
  turn <rad>      relative turn in place;  turn_to <rad> absolute yaw (odom) -- also the final turn at a waypoint
  stop            abort
Output /mot_recover_cmd_vel -> mot_cmd_mux -> safe_gate (which can still stop it). One motion at a time; a new command
replaces the running one (result `replaced`). Result: event RECOVER_DONE id=.. result=ok|timeout|blocked|no_odom|replaced|stopped.
blocked = safe_gate held the command at ~0 for blocked_s.
backup_speed_mps 0.15: the dog does not walk below ~0.13 m/s (2026-10-25 wall test: 0.10-0.16 m/s commands, dog speed ~0),
and safe_gate cuts allowed speeds < 0.12 to 0; the old 0.10 would have stood still until the 15 s timeout (2026-10-26 review,
not yet seen on the dog). Same floor as mot_nav's follower (min_move 0.15).
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from go2_msgs.msg import RecoverCmd, SafeStatus

from go2_common import names
from go2_common.node import Go2Node, CMD_QOS, SENSOR_QOS, spin


MIN_BACKUP_MPS = 0.13


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MotRecover(Go2Node):
    def __init__(self):
        super().__init__('mot_recover', 'mot')
        p = self.declare_parameter
        self.backup_speed_mps = float(p('backup_speed_mps', 0.15).value)
        if not math.isfinite(self.backup_speed_mps) or self.backup_speed_mps < MIN_BACKUP_MPS:
            # Below the dog's walking floor / safe_gate's min_move the backup would stand still until the timeout.
            raise ValueError(f'backup_speed_mps {self.backup_speed_mps} < {MIN_BACKUP_MPS} m/s (cannot move the dog)')
        self.turn_speed_radps = float(p('turn_speed_radps', 0.4).value)
        self.yaw_tol_rad = float(p('yaw_tol_rad', 0.08).value)
        self.timeout_s = float(p('timeout_s', 15.0).value)
        self.blocked_s = float(p('blocked_s', 1.5).value)
        self.pose, self.job, self.blocked_since = None, None, None
        self.safe_out = None
        self.pub = self.create_publisher(Twist, names.MOT_RECOVER_CMD_VEL, CMD_QOS)
        self.create_subscription(RecoverCmd, names.MOT_RECOVER_CMD, self.on_cmd, CMD_QOS)
        self.create_subscription(Odometry, names.LOC_ODOM, self.on_odom, SENSOR_QOS)
        self.create_subscription(SafeStatus, names.SAFE_STATUS, self.on_safe, CMD_QOS)
        self.create_timer(0.05, self.tick)
        self.event('START', out=names.MOT_RECOVER_CMD_VEL, backup_speed_mps=self.backup_speed_mps)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def on_safe(self, m):
        self.safe_out = m

    def finish(self, result):
        job, self.job = self.job, None
        self.pub.publish(Twist())
        self.event('RECOVER_DONE', id=job['id'], action=job['action'], result=result, used_s=self.now_s() - job['t0'])

    def on_cmd(self, m):
        if self.job is not None:
            self.finish('replaced' if m.action != RecoverCmd.STOP else 'stopped')
        if m.action == RecoverCmd.STOP:
            return
        if m.action not in (RecoverCmd.BACKUP, RecoverCmd.TURN, RecoverCmd.TURN_TO):
            self.event('RECOVER_REJECT', id=m.id, action=m.action, reason='unknown_action')
            return
        if self.pose is None:
            self.event('RECOVER_DONE', id=m.id, action=m.action, result='no_odom', used_s=0.0)
            return
        x, y, yaw = self.pose
        target = None
        if m.action == RecoverCmd.TURN:
            target = wrap(yaw + m.value)
        elif m.action == RecoverCmd.TURN_TO:
            target = wrap(m.value)
        self.job = dict(id=m.id, action=m.action, value=m.value, start=(x, y), target=target, t0=self.now_s())
        self.blocked_since = None
        self.event('RECOVER_START', id=m.id, action=m.action, value=m.value)

    def tick(self):
        j = self.job
        if j is None or self.pose is None:
            return
        now = self.now_s()
        if now - j['t0'] > self.timeout_s:
            self.finish('timeout')
            return
        x, y, yaw = self.pose
        t = Twist()
        if j['action'] == RecoverCmd.BACKUP:
            if math.hypot(x - j['start'][0], y - j['start'][1]) >= abs(j['value']):
                self.finish('ok')
                return
            t.linear.x = -self.backup_speed_mps
        else:
            err = wrap(j['target'] - yaw)
            if abs(err) <= self.yaw_tol_rad:
                self.finish('ok')
                return
            t.angular.z = math.copysign(min(self.turn_speed_radps, max(0.15, abs(err))), err)
        s = self.safe_out
        held = s is not None and s.limiting and (abs(s.cmd_out[0]) + abs(s.cmd_out[1]) + abs(s.cmd_out[2])) < 0.02
        if held:
            self.blocked_since = self.blocked_since or now
            if now - self.blocked_since >= self.blocked_s:
                self.finish('blocked')
                return
        else:
            self.blocked_since = None
        self.pub.publish(t)


def main():
    spin(MotRecover)


if __name__ == '__main__':
    main()
