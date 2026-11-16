#!/usr/bin/env python3
"""task_executor: the ONLY node that gives goals (spec 2 P2, spec 6). Rules in task_logic.py.

In:  /goal_pose (the web console: a one-point task, or the next waypoint while collecting), /task_cmd (task_cli, web),
     missions/<name>.yaml, /loc_status, /mot_nav_status, TF map -> base_footprint
Out: /mot_nav_goal (mot_nav: own planner + follower, C++, go2_nav; replaced Nav2 on 2026-10-26), /mot_recover_cmd,
     /task_status (2 Hz), /viz_task_markers, events [task] ...
Asks: service /mot_nav_check - "can the dog reach these waypoints" is answered by mot_nav's OWN planner (one judge, P2).

Per waypoint (spec 6.3):  PENDING -> APPROACH -> ARRIVED
  APPROACH: one NavGoal to the waypoint (new id = new plan), re-sent every keepalive_s as a heartbeat. A waypoint that carries a
  taught `path` becomes a PATH goal (mot_nav follows the polyline as it is); a waypoint with a `yaw` becomes use_yaw (mot_nav
  turns on the spot after arriving). mot_nav says ARRIVED (body centre within the waypoint radius, turned if asked) or FAILED
  with a reason. Plain multi-waypoint routes use RouteExecution within this same node: real spatial progress, checked target
  occupancy, intermediate skip, final-only checked alternatives; no same-round retry pass. Loop rounds restore ORIGINAL goals.
  Single-point and taught-path tasks keep the legacy policy below.
  Trouble = mot_nav FAILED(reason) / mot_nav silent (no_nav) / no progress. Explicit reasons act after explicit_hold_s (mot_nav
  has already held them); no_progress after decision_hold_s. RECOVER runs the ladder OF THAT REASON (task_logic.LADDERS: wait,
  alt_position, backup, turn), then APPROACH again with a new goal id. Out of budget (waypoint_budget_s of non-paused time, or
  max_tries) -> SKIPPED with the reason; legacy single/teach tasks keep their one retry pass.
  /loc_status LOST -> pause (the goal is withdrawn, budget stopped); DEGRADED -> reduced speed only.
  Time budget is enforced during recovery too; after max_tries recovery steps, cancel and advance without resending the old goal.
Checks: after waypoints change (collect / load) and before a run, the waypoints go to /mot_nav_check. Plain routes reassess
  each original point every loop rather than permanently excluding a preflight BAD point; single/teach retain BAD exclusion.
  Final route candidates are dispatched only after an available affirmative response from the original planner.
  Trapped-robot ladders (local_recovery.py, shared by route AND single/teach policies): blocked = backup 0.3 m, turn;
  start_blocked = turn, relocate (checked temporary spot, then the original goal again). Judged by actual loc_pose change;
  replans never reset time/progress/ladder. Taught paths cannot relocate.
  Per-run route_decisions.jsonl includes transitions, checks, actual progress, original/candidate targets and round numbers.
Commands: collect | run_once | run_loop | undo | clear | cancel | save <name> | load <name>.
  A command stamped before this node started is dropped (CMD_STALE): /task_cmd is transient-local, and the web console's
  publisher outlives a stack it stopped, so a new task_executor was handed the old `load _web` + `run_loop` (2026-10-26 review).
Alignment: no task starts (CMD_REJECT / GOAL_REJECT reason=not_aligned) before loc_align reported ALIGN_OK or ALIGN_WEAK in
  this run: before that map -> odom is the identity guess and every target would be in the wrong place (require_alignment).
"""
import math

from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from go2_msgs.msg import Event, LocStatus, NavGoal, NavStatus, RecoverCmd, TaskCmd, TaskStatus
from go2_msgs.srv import NavCheck

from go2_common import names
from go2_common.node import Go2Node, CMD_QOS, EVENT_QOS, TASK_CMD_QOS, spin
from go2_task.task_logic import EXPLICIT_REASONS, SINGLE_LADDERS, Hold, Progress, Waypoint, WaypointRun, WpState, load_mission, save_mission
from go2_task.task_preflight import request_from
from go2_task.route_execution import RouteExecution
from go2_task.route_policy import SpatialProgress
from go2_task.local_recovery import LocalRecovery

COLORS = {WpState.PENDING: (0.7, 0.7, 0.7), WpState.APPROACH: (0.2, 0.5, 1.0), WpState.RECOVER: (1.0, 0.6, 0.0),
          WpState.ARRIVED: (0.1, 0.8, 0.2), WpState.SKIPPED: (0.6, 0.2, 0.8), WpState.BAD: (1.0, 0.1, 0.1)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TaskExecutor(Go2Node):
    def __init__(self):
        super().__init__('task_executor', 'task')
        p = self.declare_parameter
        self.missions_dir = p('missions_dir', '').value
        self.keepalive_s = float(p('keepalive_s', 1.0).value)
        self.decision_hold_s = float(p('decision_hold_s', 2.0).value)          # for no_progress (the 20 s window is already the long wait)
        self.explicit_hold_s = float(p('explicit_hold_s', 0.5).value)          # for the reasons mot_nav already held
        self.no_progress_s = float(p('no_progress_s', 20.0).value)
        self.no_progress_m = float(p('no_progress_m', 0.4).value)
        self.pass_radius_m = float(p('route_pass_radius_m', 0.6).value)      # plain routes: intermediate waypoints are passed, not stopped at
        self.detour_cfg = (float(p('detour_ratio', 1.5).value), float(p('detour_margin_m', 2.0).value),
                           float(p('detour_hold_s', 3.0).value))
        self.waypoint_budget_s = float(p('waypoint_budget_s', 90.0).value)
        self.max_tries = int(p('max_tries', 5).value)
        self.wait_s = float(p('recover_wait_s', 2.0).value)
        self.backup_m = float(p('recover_backup_m', 0.3).value)
        self.degraded_speed_pct = float(p('degraded_speed_pct', 50.0).value)
        self.require_alignment = bool(p('require_alignment', True).value)
        self.aligned = False
        self.start_s = self.now_s()
        self.waypoints, self.runs, self.bad = [], [], {}
        self.wp_ver, self.checked_ver = 0, -1
        self.check_warned = False
        self.mode, self.mission, self.index = 'idle', '', -1
        self.queue, self.retried = [], False
        self.loc_state = LocStatus.LOST
        self.paused = False
        self.target_xy = None
        self.nav_id, self.nav_seq, self.last_send_s = '', 0, -1e9
        self.nav_status, self.nav_status_s = None, -1e9
        self.speed_pct = 100.0
        self.trouble_reason = ''
        self.recover = None       # dict(step, id, until) while a recovery step runs
        self.progress = SpatialProgress(self.no_progress_s, self.no_progress_m)   # single/teach: same sliding window as routes
        self.hold_fast = Hold(self.explicit_hold_s)
        self.hold_slow = Hold(self.decision_hold_s)
        self.recover_seq = 0
        self.last_tick_s = None
        self.target_radius = None
        self.route = RouteExecution(self)
        self.local = LocalRecovery(self)      # trapped-robot ladders shared by every policy (blocked / start_blocked)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, names.GOAL_POSE, self.on_goal, CMD_QOS)
        self.create_subscription(TaskCmd, names.TASK_CMD, self.on_cmd, TASK_CMD_QOS)
        self.create_subscription(LocStatus, names.LOC_STATUS, self.on_loc_status, CMD_QOS)
        self.create_subscription(Event, names.event_topic('mot'), self.on_mot_event, EVENT_QOS)
        self.create_subscription(Event, names.event_topic('loc'), self.on_loc_event, EVENT_QOS)
        self.create_subscription(NavStatus, names.MOT_NAV_STATUS, self.on_nav_status, CMD_QOS)
        self.status_pub = self.create_publisher(TaskStatus, names.TASK_STATUS, CMD_QOS)
        self.marker_pub = self.create_publisher(MarkerArray, names.VIZ_TASK_MARKERS, CMD_QOS)
        self.recover_pub = self.create_publisher(RecoverCmd, names.MOT_RECOVER_CMD, CMD_QOS)
        self.nav_pub = self.create_publisher(NavGoal, names.MOT_NAV_GOAL, CMD_QOS)
        self.check_client = self.create_client(NavCheck, names.MOT_NAV_CHECK)
        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.event('START', goal_in=names.GOAL_POSE, cmd_in=names.TASK_CMD, arrive_radius_m=self.geometry.arrive_radius_m,
                   budget_s=self.waypoint_budget_s)

    # ---------------- inputs ----------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_loc_status(self, m):
        if m.state != self.loc_state and self.active():
            if m.state == LocStatus.LOST:
                self.event('PAUSE', reason=m.reason or 'loc_lost')
                if self.route.enabled:
                    self.route.invalidate()
                    self.route.audit('pause', reason=m.reason or 'loc_lost')
                self.local.invalidate()
                self.cancel_motion()
            elif self.loc_state == LocStatus.LOST:
                self.event('RESUME')
                self.progress.reset(self.now_s())
                if self.route.enabled:
                    self.route.resume()
                elif self.local.active:
                    self.local.resume()
        self.loc_state = m.state
        want = self.degraded_speed_pct if m.state == LocStatus.DEGRADED else 100.0
        if want != self.speed_pct:
            self.speed_pct = want
            self.event('SPEED_LIMIT', percent=want, reason='loc_degraded' if want < 100.0 else 'loc_ok')
            if self.nav_id:
                self.send_goal()                     # mot_nav takes the new speed from the next NavGoal

    def on_loc_event(self, m):
        if m.name in ('ALIGN_OK', 'ALIGN_WEAK') and not self.aligned:
            self.aligned = True
            self.event('ALIGNED', by=m.name)

    def not_aligned(self):
        return self.require_alignment and not self.aligned

    def on_nav_status(self, m):
        self.nav_status, self.nav_status_s = m, self.now_s()

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_BASE_FOOTPRINT, Time()).transform
            return t.translation.x, t.translation.y, yaw_of(t.rotation)
        except TransformException:
            return None

    def active(self):
        return self.mode in ('once', 'loop') and 0 <= self.index < len(self.runs)

    # ---------------- checks: mot_nav's own planner answers ----------------
    def changed(self):
        self.wp_ver += 1

    def check_async(self, then=None, report=None):
        """Ask mot_nav about all waypoints; fills self.bad; `then()` runs when the answer is in (also when mot_nav is not there)."""
        ver = self.wp_ver
        if not self.waypoints:
            self.bad, self.checked_ver = {}, ver
            if then:
                then()
            return
        if not self.check_client.service_is_ready():
            if not self.check_warned:
                self.check_warned = True
                self.event('CHECK_UNAVAILABLE', service=names.MOT_NAV_CHECK, reason='mot_nav_not_ready')
            self.bad, self.checked_ver = {}, ver
            if then:
                then()
            return
        pose = self.robot_pose()
        req = request_from(self.waypoints, pose[:2] if pose else None)
        self.check_client.call_async(req).add_done_callback(lambda f: self.on_check(f, ver, then, report))

    def on_check(self, fut, ver, then, report):
        if ver != self.wp_ver:
            return                                              # the waypoints changed meanwhile: a newer check is on its way
        res = fut.result() if fut.done() and not fut.cancelled() else None
        if res is None or not res.available:
            if not self.check_warned:
                self.check_warned = True
                self.event('CHECK_UNAVAILABLE', service=names.MOT_NAV_CHECK, reason='no_costmap_yet')
            self.bad = {}
        else:
            self.bad = {k: [r] for k, r in enumerate(res.reason) if r}
        self.checked_ver = ver
        if report is not None and report < len(self.waypoints):
            w, problems = self.waypoints[report], self.bad.get(report, [])
            self.event('WP_ADD', wp=report + 1, x_m=w.x, y_m=w.y, ok=not problems, problems='; '.join(problems) or 'none')
            if problems:
                self.get_logger().warn(f'[task] wp{report + 1} failed the check: {"; ".join(problems)}')
        self.publish_markers()
        if then:
            then()

    # ---------------- commands ----------------
    def make_wp(self, msg, name):
        q = msg.pose.orientation
        none = q.x == q.y == q.z == q.w == 0.0                  # an all-zero quaternion = "no preferred heading" (no final turn)
        return Waypoint(name=name, x=msg.pose.position.x, y=msg.pose.position.y,
                        radius_m=self.geometry.arrive_radius_m, yaw=None if none else yaw_of(q))

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != names.FRAME_MAP:
            self.event('GOAL_REJECT', reason=f'frame_{msg.header.frame_id}')
            return
        if self.mode == 'collecting':
            self.waypoints.append(self.make_wp(msg, f'wp{len(self.waypoints) + 1}'))
            self.changed()
            self.check_async(report=len(self.waypoints) - 1)
            self.publish_markers()
            return
        if self.not_aligned():
            self.event('GOAL_REJECT', reason='not_aligned')
            self.get_logger().warn('[task] goal refused: localise first (point the dog on the map, then wait for ALIGN_OK)')
            return
        self.cancel_task('new_goal')
        self.waypoints, self.bad, self.mission = [self.make_wp(msg, 'goal')], {}, ''
        self.changed()
        self.start('once', check=False)      # a single clicked goal is tried even if the check would complain

    def on_cmd(self, m):
        c, arg = m.command, m.arg
        sent_s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if sent_s < self.start_s - 1.0:                    # replayed from before this node existed (transient-local)
            self.event('CMD_STALE', command=c, arg=arg or '-', age_s=self.start_s - sent_s)
            return
        self.event('CMD', command=c, arg=arg or '-')
        if c == TaskCmd.COLLECT:
            self.cancel_task('collect')
            self.mode, self.waypoints, self.bad, self.mission = 'collecting', [], {}, ''
            self.changed()
        elif c in (TaskCmd.RUN_ONCE, TaskCmd.RUN_LOOP):
            if not self.waypoints:
                self.event('CMD_REJECT', command=c, reason='no_waypoints')
                return
            if self.not_aligned():
                self.event('CMD_REJECT', command=c, reason='not_aligned')
                return
            self.start('once' if c == TaskCmd.RUN_ONCE else 'loop')
        elif c == TaskCmd.UNDO:
            if self.waypoints and not self.active():
                self.waypoints.pop()
                self.bad.pop(len(self.waypoints), None)
                self.changed()
        elif c == TaskCmd.CLEAR:
            self.cancel_task('clear')
            self.waypoints, self.bad, self.mode = [], {}, 'idle'
            self.changed()
        elif c == TaskCmd.CANCEL:
            self.cancel_task('cancel')
        elif c == TaskCmd.SAVE:
            if not arg or not self.waypoints:
                self.event('CMD_REJECT', command=c, reason='need_name_and_waypoints')
                return
            path = f'{self.missions_dir}/{arg}.yaml'
            save_mission(path, self.waypoints)
            self.event('SAVED', mission=arg, waypoints=len(self.waypoints), path=path)
        elif c == TaskCmd.LOAD:
            try:
                wps = load_mission(f'{self.missions_dir}/{arg}.yaml', self.geometry.arrive_radius_m)
            except (OSError, ValueError) as e:
                self.event('CMD_REJECT', command=c, reason=str(e)[:100])
                return
            self.cancel_task('load')
            self.waypoints, self.mission, self.mode, self.bad = wps, arg, 'idle', {}
            self.changed()
            self.check_async(then=lambda: self.event('LOADED', mission=arg, waypoints=len(wps), bad=len(self.bad)))
        else:
            self.event('CMD_REJECT', command=c, reason='unknown')
        self.publish_markers()

    def start(self, mode, check=True, next_round=False):
        if check and self.checked_ver != self.wp_ver:
            self.check_async(then=lambda: self.start(mode, check=False))
            return
        self.cancel_motion()
        self.mode, self.index, self.target_xy = mode, -1, None
        self.route.begin_round(continuing=next_round)
        self.runs = [WaypointRun(k, self.waypoint_budget_s, self.max_tries,
                                 state=WpState.BAD if k in self.bad and not self.route.enabled else WpState.PENDING)
                     for k in range(len(self.waypoints))]
        for r in self.runs:
            r.trouble_budget = not self.route.enabled      # single/teach: the 90 s budget counts problem time only
        self.queue = [r.index for r in self.runs if r.state == WpState.PENDING]
        self.retried, self.index = False, -1
        self.event('TASK_START', mode=mode, mission=self.mission or '-', waypoints=len(self.waypoints), bad=len(self.bad))
        self.next_waypoint()

    def cancel_task(self, reason):
        if self.route.enabled:
            self.route.audit('cancel', reason=reason)
        self.route.invalidate()
        self.local.reset()
        if self.active():
            self.event('TASK_CANCEL', reason=reason, wp=self.index + 1)
        self.cancel_motion()
        if self.mode in ('once', 'loop'):
            self.mode = 'idle'
        self.index, self.queue = -1, []
        self.publish_markers()

    def cancel_motion(self):
        if self.nav_id:
            self.nav_pub.publish(NavGoal(id=self.nav_id, active=False))
            self.nav_id = ''
        if self.recover is not None:
            self.recover_pub.publish(RecoverCmd(id='cancel', action=RecoverCmd.STOP))
            self.recover = None

    # ---------------- waypoint sequencing ----------------
    def next_waypoint(self):
        self.cancel_motion()
        self.route.invalidate()
        self.local.reset()
        if not self.queue:
            skipped = [r.index for r in self.runs if r.state == WpState.SKIPPED]
            if skipped and not self.retried and not self.route.enabled:
                self.retried, self.queue = True, skipped
                for k in skipped:
                    r = self.runs[k]
                    r.state, r.used_s, r.tries, r.steps_by_reason = WpState.PENDING, 0.0, 0, {}
                self.event('ROUND_RETRY', skipped=len(skipped))
            else:
                self.end_round()
                return
        k = self.queue.pop(0)
        self.index = k
        self.runs[k].state = WpState.APPROACH
        w = self.waypoints[k]
        self.target_xy = (w.x, w.y)
        self.target_radius = self.effective_radius(k)
        self.progress.reset(self.now_s())
        self.hold_fast, self.hold_slow = Hold(self.explicit_hold_s), Hold(self.decision_hold_s)
        self.trouble_reason = ''
        self.route.begin_waypoint()
        self.new_nav_goal()
        if self.route.enabled:
            self.route.audit('goal_sent', reason='original_waypoint')
        self.event('APPROACH', wp=k + 1, name=w.name, x_m=self.target_xy[0], y_m=self.target_xy[1],
                   path_pts=len(w.path) if w.path else 0, yaw=w.yaw if w.yaw is not None else 'none')
        self.publish_markers()

    def end_round(self):
        summary = {s.value.lower(): sum(r.state == s for r in self.runs) for s in (WpState.ARRIVED, WpState.SKIPPED, WpState.BAD)}
        self.event('ROUND_DONE', mode=self.mode, **summary)
        if self.route.enabled:
            self.route.audit('round_done', summary=summary, extra_retry=False)
        if self.mode == 'loop' and summary['arrived'] > 0:
            self.start('loop', check=False, next_round=True)
            return
        if self.mode == 'loop':
            self.event('LOOP_STOP', reason='no_waypoint_reached_in_round')
        self.mode, self.index = 'idle', -1
        self.publish_markers()

    def finish_waypoint(self, state, reason=''):
        run = self.runs[self.index]
        run.state, run.reason = state, reason
        if self.route.enabled:
            self.route.transition(state.value.lower(), reason or state.value.lower())
        w = self.waypoints[self.index]
        if state == WpState.ARRIVED:
            self.event('ARRIVED', wp=self.index + 1, name=w.name, used_s=run.used_s, tries=run.tries)
        else:
            self.event('SKIP', wp=self.index + 1, name=w.name, reason=reason, used_s=run.used_s, tries=run.tries)
            self.get_logger().warn(f'[task] SKIP wp{self.index + 1}: {reason}')
        self.next_waypoint()

    # ---------------- mot_nav ----------------
    def send_goal(self):
        w = self.waypoints[self.index]
        m = NavGoal(id=self.nav_id, active=True, speed_pct=self.speed_pct)
        m.header.stamp = self.get_clock().now().to_msg()
        m.x, m.y = self.target_xy
        m.radius_m = self.target_radius if self.target_radius is not None else w.radius_m
        if w.yaw is not None and not self.local.relocating and self.heading_enforced(self.index):
            m.use_yaw, m.yaw = True, w.yaw          # a temporary relocation spot has no heading
        if w.path and self.target_xy == (w.x, w.y):
            m.follow_path = True
            m.path_x = [float(q[0]) for q in w.path]
            m.path_y = [float(q[1]) for q in w.path]
        self.nav_pub.publish(m)
        self.last_send_s = self.now_s()

    def heading_enforced(self, k):
        """Plain routes only stop and turn to the heading at the FINAL waypoint; intermediate ones are passed."""
        return not self.route.enabled or k == len(self.waypoints)-1

    def effective_radius(self, k):
        """Arrival radius. Intermediate waypoints of a plain route count as passed inside route_pass_radius_m."""
        w = self.waypoints[k]
        return w.radius_m if self.heading_enforced(k) else max(w.radius_m, self.pass_radius_m)

    def new_nav_goal(self):
        """A new id makes mot_nav plan from scratch (new waypoint, alternative position, after a recovery)."""
        self.nav_seq += 1
        self.nav_id = f'g{self.nav_seq}'
        self.send_goal()

    def nav_for_me(self):
        s = self.nav_status
        return s if s is not None and self.nav_id and s.id == self.nav_id else None

    def nav_remaining(self, robot):
        s = self.nav_for_me()
        if s is not None and math.isfinite(s.remaining_m):
            return s.remaining_m
        w = self.waypoints[self.index]
        return math.hypot(robot[0] - w.x, robot[1] - w.y)

    def current_trouble(self, now, progressing):
        """Why the dog is not getting there right now ('' = fine): mot_nav's reason, mot_nav gone, or no progress."""
        if now - self.nav_status_s > 2.0:
            self.warn_throttled('no_nav', 'mot_nav is not publishing /mot_nav_status (is it running?)')
            return 'no_nav'
        s = self.nav_for_me()
        if s is not None and s.state == NavStatus.FAILED:
            return s.reason or 'nav_failed'
        return '' if progressing else 'no_progress'

    def on_mot_event(self, m):
        if m.name != 'RECOVER_DONE':
            return
        f = {kv.key: kv.value for kv in m.fields}
        if self.local.owns(f.get('id')):
            self.local.on_recover_done(f)           # judged by the actual localisation change
            return
        if self.recover is not None and f.get('id') == self.recover.get('id'):
            self.event('RECOVER_RESULT', wp=self.index + 1, step=self.recover['step'], result=f.get('result', '?'))
            self.recover = None
            self.back_to_approach()

    def back_to_approach(self):
        if not self.active():
            return
        if self.runs[self.index].out_of_budget():
            self.finish_waypoint(WpState.SKIPPED, reason='budget:recovery_limit')
            return
        self.runs[self.index].state = WpState.APPROACH
        self.progress.reset(self.now_s())
        self.trouble_reason = ''
        self.new_nav_goal()
        self.publish_markers()

    def alt_target(self, run, w):
        """Another spot inside the waypoint area: a ring at half the radius, a new angle every time (mot_nav snaps it to a
        passable cell or reports goal_blocked)."""
        n = len(run.tried_positions)
        ang = n * math.pi / 2 + math.pi / 4
        r = 0.5 * w.radius_m
        return (w.x + r * math.cos(ang), w.y + r * math.sin(ang))

    def start_recovery(self, reason):
        run = self.runs[self.index]
        w = self.waypoints[self.index]
        step = run.next_recovery(reason)
        if step == 'alt_position' and w.path:
            step = 'wait'                                      # a taught path has no "other spot"
        run.state = WpState.RECOVER
        self.cancel_motion()
        self.recover = dict(step=step, id='', until=None)
        self.event('RECOVER', wp=self.index + 1, step=step, trigger=reason, tries=run.tries, used_s=run.used_s)
        if step == 'wait':
            self.recover['until'] = self.now_s() + self.wait_s          # let the evidence map update
        elif step == 'alt_position':
            run.tried_positions.append(self.target_xy)
            self.target_xy = self.alt_target(run, w)
            self.recover['until'] = self.now_s() + 0.2
        else:
            self.recover_seq += 1
            rid = f'r{self.recover_seq}'
            self.recover['id'] = rid
            self.recover_pub.publish(RecoverCmd(id=rid, action=RecoverCmd.BACKUP if step == 'backup' else RecoverCmd.TURN,
                                                value=self.backup_m if step == 'backup' else math.pi / 2))
        self.hold_fast.last_decision = self.hold_slow.last_decision = self.now_s()
        self.trouble_reason = ''
        self.publish_markers()

    # ---------------- main loop (5 Hz) ----------------
    def tick(self):
        now = self.now_s()
        dt = 0.0 if self.last_tick_s is None else min(now - self.last_tick_s, 1.0)
        self.last_tick_s = now
        if not self.active():
            return
        run = self.runs[self.index]
        self.paused = self.loc_state == LocStatus.LOST
        stalled = run.used_s-self.progress.last_progress_s > 5.0 if self.progress.samples else False
        trouble = run.trouble_budget and (self.recover is not None or self.local.active or run.state == WpState.RECOVER
                                          or bool(self.trouble_reason) or stalled)
        run.tick(dt, self.paused, trouble)
        if self.paused:
            return
        if run.trouble_budget:
            stage = run.trouble_stage()
            if stage == 2:
                self.event('TROUBLE_BUDGET', stage='abandon', wp=self.index + 1, trouble_s=round(run.trouble_s, 1),
                           used_s=round(run.used_s, 1))
                self.retried = True                       # no ROUND_RETRY: both budgets are spent
                self.finish_waypoint(WpState.SKIPPED, reason='trouble_budget_exhausted')
                return
            if stage == 1:
                # First budget spent: stop, tell the user, continue once (ladders start over). trouble_s keeps running.
                run.stage_one_done = True
                self.event('TROUBLE_BUDGET', stage='retry', wp=self.index + 1, trouble_s=round(run.trouble_s, 1),
                           used_s=round(run.used_s, 1))
                self.cancel_motion()
                self.local.reset()
                self.recover = None
                run.tries, run.steps_by_reason, run.state = 0, {}, WpState.APPROACH
                self.trouble_reason = ''
                self.progress.reset(self.now_s())
                self.hold_fast, self.hold_slow = Hold(self.explicit_hold_s), Hold(self.decision_hold_s)
                self.publish_markers()
                return
        # LOST pauses above; DEGRADED never removes the per-waypoint limits.
        # Let the final recovery finish, but enforce the time deadline even inside it.
        if (not run.trouble_budget and run.used_s >= run.budget_s) or (
                not self.route.enabled and self.recover is None and run.out_of_budget()):
            self.finish_waypoint(WpState.SKIPPED, reason=f'budget:{self.trouble_reason or "limit"}')
            return
        if self.route.enabled:
            self.route.tick(now, run)
            return
        if self.local.active:
            robot = self.robot_pose()
            if robot is None:
                self.cancel_motion()
                return
            self.local.tick(now, run, robot)
            return
        if self.recover is not None:
            if self.recover['until'] is not None and now >= self.recover['until']:
                self.recover = None
                self.back_to_approach()
            return
        robot = self.robot_pose()
        if robot is None:
            return
        if not self.nav_id:
            self.new_nav_goal()                                   # after a pause (loc LOST) the goal was withdrawn
        elif now - self.last_send_s >= self.keepalive_s:
            self.send_goal()
        s = self.nav_for_me()
        if s is not None and s.state == NavStatus.ARRIVED:
            self.finish_waypoint(WpState.ARRIVED)
            return
        progressing = self.progress.update(run.used_s, robot[:2])
        self.trouble_reason = self.current_trouble(now, progressing)
        explicit = self.trouble_reason in EXPLICIT_REASONS
        fire_fast = self.hold_fast.update(now, bool(self.trouble_reason) and explicit)
        fire_slow = self.hold_slow.update(now, bool(self.trouble_reason) and not explicit)
        if fire_fast or fire_slow:
            if self.trouble_reason in SINGLE_LADDERS:
                self.local.begin(self.trouble_reason)        # shared ladder, actual-displacement judged
            else:
                self.start_recovery(self.trouble_reason)

    # ---------------- output ----------------
    def publish_status(self):
        s = TaskStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.mission, s.mode = self.mission, self.mode
        s.waypoint_index, s.waypoint_count = self.index, len(self.waypoints)
        s.remaining_m = math.nan
        if self.active():
            r = self.runs[self.index]
            s.waypoint_state, s.budget_used_s, s.tries = r.state.value, r.used_s, r.tries
            robot = self.robot_pose()
            if robot and self.nav_id:
                s.remaining_m = self.nav_remaining(robot)
        self.status_pub.publish(s)

    def publish_markers(self):
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        stamp = self.get_clock().now().to_msg()
        running = self.mode in ('once', 'loop') and len(self.runs) == len(self.waypoints)
        for k, w in enumerate(self.waypoints):
            st = self.runs[k].state if running else (WpState.BAD if k in self.bad else WpState.PENDING)
            col = COLORS[st]
            area = Marker(ns='wp', id=2 * k, type=Marker.CYLINDER, action=Marker.ADD)
            area.header.frame_id, area.header.stamp = names.FRAME_MAP, stamp
            area.pose.position.x, area.pose.position.y = w.x, w.y
            area.pose.orientation.w = 1.0
            area.scale.x = area.scale.y = 2 * self.effective_radius(k)
            area.scale.z = 0.02
            area.color.r, area.color.g, area.color.b = col
            area.color.a = 0.45
            text = Marker(ns='wp', id=2 * k + 1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            text.header = area.header
            text.pose.position.x, text.pose.position.y, text.pose.position.z = w.x, w.y, 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'{k + 1} {st.value}' + (f': {self.bad[k][0]}' if k in self.bad else '')
            arr.markers += [area, text]
            if w.yaw is not None:                  # the heading the user gave (enforced only at a final/single waypoint)
                arrow = Marker(ns='wp_yaw', id=k, type=Marker.ARROW, action=Marker.ADD)
                arrow.header = area.header
                arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = w.x, w.y, 0.05
                arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(w.yaw / 2), math.cos(w.yaw / 2)
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 0.08, 0.08
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.11, 0.13, 0.19, 1.0
                arr.markers.append(arrow)
        self.marker_pub.publish(arr)


def main():
    spin(TaskExecutor)


if __name__ == '__main__':
    main()
