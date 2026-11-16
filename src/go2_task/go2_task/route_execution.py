"""Policy within the sole TaskExecutor: skip intermediate failures this round;
check nearby alternatives only for the original final waypoint. No extra node,
goal publisher, map classifier or planner. Single-point/teach policies are separate.

Trapped-robot failures (mot_nav blocked / start_blocked) are NOT target failures: they go to the shared
go2_task.local_recovery.LocalRecovery (one ladder definition and executor for every policy). This class is its host:
resume_goal() resumes the unchanged goal without touching elapsed time or the spatial-progress window, and an exhausted
ladder falls to fail() (intermediate skip / final alternative).
"""
import math
import time

from go2_msgs.msg import NavStatus
from go2_task.task_logic import LOCAL_LADDERS, Waypoint, WpState
from go2_task.task_preflight import request_from
from go2_task.route_policy import DetourGuard, SpatialProgress, append_route_audit, final_candidates


class RouteExecution:
    ladders = LOCAL_LADDERS         # trapped-robot ladders only: everything else is a failure of the waypoint

    def __init__(self, owner):
        self.node = owner
        p = owner.declare_parameter
        self.stall_s = float(p('route_no_progress_s', 20.).value)
        self.progress_m = float(p('route_progress_m', .4).value)
        self.alt_offset = float(p('final_alternative_offset_m', 1.).value)
        self.settle_m = float(p('route_settle_m', .15).value)      # final waypoint: this close outside its radius is settling, not stalled
        if any(not math.isfinite(v) or v <= 0 for v in (self.stall_s, self.progress_m, self.alt_offset)):
            raise ValueError('route progress thresholds and final alternative offset must be finite and positive')
        self.check_timeout_s = 2.
        self.enabled = False
        self.round = 0
        self.task_sequence = 0
        self.generation = 0
        self.pending = None
        self.phase = 'idle'
        self.progress = SpatialProgress(self.stall_s, self.progress_m)
        self.detour = DetourGuard(*owner.detour_cfg)
        self.next_snapshot = 0.

    # ---- host interface of LocalRecovery ----
    def can_relocate(self):
        return True

    def resume_goal(self, reason):
        """Back to the unchanged goal with a new id. Elapsed time, spatial progress and ladder position are kept."""
        n = self.node
        n.runs[n.index].state = WpState.APPROACH
        # Stationary recovery steps fill the window; a bounded 10 s grace lets the unchanged goal move the dog again.
        self.progress_grace_until = n.runs[n.index].used_s+10.
        self.transition('alternative_approach' if self.alt_active else 'approach', reason)
        n.new_nav_goal()
        self.audit('goal_sent', reason=reason)
        n.hold_fast.last_decision = n.hold_slow.last_decision = n.now_s()
        n.publish_markers()

    def exhausted(self, reason):
        self.fail(f'{reason}_recovery_exhausted')

    def audit(self, kind, **fields):
        n = self.node
        valid = 0 <= n.index < len(n.waypoints)
        run = n.runs[n.index] if valid and n.index < len(n.runs) else None
        w = n.waypoints[n.index] if valid else None
        remaining = n.nav_for_me()
        remaining = remaining.remaining_m if remaining else None
        if remaining is not None and not math.isfinite(remaining):
            remaining = None
        row = dict(kind=kind, sim_s=n.now_s(), mission=n.mission, mode=n.mode,
                   task_sequence=self.task_sequence, round=self.round,
                   waypoint=n.index+1 if valid else None, phase=self.phase,
                   goal_id=n.nav_id, original=[w.x, w.y] if w else None,
                   target=list(n.target_xy) if valid and n.target_xy else None,
                   used_s=run.used_s if run else None, tries=run.tries if run else None,
                   stall_s=max(0., run.used_s-self.progress.last_progress_s) if run else None,
                   remaining_m=remaining, local_steps=dict(n.local.steps), **fields)
        try:
            append_route_audit(row)
        except (OSError, ValueError) as exc:
            n.warn_throttled('route_audit', f'route_decisions.jsonl: {exc}')

    def transition(self, phase, reason):
        previous, self.phase = self.phase, phase
        self.audit('transition', from_phase=previous, to_phase=phase, reason=reason)
        self.node.event('ROUTE_STATE', round=self.round, wp=self.node.index+1,
                        from_phase=previous, to_phase=phase, reason=reason)

    def invalidate(self):
        self.node.local.invalidate()
        self.generation += 1
        if self.pending is not None:
            self.pending['future'].cancel()
        self.pending = None

    def begin_round(self, continuing=False):
        self.invalidate()
        n = self.node
        self.enabled = len(n.waypoints) > 1 and not any(w.path for w in n.waypoints)
        self.round = self.round+1 if continuing else 1
        if not continuing:
            self.task_sequence += 1
        self.phase = 'idle'
        if self.enabled:
            self.audit('round_start', reason='next_loop_original_waypoints' if continuing else 'start',
                       original_waypoints=[[w.x, w.y] for w in n.waypoints],
                       no_progress_limit_s=self.stall_s, progress_threshold_m=self.progress_m,
                       alternative_offset_m=self.alt_offset, detour_ratio=self.detour.ratio,
                       detour_margin_m=self.detour.margin_m, pass_radius_m=n.pass_radius_m)

    def begin_waypoint(self):
        self.invalidate()
        self.progress.reset()
        self.detour.reset()
        self.next_check = self.next_snapshot = 0.
        self.occupied_since = None
        self.candidates = None
        self.search_until = None
        self.alt_active = False
        self.progress_grace_until = 0.
        self.node.local.reset()
        if self.enabled:
            self.transition('approach', 'original_waypoint')

    def resume(self):
        # The waypoint's used_s and spatial history were frozen during LOST.
        if self.phase == 'alternative_search':
            self.search_until = time.monotonic()+8.
        if self.node.local.active:
            self.node.local.resume()
        self.occupied_since = None
        self.audit('resume', reason='localisation_restored')

    def fail(self, reason):
        n = self.node
        self.audit('failure', reason=reason)
        if n.index != len(n.waypoints)-1:
            n.finish_waypoint(WpState.SKIPPED, reason=reason)
            return
        if n.runs[n.index].tries >= n.runs[n.index].max_tries:
            n.finish_waypoint(WpState.SKIPPED, reason='final_alternative_attempt_limit')
            return
        self.invalidate()
        n.cancel_motion()
        robot = n.robot_pose()
        if robot is None:
            # Missing localisation must not be labelled an occupied target.
            self.transition('await_pose', 'alternative_requires_pose')
            return
        if self.candidates is None:
            self.candidates = final_candidates(n.waypoints[n.index], robot, self.alt_offset)
        if not self.candidates:
            n.finish_waypoint(WpState.SKIPPED, reason='final_alternatives_exhausted')
            return
        self.search_until = time.monotonic()+8.
        n.runs[n.index].state = WpState.RECOVER
        self.transition('alternative_search', reason)

    def submit(self, kind, xy, radius, robot):
        n = self.node
        if not n.check_client.service_is_ready():
            return False
        points = [Waypoint('route_check', xy[0], xy[1], radius)]
        future = n.check_client.call_async(request_from(points, robot[:2]))
        self.pending = dict(future=future, generation=self.generation, index=n.index,
                            kind=kind, xy=xy, radius=radius, deadline=time.monotonic()+self.check_timeout_s)
        self.audit('check_request', check_kind=kind, candidate=list(xy), radius_m=radius,
                   check_start=list(robot[:2]))
        return True

    def poll(self):
        n = self.node
        pending = self.pending
        if pending is None:
            return
        future = pending['future']
        if not future.done() and time.monotonic() < pending['deadline']:
            return
        self.pending = None
        if pending['generation'] != self.generation or pending['index'] != n.index:
            future.cancel()
            return
        try:
            res = future.result() if future.done() and not future.cancelled() else None
        except Exception as exc:
            res = None
            self.audit('check_error', reason=str(exc))
        if res is None or not res.available or len(res.ok) != 1 or len(res.reason) != 1:
            future.cancel()
            self.occupied_since = None
            self.audit('check_unavailable', check_kind=pending['kind'])
            return
        ok, reason = bool(res.ok[0]), res.reason[0]
        leg = float(res.leg_m[0]) if len(res.leg_m) == 1 else None
        if ok and pending['kind'] == 'alternative' and leg is not None and not self.detour.allows(leg):
            ok, reason = False, 'detour_too_long'     # a candidate that needs the same big detour is no alternative
        self.audit('check_result', check_kind=pending['kind'], candidate=list(pending['xy']),
                   ok=ok, reason=reason, leg_m=leg)
        if pending['kind'] == 'target':
            # This answer is about the target AREA, not an obstacle along the way.
            occupied = not ok and reason.startswith('no passable position in the area')
            if occupied:
                used = n.runs[n.index].used_s
                if self.occupied_since is None:
                    self.occupied_since = used
                elif used-self.occupied_since >= 1.:
                    self.fail('target_area_occupied')
            else:
                self.occupied_since = None
                if ok and self.detour.update(n.runs[n.index].used_s, leg):
                    self.audit('detour', leg_m=leg, best_m=self.detour.best, limit_m=self.detour.limit())
                    self.fail('detour_too_long')
                    return
        elif ok:
            n.target_xy = pending['xy']
            n.target_radius = pending['radius']
            n.runs[n.index].tries += 1
            n.runs[n.index].state = WpState.APPROACH
            self.alt_active = True
            # A bounded grace permits the new candidate to move the robot. It
            # does not erase the spatial history or grant another full wait.
            self.progress_grace_until = n.runs[n.index].used_s+10.
            self.occupied_since = None
            self.next_check = n.now_s()+1.
            self.transition('alternative_approach', 'candidate_reachable')
            n.new_nav_goal()
            self.audit('goal_sent', reason='checked_final_alternative')
            n.publish_markers()

    def settling(self, robot):
        """Only the final waypoint (or its alternative): hovering within settle_m beyond the arrival radius is not a stall.
        Intermediate points have no such exemption: they are passed or skipped."""
        n = self.node
        if n.index != len(n.waypoints)-1 or n.target_xy is None or n.target_radius is None:
            return False
        return math.hypot(robot[0]-n.target_xy[0], robot[1]-n.target_xy[1]) <= n.target_radius+self.settle_m

    def tick(self, now, run):
        n = self.node
        robot = n.robot_pose()
        if robot is None:
            n.cancel_motion()
            return
        progressing = self.progress.update(run.used_s, robot[:2])
        if not progressing and self.settling(robot):
            progressing = True      # fine positioning near the final target (radius 0.1-0.4 m); the 90 s budget bounds it
        if now >= self.next_snapshot:
            self.next_snapshot = now+1.
            self.audit('progress', pose=list(robot), progressing=progressing,
                       window_samples=len(self.progress.samples))
        if self.phase == 'await_pose':
            self.fail('pose_restored_retry_alternative')
            return
        if n.local.active:
            n.local.tick(now, run, robot)
            return
        status = n.nav_for_me()
        if status is not None and now-n.nav_status_s <= 2. and status.state == NavStatus.ARRIVED:
            n.finish_waypoint(WpState.ARRIVED, reason='final_alternative' if self.alt_active else '')
            return
        self.poll()
        if not n.active() or n.runs[n.index] is not run:
            return
        if self.phase == 'alternative_search':
            if time.monotonic() >= self.search_until:
                n.finish_waypoint(WpState.SKIPPED, reason='alternative_check_deadline')
                return
            if self.pending is None:
                if not self.candidates:
                    n.finish_waypoint(WpState.SKIPPED, reason='no_reachable_final_alternative')
                    return
                xy = self.candidates[0]
                radius = max(n.geometry.loc_sigma_m, min(.15, n.waypoints[n.index].radius_m))
                if self.submit('alternative', xy, radius, robot):
                    self.candidates.pop(0)
            return
        if now-n.nav_status_s > 2.:
            # Infrastructure silence is not evidence of a blocked waypoint.
            if n.nav_id:
                self.audit('feedback_stop', reason='no_nav')
                n.cancel_motion()
            n.warn_throttled('no_nav', 'route waiting for mot_nav status')
            return
        if not n.nav_id:
            n.new_nav_goal()
            self.audit('goal_sent', reason='resume')
        elif now-n.last_send_s >= n.keepalive_s:
            n.send_goal()
        status = n.nav_for_me()
        if status is not None and status.state == NavStatus.ARRIVED:
            n.finish_waypoint(WpState.ARRIVED, reason='final_alternative' if self.alt_active else '')
            return
        if status is not None and status.state == NavStatus.FAILED:
            if n.hold_fast.update(now, True):
                reason = status.reason or 'nav_failed'
                if reason in LOCAL_LADDERS:
                    n.local.begin(reason)       # trapped robot, not a failed target
                else:
                    self.fail(reason)
                return
        else:
            n.hold_fast.update(now, False)
        if not progressing and run.used_s >= self.progress_grace_until:
            self.fail('no_spatial_progress')
            return
        if self.pending is None and now >= self.next_check:
            self.next_check = now+1.
            self.submit('target', n.target_xy, n.target_radius, robot)
