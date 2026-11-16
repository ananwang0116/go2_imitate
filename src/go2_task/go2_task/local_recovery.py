"""Trapped-robot recovery shared by EVERY task policy (multi-point RouteExecution and the legacy single/teach policy).

One definition of the ladders (task_logic.LOCAL_LADDERS) and one executor of the steps, inside the sole TaskExecutor:
  blocked        local rollouts all collide (evidence-map/voxel memory exists)  -> backup, turn
  start_blocked  global planner start unusable (that map has no voxel memory)   -> turn, relocate (never backup)
Chain stays single-owner: TaskExecutor decides -> /mot_recover_cmd (mot_recover) or a NavGoal to a checked temporary spot
(mot_nav) -> mot_cmd_mux -> safe_gate (the only limiter). Nothing here publishes velocity or plans.

Each step is judged by the ACTUAL localisation change (loc_pose), never by mot_recover's `ok`: backup must really move back
backup_m (default 0.3 m, tolerance local_backup_tol_m), turn must really turn >= local_turn_min_effect_rad in the asked
direction, relocate must reach the temporary spot. Ladder position is counted per waypoint visit and reason; neither a new goal
id nor a replan resets it, nor the waypoint's elapsed time (the host policies keep those). Timeouts are only watchdogs.
Hosts provide: can_relocate(), audit(kind, **f), transition(phase, reason), resume_goal(reason), exhausted(reason).
"""
import math
import time

from go2_msgs.msg import NavStatus, RecoverCmd
from go2_task.route_policy import backup_effect_m, final_candidates, relocation_candidates, turn_effect_rad
from go2_task.task_logic import SINGLE_LADDERS, Waypoint, WpState
from go2_task.task_preflight import request_from


class LegacyLocalHost:
    """Host for single-point / teach tasks (no RouteExecution)."""
    ladders = SINGLE_LADDERS        # + no_path/no_progress (backup, turn) and goal_blocked (alt_goal), 2026-11-12

    def __init__(self, owner):
        self.n = owner

    def can_relocate(self):
        n = self.n
        return not n.waypoints[n.index].path        # a taught path has no "other spot"

    def audit(self, kind, **fields):
        from go2_task.route_policy import append_route_audit
        n = self.n
        run = n.runs[n.index] if 0 <= n.index < len(n.runs) else None
        row = dict(kind=kind, sim_s=n.now_s(), mission=n.mission, mode=n.mode, policy='single_or_teach',
                   waypoint=n.index+1, goal_id=n.nav_id, local_steps=dict(n.local.steps),
                   used_s=run.used_s if run else None, **fields)
        try:
            append_route_audit(row)
        except (OSError, ValueError) as exc:
            n.warn_throttled('local_audit', f'route_decisions.jsonl: {exc}')

    def transition(self, phase, reason):
        n = self.n
        self.audit('transition', to_phase=phase, reason=reason)
        n.event('LOCAL_STATE', wp=n.index+1, phase=phase, reason=reason)

    def resume_goal(self, reason):
        n = self.n
        n.runs[n.index].state = WpState.APPROACH
        n.trouble_reason = ''
        n.progress.reset(n.now_s())         # legacy 10 s remaining-distance window restarts (as back_to_approach did)
        n.new_nav_goal()                    # the 90 s per-waypoint budget and ladder position are NOT reset
        n.hold_fast.last_decision = n.hold_slow.last_decision = n.now_s()
        self.audit('goal_sent', reason=reason)
        n.publish_markers()

    def exhausted(self, reason):
        self.n.finish_waypoint(WpState.SKIPPED, reason=f'{reason}_recovery_exhausted')


class LocalRecovery:
    def __init__(self, owner):
        self.node = owner
        p = owner.declare_parameter
        self.backup_tol_m = float(p('local_backup_tol_m', .03).value)
        self.turn_min_rad = float(p('local_turn_min_effect_rad', .5).value)
        self.step_timeout_s = float(p('local_step_timeout_s', 20.).value)
        self.relocate_timeout_s = float(p('local_relocate_timeout_s', 15.).value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.backup_tol_m, self.turn_min_rad, self.step_timeout_s,
                                                       self.relocate_timeout_s)):
            raise ValueError('local recovery thresholds must be finite and positive')
        self.check_timeout_s = 2.
        self.legacy_host = LegacyLocalHost(owner)
        self.generation = 0
        self.pending = None
        self.reset()

    @property
    def host(self):
        n = self.node
        return n.route if n.route.enabled else self.legacy_host

    @property
    def active(self):
        return self.phase is not None

    @property
    def relocating(self):
        return self.phase == 'relocate_approach'

    def reset(self):
        """New waypoint / new task: ladder position and any step in flight are forgotten."""
        self.invalidate()
        self.steps = {}
        self.turn_count = 0
        self.phase = None
        self.job = None
        self.candidates = None
        self.alt_candidates = None        # alt_goal: ring of checked landing spots around the ORIGINAL target (kept across rungs)
        self.until = None
        self.return_target = None
        self.t0 = 0.
        self.reason = ''

    def invalidate(self):
        self.generation += 1
        if self.pending is not None:
            self.pending['future'].cancel()
        self.pending = None

    def owns(self, rid):
        return self.job is not None and rid == self.job['id']

    def resume(self):
        if self.phase in ('relocate_search', 'alt_search'):
            self.until = time.monotonic()+8.
        self.host.audit('resume', reason='localisation_restored', local_phase=self.phase)

    def set_phase(self, phase, reason):
        self.phase = phase
        self.host.transition(phase, reason)

    # ---------------- ladder ----------------
    def begin(self, reason):
        """mot_nav reported a trapped robot (blocked/start_blocked): run the next rung of THIS reason's ladder."""
        n, host = self.node, self.host
        ladder = host.ladders[reason]
        k = self.steps.get(reason, 0)
        if k >= len(ladder):
            self.exhaust(reason)
            return
        robot = n.robot_pose()
        if robot is None:
            n.cancel_motion()
            return
        step = ladder[k]
        self.steps[reason] = k+1
        if step in ('relocate', 'alt_goal') and not host.can_relocate():
            host.audit('local_step_unavailable', local_reason=reason, step=step)
            self.begin(reason)                 # next rung, or exhausted
            return
        self.invalidate()                      # a pending check belongs to the abandoned approach
        if n.route.enabled:
            n.route.invalidate()               # ... including the route's own target check
        n.cancel_motion()
        run = n.runs[n.index]
        run.state = WpState.RECOVER
        host.audit('local_step', local_reason=reason, step=step, step_index=k+1, ladder=list(ladder))
        n.event('RECOVER', wp=n.index+1, step=step, trigger=reason, tries=run.tries, used_s=run.used_s)
        if step == 'alt_goal':
            self.reason = reason
            if self.alt_candidates is None:
                w = n.waypoints[n.index]
                self.alt_candidates = final_candidates(w, robot, n.route.alt_offset)
            self.until = time.monotonic()+8.
            self.set_phase('alt_search', reason)
            return
        if step == 'relocate':
            self.reason = reason
            self.return_target = (n.target_xy, n.target_radius)
            self.candidates = relocation_candidates(robot, n.target_xy)
            self.until = time.monotonic()+8.
            self.set_phase('relocate_search', reason)
            return
        n.recover_seq += 1
        rid = f'r{n.recover_seq}'
        if step == 'backup':
            value = n.backup_m
        else:
            value = math.pi/2 if self.turn_count % 2 == 0 else -math.pi/2
            self.turn_count += 1
        n.recover = dict(step=step, id=rid, until=None)
        self.job = dict(reason=reason, step=step, id=rid, start=robot, t0_used=run.used_s, value=value)
        n.recover_pub.publish(RecoverCmd(id=rid, action=RecoverCmd.BACKUP if step == 'backup' else RecoverCmd.TURN,
                                         value=float(value)))
        self.set_phase('local_recover', f'{reason}:{step}')
        n.hold_fast.last_decision = n.hold_slow.last_decision = n.now_s()
        n.publish_markers()

    def exhaust(self, reason):
        self.phase = None
        self.host.exhausted(reason)

    def resume_goal(self, reason):
        self.phase = None
        self.host.resume_goal(reason)

    def on_recover_done(self, fields):
        """mot_recover finished. Success is decided from the ACTUAL localisation change, not from its odom `ok`."""
        n = self.node
        job = self.job
        if job is None or fields.get('id') != job['id']:
            return
        if fields.get('result') in ('stopped', 'replaced'):
            # Our own STOP (LOST, cancel) or a replacement: the step proved nothing. Keep `job` so tick() sees the
            # interrupted step and resumes the unchanged goal once localisation is back; never judge/advance the ladder here.
            return
        self.job = None
        n.recover = None
        robot = n.robot_pose()
        effect, effective = None, False
        if robot is not None:
            if job['step'] == 'backup':
                effect = backup_effect_m(job['start'], robot)
                effective = effect >= n.backup_m-self.backup_tol_m
            else:
                effect = turn_effect_rad(job['start'], robot)
                effective = abs(effect) >= self.turn_min_rad and effect*job['value'] > 0
        n.event('RECOVER_RESULT', wp=n.index+1, step=job['step'], result=fields.get('result', '?'),
                effect=effect if effect is not None else 'no_pose', effective=effective)
        self.host.audit('local_result', step=job['step'], local_reason=job['reason'], result=fields.get('result', '?'),
                        effect=effect, effective=effective, requested=job['value'], start_pose=list(job['start']))
        if effective:
            self.resume_goal(f"local_recover_done:{job['step']}")
        else:
            # An ineffective step (e.g. wedged, blocked behind) is not retried: next rung of the same ladder.
            self.begin(job['reason'])

    # ---------------- relocate ----------------
    def submit_alt(self, xy, radius, robot):
        n = self.node
        if not n.check_client.service_is_ready():
            return False
        future = n.check_client.call_async(request_from([Waypoint('alt_goal', xy[0], xy[1], radius)], robot[:2]))
        self.pending = dict(future=future, generation=self.generation, index=n.index, xy=xy, radius=radius, kind='alt',
                            deadline=time.monotonic()+self.check_timeout_s)
        self.host.audit('check_request', check_kind='alt_goal', candidate=list(xy), radius_m=radius,
                        check_start=list(robot[:2]))
        return True

    def submit(self, xy, radius, robot):
        n = self.node
        if not n.check_client.service_is_ready():
            return False
        (gx, gy), gr = self.return_target
        # Connected from the dog to the temporary spot, then from that spot to the ORIGINAL goal.
        points = [Waypoint('local_spot', xy[0], xy[1], radius), Waypoint('local_goal', gx, gy, gr)]
        future = n.check_client.call_async(request_from(points, robot[:2]))
        self.pending = dict(future=future, generation=self.generation, index=n.index, xy=xy, radius=radius,
                            deadline=time.monotonic()+self.check_timeout_s)
        self.host.audit('check_request', check_kind='relocate', candidate=list(xy), radius_m=radius,
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
            self.host.audit('check_error', reason=str(exc))
        if pending.get('kind') == 'alt':
            if res is None or not res.available or len(res.ok) != 1 or len(res.reason) != 1:
                future.cancel()
                self.host.audit('check_unavailable', check_kind='alt_goal')
                return
            self.host.audit('check_result', check_kind='alt_goal', candidate=list(pending['xy']), ok=bool(res.ok[0]),
                            reason=res.reason[0], leg_m=float(res.leg_m[0]) if len(res.leg_m) == 1 else None)
            if res.ok[0]:
                n.target_xy, n.target_radius = pending['xy'], pending['radius']
                n.runs[n.index].tries += 1
                self.resume_goal('alt_goal_reachable')          # the checked landing spot becomes the target (original yaw kept)
            return
        if res is None or not res.available or len(res.ok) != 2 or len(res.reason) != 2:
            future.cancel()
            self.host.audit('check_unavailable', check_kind='relocate')
            return
        ok = all(bool(v) for v in res.ok)
        reason = next((r for r, v in zip(res.reason, res.ok) if not v), res.reason[0])
        self.host.audit('check_result', check_kind='relocate', candidate=list(pending['xy']), ok=ok, reason=reason)
        if ok:
            n.target_xy, n.target_radius = pending['xy'], pending['radius']
            self.t0 = n.runs[n.index].used_s
            n.runs[n.index].state = WpState.APPROACH
            self.set_phase('relocate_approach', 'temporary_spot_reachable')
            n.new_nav_goal()
            self.host.audit('goal_sent', reason='temporary_relocation_spot')
            n.publish_markers()

    def restore_target(self):
        if self.return_target is not None:
            self.node.target_xy, self.node.target_radius = self.return_target

    def end_relocation(self, reason):
        self.host.audit('relocation_failed', reason=reason)
        self.restore_target()
        self.exhaust(self.reason)

    # ---------------- per tick ----------------
    def tick(self, now, run, robot):
        """Called by the host's tick while active. Pose is available and the localisation is not LOST."""
        n = self.node
        self.poll()
        if not self.active or n.runs[n.index] is not run:
            return
        if self.phase == 'local_recover':
            job = self.job
            if job is None or n.recover is None:
                # LOST (or an external cancel) stopped the motion: it proved nothing, resume the same goal.
                self.job = None
                self.resume_goal('local_recover_interrupted')
            elif run.used_s-job['t0_used'] > self.step_timeout_s:
                n.cancel_motion()
                self.job = None
                self.host.audit('local_result', step=job['step'], local_reason=job['reason'], result='watchdog_timeout',
                                effect=None, effective=False)
                self.begin(job['reason'])
            return
        if self.phase == 'alt_search':
            if time.monotonic() >= self.until or (self.pending is None and not self.alt_candidates):
                self.exhaust(self.reason)
                return
            if self.pending is None:
                w = n.waypoints[n.index]
                radius = max(n.geometry.loc_sigma_m, min(.15, w.radius_m))
                if self.submit_alt(self.alt_candidates[0], radius, robot):
                    self.alt_candidates.pop(0)
            return
        if self.phase == 'relocate_search':
            if time.monotonic() >= self.until:
                self.exhaust(self.reason)
                return
            if self.pending is None:
                if not self.candidates:
                    self.exhaust(self.reason)
                    return
                xy = self.candidates[0]
                if self.submit(xy, max(n.geometry.loc_sigma_m, .2), robot):
                    self.candidates.pop(0)
            return
        # relocate_approach
        if run.used_s-self.t0 > self.relocate_timeout_s:
            self.end_relocation('relocate_timeout')      # also when mot_nav status is silent
            return
        if now-n.nav_status_s > 2.:
            if n.nav_id:
                self.host.audit('feedback_stop', reason='no_nav')
                n.cancel_motion()
            n.warn_throttled('no_nav', 'local recovery waiting for mot_nav status')
            return
        if not n.nav_id:
            n.new_nav_goal()
        elif now-n.last_send_s >= n.keepalive_s:
            n.send_goal()
        status = n.nav_for_me()
        if status is not None and status.state == NavStatus.ARRIVED:
            # Reaching the temporary spot is not reaching the waypoint: back to the ORIGINAL goal.
            self.host.audit('relocated', reason='temporary_spot_reached')
            self.restore_target()
            self.resume_goal('relocated_resume_original')
            return
        if status is not None and status.state == NavStatus.FAILED:
            if n.hold_fast.update(now, True):
                self.end_relocation('relocate_nav_failed:'+(status.reason or 'nav_failed'))
        else:
            n.hold_fast.update(now, False)
