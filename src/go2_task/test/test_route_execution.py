"""Route sequencing and asynchronous planner replies through real executor methods."""
from concurrent.futures import Future
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

from go2_msgs.msg import LocStatus, NavStatus, RecoverCmd
from go2_task.route_policy import DetourGuard, SpatialProgress, final_candidates
from go2_task.task_logic import Waypoint, WpState
from test_executor_budget import executor


def route_executor():
    n = executor(LocStatus.OK)
    n.check_client = Mock()
    n.check_client.service_is_ready.return_value = True
    n.check_client.call_async.side_effect = lambda req: Future()
    n.robot_pose.return_value = (0., 0., 0.)
    n.waypoint_budget_s, n.max_tries = 90., 5
    n.bad = {}
    n.route.begin_round()
    n.route.begin_waypoint()
    return n


def answer(n, ok, reason=''):
    n.route.pending['future'].set_result(NS(available=True, ok=[ok], reason=[reason], leg_m=[1.]))
    n.route.poll()


def test_progress_ignores_creep_jitter_and_turning_but_counts_real_motion():
    p = SpatialProgress(20., .4)
    for t in range(20):
        assert p.update(t, (.05*(t % 2), 0.))   # jitter / turning in place earns nothing
    assert not p.update(20., (0., 0.))
    p.reset()
    for t in range(41):
        ok = p.update(t, (.006*t, 0.))          # creeping 0.006 m/s (the 20261112_144912_nav case)
    assert not ok
    p.reset()
    for t in range(60):
        assert p.update(t, (.5*t if t < 30 else .5*(60-t), 0.))   # out and back along one corridor is progress


def test_progress_has_full_window_from_start_and_slow_start_is_not_failed():
    p = SpatialProgress(20., .4)
    for t in range(19):
        assert p.update(t, (0., 0.))           # standing before the gait starts
    assert p.update(19., (.0, 0.))
    assert p.update(19.5, (.5, 0.))            # moved: the last sample at the old spot (t=19) is the last 'far' one
    assert p.update(38.9, (.5, 0.))
    assert not p.update(39., (.5, 0.))         # then 20 s without leaving the 0.4 m disc


def test_executor_skips_after_window_even_while_remaining_path_keeps_shrinking():
    n = route_executor()
    n.check_client.service_is_ready.return_value = False
    for tick in range(21):
        n.now_s = lambda t=tick: float(t)
        n.nav_status_s = float(tick)
        n.runs[0].used_s = float(tick)
        n.robot_pose.return_value = (.02*(tick % 2), 0., 0.)
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=10.-tick*.2)
        n.route.tick(float(tick), n.runs[0])
        if tick < 20:
            assert n.index == 0
    assert n.index == 1 and n.runs[0].reason == 'no_spatial_progress'


def test_intermediate_skip_cancels_old_goal_without_backup_or_alternative():
    n = route_executor()
    n.route.fail('target_area_occupied')
    assert n.runs[0].state == WpState.SKIPPED and n.index == 1
    messages = [c.args[0] for c in n.nav_pub.publish.call_args_list]
    assert not messages[0].active and messages[0].id == 'old'
    assert messages[1].active and messages[1].x == 2.
    n.recover_pub.publish.assert_not_called()


def test_once_finishes_without_revisiting_skipped_points():
    n = route_executor()
    n.route.fail('target_area_occupied')
    n.finish_waypoint(WpState.ARRIVED)
    assert n.mode == 'idle' and n.index == -1
    assert not any(c.args[0] == 'ROUND_RETRY' for c in n.event.call_args_list)


def test_loop_next_round_retries_original_points_and_original_final_target():
    n = route_executor()
    n.mode = 'loop'
    original = [(w.x, w.y) for w in n.waypoints]
    n.route.fail('target_area_occupied')
    n.target_xy = (2.7, .3)  # accepted final alternative, without editing the waypoint
    n.route.alt_active = True
    n.finish_waypoint(WpState.ARRIVED, 'final_alternative')
    assert n.route.round == 2 and n.index == 0 and n.queue == [1]
    assert n.target_xy == original[0]
    assert [(w.x, w.y) for w in n.waypoints] == original
    n.finish_waypoint(WpState.ARRIVED)
    assert n.index == 1 and n.target_xy == original[1]
    assert n.target_radius == .4


def test_route_block_on_the_way_does_not_mean_target_area_occupied():
    n = route_executor()
    n.route.submit('target', n.target_xy, .4, (0., 0., 0.))
    answer(n, False, 'not connected to the dog (no_path)')
    assert n.index == 0 and n.runs[0].state == WpState.APPROACH
    n.route.submit('target', n.target_xy, .4, (0., 0., 0.))
    answer(n, False, 'no passable position in the area (hard clearance 0.3 m)')
    assert n.index == 0  # require sustained target evidence
    n.runs[0].used_s = 1.2
    n.route.submit('target', n.target_xy, .4, (0., 0., 0.))
    answer(n, False, 'no passable position in the area (hard clearance 0.3 m)')
    assert n.index == 1


def test_final_candidate_checked_from_robot_before_dispatch_and_keeps_yaw():
    n = route_executor()
    n.route.fail('target_area_occupied')  # now original last point
    n.waypoints[1].yaw = 1.2
    n.route.fail('goal_blocked')
    assert n.route.phase == 'alternative_search'
    n.route.tick(10., n.runs[1])
    request = n.check_client.call_async.call_args.args[0]
    assert request.use_start and request.start_x == 0. and len(request.x) == 1
    xy = n.route.pending['xy']
    n.nav_pub.publish.reset_mock()
    answer(n, True)
    goal = n.nav_pub.publish.call_args.args[0]
    assert (goal.x, goal.y) == xy and goal.use_yaw and goal.yaw == 1.2
    assert goal.radius_m == .15
    assert n.waypoints[1].x == 2. and n.waypoints[1].radius_m == .4
    assert n.route.phase == 'alternative_approach'


def test_cancel_invalidates_pending_candidate_and_never_dispatches_late_reply():
    n = route_executor()
    n.route.fail('blocked')
    n.route.fail('blocked')
    n.route.tick(10., n.runs[1])
    future = n.route.pending['future']
    n.cancel_task('user_cancel')
    assert future.cancelled() and n.route.pending is None
    n.nav_pub.publish.reset_mock()
    n.route.poll()
    n.nav_pub.publish.assert_not_called()
    assert not n.active()


def test_final_no_candidates_finishes_without_retrying_earlier_points():
    n = route_executor()
    n.route.fail('blocked')
    n.route.fail('blocked')
    n.route.candidates = []
    n.route.tick(10., n.runs[1])
    assert n.mode == 'idle' and all(r.state == WpState.SKIPPED for r in n.runs)


def test_stale_nav_feedback_withdraws_goal_instead_of_skipping_point():
    n = route_executor()
    n.nav_status_s = 0.
    n.route.tick(10., n.runs[0])
    assert not n.nav_id and n.index == 0
    assert not n.nav_pub.publish.call_args.args[0].active


def test_single_and_teach_tasks_keep_their_separate_policy():
    n = route_executor()
    n.waypoints = [n.waypoints[0]]
    n.route.begin_round()
    assert not n.route.enabled
    n.waypoints.append(Waypoint('teach', 2., 0., .4, path=[[0., 0.], [2., 0.]]))
    n.route.begin_round()
    assert not n.route.enabled


def test_audit_has_round_transition_cause_original_and_candidate(tmp_path, monkeypatch):
    monkeypatch.setenv('GO2_RUN_DIR', str(tmp_path))
    n = route_executor()
    n.route.fail('target_area_occupied')
    rows = [json.loads(line) for line in (tmp_path/'route_decisions.jsonl').read_text().splitlines()]
    skip = next(r for r in rows if r.get('to_phase') == 'skipped')
    assert skip['original'] == [1., 0.] and skip['round'] == 1
    assert skip['reason'] == 'target_area_occupied' and skip['from_phase'] == 'approach'
    assert any(r['kind'] == 'goal_sent' and r['waypoint'] == 2 for r in rows)


# ---- trapped-robot ladders: blocked = backup, turn ; start_blocked = turn, relocate ----
def nav_fail(n, reason, t):
    """mot_nav reports FAILED(reason) on the current goal for longer than the explicit hold."""
    for k, now in enumerate((t, t+.6)):
        n.now_s = lambda v=now: v
        n.nav_status_s = now
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FAILED, reason=reason, remaining_m=3.)
        n.runs[0].used_s = now
        n.route.tick(now, n.runs[0])


def last_recover(n):
    return n.recover_pub.publish.call_args.args[0]


def done(n, pose, result='ok'):
    n.robot_pose.return_value = pose
    n.local.on_recover_done({'id': n.recover['id'], 'result': result})


def test_blocked_runs_backup_then_turn_and_judges_by_actual_displacement():
    n = route_executor()
    nav_fail(n, 'blocked', 1.)
    assert last_recover(n).action == RecoverCmd.BACKUP and last_recover(n).value == n.backup_m
    assert n.local.phase == 'local_recover' and n.index == 0 and not n.nav_id
    # mot_recover says ok but the localisation did not move: not effective -> next rung (turn), no goal resend
    done(n, (0., 0., 0.))
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (0., 0., 0.1), 'ok')            # barely turned: exhausted -> intermediate point skipped, not a target failure
    assert n.runs[0].state == WpState.SKIPPED and n.runs[0].reason == 'blocked_recovery_exhausted' and n.index == 1


def test_effective_backup_resumes_same_goal_without_resetting_clock_progress_or_ladder():
    n = route_executor()
    nav_fail(n, 'blocked', 5.)
    used_before, last_progress = n.runs[0].used_s, n.route.progress.last_progress_s
    first_goal = n.nav_id
    done(n, (-.3, 0., 0.))                     # really moved the requested .3 m backwards
    assert n.route.phase == 'approach' and n.nav_id and n.nav_id != first_goal
    assert (n.target_xy, n.runs[0].used_s) == ((1., 0.), used_before)
    assert n.route.progress.last_progress_s == last_progress
    assert n.local.steps == {'blocked': 1}
    # the same reason again continues with the NEXT rung: a new goal id/replan did not restart the ladder
    nav_fail(n, 'blocked', 9.)
    assert last_recover(n).action == RecoverCmd.TURN
    assert n.local.steps == {'blocked': 2}


def test_start_blocked_never_backs_up_and_starts_with_turn():
    n = route_executor()
    nav_fail(n, 'start_blocked', 1.)
    assert last_recover(n).action == RecoverCmd.TURN
    assert not any(c.args[0].action == RecoverCmd.BACKUP for c in n.recover_pub.publish.call_args_list)
    done(n, (0., 0., 1.4))                      # turned about 80 degrees to the left
    assert n.route.phase == 'approach'
    nav_fail(n, 'start_blocked', 5.)
    assert n.local.phase == 'relocate_search'
    assert not any(c.args[0].action == RecoverCmd.BACKUP for c in n.recover_pub.publish.call_args_list)


def test_turn_direction_alternates_and_wrong_direction_is_not_effective():
    n = route_executor()
    nav_fail(n, 'start_blocked', 1.)
    assert last_recover(n).value > 0
    done(n, (0., 0., -1.4))                     # turned the other way: ineffective even though large
    assert n.local.phase == 'relocate_search'


def test_relocation_checks_spot_with_original_goal_from_dog_then_returns_to_original_goal():
    n = route_executor()
    n.waypoints[0].yaw = 1.2
    nav_fail(n, 'start_blocked', 1.)
    done(n, (0., 0., 0.1))                      # ineffective turn -> relocate immediately
    assert n.local.phase == 'relocate_search'
    n.route.tick(10., n.runs[0])
    request = n.check_client.call_async.call_args.args[0]
    assert request.use_start and len(request.x) == 2 and (request.x[1], request.y[1]) == (1., 0.)
    spot = n.local.pending['xy']
    n.nav_pub.publish.reset_mock()
    n.local.pending['future'].set_result(NS(available=True, ok=[True, True], reason=['', ''], leg_m=[.5, 1.]))
    n.local.poll()
    goal = n.nav_pub.publish.call_args.args[0]
    assert (goal.x, goal.y) == spot and not goal.use_yaw and n.local.phase == 'relocate_approach'
    assert n.waypoints[0].x == 1. and n.waypoints[0].yaw == 1.2
    # the target-area check must not treat the temporary spot as the waypoint
    n.check_client.call_async.reset_mock()
    n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=1.)
    n.nav_status_s = 10.
    n.robot_pose.return_value = (.4, .1, 0.)
    n.route.next_check = 0.
    n.route.tick(10., n.runs[0])
    n.check_client.call_async.assert_not_called()
    n.nav_pub.publish.reset_mock()
    n.nav_status = NS(id=n.nav_id, state=NavStatus.ARRIVED, remaining_m=0.)
    n.route.tick(10.2, n.runs[0])
    resumed = n.nav_pub.publish.call_args.args[0]
    assert n.index == 0 and (resumed.x, resumed.y) == (1., 0.) and not resumed.use_yaw   # intermediate: passed, no turn
    assert resumed.radius_m == n.target_radius      # the (effective) radius is restored with the target
    assert n.target_xy == (1., 0.) and not n.local.relocating


def test_relocation_without_any_accepted_spot_falls_to_normal_failure_path():
    n = route_executor()
    nav_fail(n, 'start_blocked', 1.)
    done(n, (0., 0., 0.))
    n.local.candidates = []
    n.route.tick(10., n.runs[0])
    assert n.runs[0].state == WpState.SKIPPED and n.runs[0].reason == 'start_blocked_recovery_exhausted'


def test_lost_during_local_step_resumes_original_goal_and_keeps_ladder():
    n = route_executor()
    nav_fail(n, 'blocked', 1.)
    n.cancel_motion()                           # LOST withdraws motion
    assert n.recover is None
    n.route.resume()
    n.route.tick(10., n.runs[0])
    assert n.route.phase == 'approach' and n.nav_id and n.local.steps == {'blocked': 1}


def test_goal_level_failures_keep_the_target_failure_path():
    n = route_executor()
    nav_fail(n, 'goal_blocked', 1.)
    assert n.runs[0].state == WpState.SKIPPED and n.runs[0].reason == 'goal_blocked'
    n.recover_pub.publish.assert_not_called()


def test_backup_must_really_move_the_requested_distance():
    n = route_executor()
    nav_fail(n, 'blocked', 1.)
    done(n, (-.2, 0., 0.))                      # only .2 m of the requested .3 m: ineffective, next rung
    assert last_recover(n).action == RecoverCmd.TURN
    n2 = route_executor()
    nav_fail(n2, 'blocked', 1.)
    done(n2, (-.28, 0., 0.))                    # within the .03 m tolerance of .3 m: effective
    assert n2.local.phase is None and n2.nav_id


# ---- single-point / teach policy (RouteExecution disabled) shares the SAME ladders and executor ----
def legacy_executor(path=None):
    n = route_executor()
    n.waypoints = [n.waypoints[0]]
    n.waypoints[0].path = path
    n.runs = n.runs[:1]
    n.queue = []
    n.route.begin_round()
    assert not n.route.enabled
    n.local.reset()
    n.paused = False
    n.loc_state = LocStatus.OK
    return n


def skip_reasons(n):
    return [c.kwargs['reason'] for c in n.event.call_args_list if c.args[0] == 'SKIP']


def legacy_trouble(n, reason, t):
    for now in (t, t+.6):
        n.now_s = lambda v=now: v
        n.last_tick_s = now-.2
        n.nav_status_s, n.last_send_s = now, now
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FAILED, reason=reason, remaining_m=3.)
        n.tick()


def test_single_point_blocked_uses_shared_backup_turn_ladder_not_old_one():
    n = legacy_executor()
    legacy_trouble(n, 'blocked', 1.)
    assert last_recover(n).action == RecoverCmd.BACKUP and n.local.phase == 'local_recover'
    done(n, (-.3, 0., 0.))
    assert n.local.phase is None and n.runs[0].state == WpState.APPROACH and n.nav_id
    legacy_trouble(n, 'blocked', 5.)
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (-.3, 0., 0.))                      # turn without heading change: ineffective -> exhausted -> skipped
    assert skip_reasons(n) == ['blocked_recovery_exhausted']


def test_single_point_start_blocked_turns_then_relocates_never_backs_up():
    n = legacy_executor()
    legacy_trouble(n, 'start_blocked', 1.)
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (0., 0., 0.))
    assert n.local.phase == 'relocate_search'
    assert not any(c.args[0].action == RecoverCmd.BACKUP for c in n.recover_pub.publish.call_args_list)


def test_teach_path_cannot_relocate_and_falls_to_exhausted_after_turn():
    n = legacy_executor(path=[[0., 0.], [1., 0.]])
    legacy_trouble(n, 'start_blocked', 1.)
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (0., 0., 0.))
    assert skip_reasons(n) == ['start_blocked_recovery_exhausted']
    n.check_client.call_async.assert_not_called()


def test_single_point_no_nav_still_waits_but_no_path_backs_up_then_turns():
    n = legacy_executor()
    legacy_trouble(n, 'no_nav', 1.)
    assert n.recover is not None and n.recover['step'] == 'wait' and not n.local.active
    n = legacy_executor()
    legacy_trouble(n, 'no_path', 1.)
    assert last_recover(n).action == RecoverCmd.BACKUP and n.local.phase == 'local_recover'
    done(n, (-.3, 0., 0.))
    legacy_trouble(n, 'no_path', 5.)
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (-.3, 0., 0.))
    assert skip_reasons(n) == ['no_path_recovery_exhausted']


def test_single_point_no_progress_backs_up_then_turns_too():
    n = legacy_executor()
    n.local.begin('no_progress')
    assert last_recover(n).action == RecoverCmd.BACKUP
    done(n, (0., 0., 0.))                       # no real backward motion: ineffective -> next rung
    assert last_recover(n).action == RecoverCmd.TURN
    done(n, (0., 0., 0.))
    assert skip_reasons(n) == ['no_progress_recovery_exhausted']


def test_goal_blocked_has_no_start_special_case_no_popup_and_goes_to_checked_alternatives():
    n = legacy_executor()
    for now in (1., 1.6):                        # no path ever existed (remaining_m nan): the same ladder, nothing is rejected
        n.now_s = lambda v=now: v
        n.last_tick_s = now-.2
        n.nav_status_s, n.last_send_s = now, now
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FAILED, reason='goal_blocked', remaining_m=float('nan'))
        n.tick()
    assert n.local.phase == 'alt_search' and skip_reasons(n) == []
    assert not any(c.args[0] == 'GOAL_REJECT' for c in n.event.call_args_list)


def test_goal_blocked_tries_checked_alternative_landing_spots_like_a_final_waypoint():
    n = legacy_executor()
    n.waypoints[0].yaw = .7
    legacy_trouble(n, 'goal_blocked', 1.)         # remaining_m is finite: a plan existed, the area closed on the way
    assert n.local.phase == 'alt_search' and n.local.pending is None
    n.robot_pose.return_value = (0., 0., 0.)
    n.nav_status_s = 2.
    n.tick()
    assert n.local.pending is not None and n.local.pending['kind'] == 'alt'
    req = n.check_client.call_async.call_args.args[0]
    assert len(req.x) == 1 and req.use_start
    xy = (req.x[0], req.y[0])
    assert max(abs(xy[0]-n.waypoints[0].x), abs(xy[1]-n.waypoints[0].y)) <= 1.0 + 1e-9
    n.nav_pub.publish.reset_mock()
    n.local.pending['future'].set_result(NS(available=True, ok=[True], reason=[''], leg_m=[3.]))
    n.local.poll()
    goal = n.nav_pub.publish.call_args.args[0]
    assert (goal.x, goal.y) == xy and goal.use_yaw and goal.yaw == .7 and goal.radius_m <= .15
    assert n.runs[0].tries == 1 and n.runs[0].state == WpState.APPROACH and not n.local.active
    assert (n.waypoints[0].x, n.waypoints[0].y) == (1., 0.)          # the task's own waypoint is never rewritten


def test_goal_blocked_on_a_taught_path_cannot_move_the_target():
    n = legacy_executor(path=[[0., 0.], [1., 0.]])
    legacy_trouble(n, 'goal_blocked', 1.)
    assert skip_reasons(n) == ['goal_blocked_recovery_exhausted']
    n.check_client.call_async.assert_not_called()


def test_single_point_has_no_detour_limit_a_big_detour_is_allowed():
    n = executor(LocStatus.OK)
    n.waypoints, n.runs, n.queue = n.waypoints[:1], n.runs[:1], []
    n.route.enabled = False
    n.now_s = lambda: 5.
    n.robot_pose = Mock(return_value=(0., 0., 0.))
    n.nav_status_s = 5.
    for k, remaining in enumerate((3.4, 3.4, 12.0, 12.0, 12.0, 12.0, 12.0)):
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=remaining, reason='')
        n.runs[0].used_s = float(k)
        n.robot_pose.return_value = (float(k), 0., 0.)     # walking
        n.tick()
    assert n.runs[0].state == WpState.APPROACH and n.runs[0].reason != 'detour_too_long'


def test_stopped_by_lost_is_not_judged_and_never_consumes_the_ladder():
    n = route_executor()
    nav_fail(n, 'blocked', 1.)
    rid = n.recover['id']
    n.cancel_motion()                              # LOST: STOP is published, mot_recover answers `stopped`
    n.robot_pose.return_value = (-.1, 0., 0.)
    n.local.on_recover_done({'id': rid, 'result': 'stopped'})
    assert n.local.steps == {'blocked': 1} and n.local.phase == 'local_recover'
    assert sum(1 for c in n.recover_pub.publish.call_args_list if c.args[0].action != RecoverCmd.STOP) == 1
    n.route.tick(10., n.runs[0])                   # localisation is back: resume the unchanged goal
    assert n.local.phase is None and n.nav_id and n.local.steps == {'blocked': 1}


def test_relocation_timeout_applies_even_when_mot_nav_status_is_silent():
    n = route_executor()
    nav_fail(n, 'start_blocked', 1.)
    done(n, (0., 0., 0.))
    n.route.tick(10., n.runs[0])
    n.local.pending['future'].set_result(NS(available=True, ok=[True, True], reason=['', ''], leg_m=[.5, 1.]))
    n.local.poll()
    assert n.local.phase == 'relocate_approach'
    n.nav_status_s = -100.                         # mot_nav silent
    n.runs[0].used_s = n.local.t0 + n.local.relocate_timeout_s + 1
    n.route.tick(10., n.runs[0])
    assert n.local.phase is None and n.index == 1 and n.runs[0].reason == 'start_blocked_recovery_exhausted'
    assert n.waypoints[0].x == 1. and n.target_xy == (2., 0.)       # the original waypoint was never rewritten


def test_local_step_invalidates_the_routes_pending_target_check():
    n = route_executor()
    n.route.submit('target', n.target_xy, .4, (0., 0., 0.))
    future = n.route.pending['future']
    nav_fail(n, 'blocked', 1.)
    assert future.cancelled() and n.route.pending is None


def test_legacy_local_resume_restarts_no_progress_window_but_not_budget():
    n = legacy_executor()
    legacy_trouble(n, 'blocked', 1.)
    used = n.runs[0].used_s
    n.progress.reset = Mock()
    done(n, (-.3, 0., 0.))
    n.progress.reset.assert_called_once()
    assert n.runs[0].used_s == used and n.local.steps == {'blocked': 1}


def test_detour_guard_allows_long_baseline_and_refuses_growth_after_hold():
    g = DetourGuard(1.5, 2., 3.)
    for t in range(5):
        assert not g.update(t, 10.)             # a long route that exists from the start is the baseline
    assert not g.update(5., 11.)                # small growth
    assert g.limit() == 15.                     # max(10*1.5, 10+2)
    g = DetourGuard(1.5, 2., 3.)
    assert not g.update(0., 3.4)
    assert g.limit() == 5.4                     # 3.4*1.5=5.1 vs 3.4+2
    assert not g.update(1., 7.0)                # over the limit but not for 3 s yet
    assert not g.update(3.9, 7.0)
    assert g.update(4., 7.0)                    # hold elapsed
    g = DetourGuard(1.5, 2., 3.)
    g.update(0., 3.4)
    g.update(1., 7.0)
    assert not g.update(2., 4.0)                # back under the limit clears the hold
    assert not g.update(2.5, 7.0) and not g.update(5.4, 7.0)
    for bad in (None, float('nan'), float('inf'), 0., -1.):
        assert not g.update(9., bad)            # invalid costs are ignored
    assert g.allows(5.0) and not g.allows(9.0)


def test_route_skips_waypoint_when_planner_leg_grows_to_a_big_detour():
    n = route_executor()
    n.index = 0
    legs = [3.4, 3.4, 7.0, 7.0, 7.0, 7.0]
    for k, leg in enumerate(legs):
        n.runs[0].used_s = float(k)
        n.route.next_check = 0.
        n.route.tick(float(k), n.runs[0])
        n.route.pending['future'].set_result(NS(available=True, ok=[True], reason=[''], leg_m=[leg]))
        n.route.poll()
        if k < 5:
            assert n.index == 0
    assert n.index == 1 and n.runs[0].reason == 'detour_too_long'


def test_alternative_candidate_that_needs_the_same_detour_is_rejected():
    n = route_executor()
    n.index = 1                                  # the final waypoint
    n.route.detour.update(0., 3.4)
    n.route.fail('target_area_occupied')
    xy = n.route.candidates[0]
    n.route.submit('alternative', xy, .15, (0., 0., 0.))
    n.route.pending['future'].set_result(NS(available=True, ok=[True], reason=[''], leg_m=[9.0]))
    n.route.poll()
    assert n.runs[1].tries == 0 and n.route.phase == 'alternative_search'     # not dispatched
    n.route.submit('alternative', xy, .15, (0., 0., 0.))
    n.route.pending['future'].set_result(NS(available=True, ok=[True], reason=[''], leg_m=[4.0]))
    n.route.poll()
    assert n.runs[1].tries == 1


def test_only_the_final_waypoint_turns_to_its_heading_and_intermediates_get_pass_radius():
    n = route_executor()
    n.waypoints = [Waypoint('a', 1., 0., .4, yaw=1.2), Waypoint('b', 2., 0., .4, yaw=.5)]
    assert n.effective_radius(0) == .6 and n.effective_radius(1) == .4
    assert n.heading_enforced(1) and not n.heading_enforced(0)
    n.index, n.nav_id, n.target_xy, n.target_radius = 0, 'g', (1., 0.), .6
    n.send_goal()
    assert not n.nav_pub.publish.call_args.args[0].use_yaw
    n.index, n.target_xy, n.target_radius = 1, (2., 0.), .4
    n.send_goal()
    g = n.nav_pub.publish.call_args.args[0]
    assert g.use_yaw and g.yaw == .5 and g.radius_m == .4


def test_single_point_uses_the_sliding_window_for_no_progress():
    n = executor(LocStatus.OK)
    n.waypoints, n.runs, n.queue = n.waypoints[:1], n.runs[:1], []
    n.route.enabled = False
    n.now_s = lambda: 5.
    n.robot_pose = Mock(return_value=(0., 0., 0.))
    n.nav_status_s = 5.
    n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=3.4, reason='')
    for t in range(0, 22):
        n.runs[0].used_s = float(t)
        n.tick()
    assert n.trouble_reason == 'no_progress'                # twitching in place for 20 s, remaining_m is irrelevant


def test_markers_show_effective_radius_and_heading_arrow():
    n = route_executor()
    n.waypoints = [Waypoint('a', 1., 0., .4, yaw=1.2), Waypoint('b', 2., 0., .4)]
    n.bad = {}
    n.marker_pub = Mock()
    from go2_task.task_executor import TaskExecutor
    TaskExecutor.publish_markers(n)
    arr = n.marker_pub.publish.call_args.args[0].markers
    areas = {m.id: m for m in arr if m.ns == 'wp' and m.id % 2 == 0}
    assert abs(areas[0].scale.x - 1.2) < 1e-9 and abs(areas[2].scale.x - .8) < 1e-9     # passed 0.6 m vs final 0.4 m
    arrows = [m for m in arr if m.ns == 'wp_yaw']
    assert len(arrows) == 1 and arrows[0].id == 0
    import math
    q = arrows[0].pose.orientation
    assert abs(2*math.atan2(q.z, q.w) - 1.2) < 1e-9


def test_final_waypoint_settling_near_its_radius_is_not_a_stall_but_far_hovering_is():
    n = route_executor()
    n.check_client.service_is_ready.return_value = False
    n.index = 1                                          # the final waypoint
    n.target_xy, n.target_radius = (2., 0.), .1
    n.runs[1].used_s = 0.
    for tick in range(40):
        n.now_s = lambda t=tick: float(t)
        n.nav_status_s = float(tick)
        n.runs[1].used_s = float(tick)
        n.robot_pose.return_value = (2.-.2, 0., 0.)      # 0.2 m from the target, radius 0.1 + 0.15 settle band
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=.2)
        n.route.tick(float(tick), n.runs[1])
    assert n.index == 1 and n.runs[1].state != WpState.SKIPPED
    n = route_executor()
    n.check_client.service_is_ready.return_value = False
    n.index = 1
    n.target_xy, n.target_radius = (2., 0.), .1
    for tick in range(25):
        n.now_s = lambda t=tick: float(t)
        n.nav_status_s = float(tick)
        n.runs[1].used_s = float(tick)
        n.robot_pose.return_value = (2.-.6, 0., 0.)      # 0.6 m away: a real stall
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, remaining_m=.6)
        n.route.tick(float(tick), n.runs[1])
    assert n.runs[1].state != WpState.APPROACH or n.route.phase != 'approach'      # failed into the alternative search


def trouble_executor():
    n = legacy_executor()
    n.runs[0].trouble_budget = True
    n.runs[0].state = WpState.APPROACH
    n.robot_pose.return_value = (0., 0., 0.)
    return n


def tick_for(n, seconds, walk=False, **status):
    """Advance 0.2 s ticks; nav feedback follows `status` (default: following, plan exists). walk: the dog really moves."""
    t = n.now_s()
    for _ in range(int(round(seconds/.2))):
        t += .2
        n.now_s = lambda v=t: v
        if walk:
            n.walked = getattr(n, 'walked', 0.) + .1
            n.robot_pose.return_value = (n.walked, 0., 0.)
        n.nav_status_s, n.last_send_s = t, t
        n.nav_status = NS(id=n.nav_id, state=status.get('state', NavStatus.FOLLOWING), reason=status.get('reason', ''),
                          remaining_m=status.get('remaining_m', 5.))
        if n.mode == 'idle' or n.index < 0:
            break
        n.tick()


def test_single_point_long_healthy_walk_has_no_time_limit():
    n = trouble_executor()
    n.now_s = lambda: 10.
    n.last_tick_s = 10.
    t = 10.
    for k in range(1500):                        # 300 s of walking 0.5 m/s: far longer than the old 90 s total budget
        t += .2
        n.now_s = lambda v=t: v
        n.robot_pose.return_value = (.1*k, 0., 0.)
        n.nav_status_s, n.last_send_s = t, t
        n.nav_status = NS(id=n.nav_id, state=NavStatus.FOLLOWING, reason='', remaining_m=500.-.1*k)
        n.tick()
    assert n.runs[0].state == WpState.APPROACH and n.runs[0].used_s > 250 and n.runs[0].trouble_s < 1.


def test_trouble_time_is_a_real_clock_stop_and_continue_at_90_abandon_at_180_never_reset():
    n = trouble_executor()
    n.now_s = lambda: 10.
    n.last_tick_s = 10.
    n.recover = dict(step='wait', id='', until=None)          # a problem is being handled (never ends on its own here)
    tick_for(n, 80.)
    assert 79. < n.runs[0].trouble_s < 81. and not n.runs[0].stage_one_done
    n.recover = None
    n.trouble_reason = ''
    tick_for(n, 30., walk=True)                               # healthy again: the clock stops, it is NOT reset
    n.runs[0].tries = 3
    assert 79. < n.runs[0].trouble_s < 81.
    n.local.steps = {'blocked': 1}
    n.recover = dict(step='wait', id='', until=None)
    tick_for(n, 12.)                                          # 90 s of real problem time reached
    stage = [c.kwargs for c in n.event.call_args_list if c.args[0] == 'TROUBLE_BUDGET']
    assert [e['stage'] for e in stage] == ['retry'] and n.runs[0].stage_one_done
    assert n.runs[0].tries == 0 and n.local.steps == {} and n.recover is None       # stopped, ladders start over
    assert 89. < n.runs[0].trouble_s < 93. and n.runs[0].state == WpState.APPROACH
    n.runs[0].state = WpState.RECOVER                         # recovery again: other resets must not touch the clock
    before = n.runs[0].trouble_s
    n.runs[0].state, n.runs[0].used_s = WpState.APPROACH, 0.0
    n.local.reset()
    n.progress.reset(n.now_s())
    n.new_nav_goal()
    assert n.runs[0].trouble_s == before
    n.recover = dict(step='wait', id='', until=None)
    tick_for(n, 100.)
    stage = [c.kwargs['stage'] for c in n.event.call_args_list if c.args[0] == 'TROUBLE_BUDGET']
    assert stage == ['retry', 'abandon'] and skip_reasons(n) == ['trouble_budget_exhausted']
    assert 179. < n.runs[0].trouble_s < 184.
    assert not any(c.args[0] == 'ROUND_RETRY' for c in n.event.call_args_list)
