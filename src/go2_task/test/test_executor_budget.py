"""Run the real executor transitions offline; no DDS publishers or simulated motion."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from go2_msgs.msg import LocStatus, RecoverCmd
from go2_task.task_executor import TaskExecutor
from go2_task.task_logic import Waypoint, WaypointRun, WpState, Progress, Hold
from go2_task.route_execution import RouteExecution
from go2_task.route_policy import DetourGuard, SpatialProgress
from go2_task.local_recovery import LocalRecovery


def executor(state=LocStatus.DEGRADED):
    e = object.__new__(TaskExecutor)
    e.mode = 'once'
    e.mission = 'test'
    e.declare_parameter = lambda name, value: NS(value=value)
    e.geometry = NS(loc_sigma_m=.1)
    e.index = 0
    e.queue = [1]
    e.retried = False
    e.waypoints = [Waypoint('blocked', 1., 0., .4), Waypoint('next', 2., 0., .4)]
    e.runs = [WaypointRun(i, 90., 5, state=WpState.APPROACH) for i in range(2)]
    e.now_s = lambda: 10.
    e.last_tick_s = 10.
    e.loc_state = state
    e.recover = None
    e.recover_seq = 0
    e.backup_m = .3
    e.wait_s = 2.
    e.nav_id = 'old'
    e.nav_seq = 1
    e.speed_pct = 50.
    e.target_xy = (1., 0.)
    e.target_radius = .4
    e.nav_status = None
    e.nav_status_s = 10.
    e.keepalive_s = 1.
    e.last_send_s = 10.
    e.trouble_reason = 'no_progress'
    e.explicit_hold_s = .5
    e.decision_hold_s = 5.
    e.progress = SpatialProgress(20., .4)
    e.detour_cfg = (1.5, 2., 3.)
    e.detour = DetourGuard(*e.detour_cfg)
    e.pass_radius_m = .6
    e.no_progress_m = .4
    e.hold_fast = Hold(.5)
    e.hold_slow = Hold(5.)
    e.nav_pub = Mock()
    e.recover_pub = Mock()
    e.publish_markers = Mock()
    e.robot_pose = Mock(return_value=None)  # expiry must not depend on available pose
    e.event = Mock()
    e.warn_throttled = Mock()
    e.get_logger = lambda: Mock()
    e.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time()))
    e.route = RouteExecution(e)
    e.local = LocalRecovery(e)
    return e


@pytest.mark.parametrize('loc', [LocStatus.OK, LocStatus.DEGRADED])
def test_time_budget_cancels_running_recovery_and_advances(loc):
    e = executor(loc)
    e.runs[0].used_s = 90.
    e.recover = dict(step='turn', id='r4', until=None)
    e.tick()
    assert e.runs[0].state == WpState.SKIPPED and e.index == 1
    messages = [c.args[0] for c in e.nav_pub.publish.call_args_list]
    assert len(messages) == 2
    assert messages[0].id == 'old' and not messages[0].active
    assert messages[1].active and messages[1].x == 2. and messages[1].id != 'old'
    assert e.recover_pub.publish.call_args.args[0].action == RecoverCmd.STOP


def test_fifth_recovery_completion_skips_without_resending_old_target():
    e = executor()
    e.runs[0].tries = 5
    e.nav_id = ''  # recovery already withdrew the old goal
    e.back_to_approach()
    assert e.runs[0].state == WpState.SKIPPED and e.index == 1
    messages = [c.args[0] for c in e.nav_pub.publish.call_args_list]
    assert len(messages) == 1 and messages[0].x == 2.


def test_final_recovery_can_finish_before_deadline_and_lost_still_pauses():
    e = executor()
    e.runs[0].tries = 5
    e.recover = dict(step='turn', id='r5', until=None)
    e.tick()
    assert e.index == 0 and e.recover is not None
    e.loc_state = LocStatus.LOST
    e.runs[0].used_s = 90.
    e.tick()
    assert e.index == 0 and e.paused
    e.nav_pub.publish.assert_not_called()
