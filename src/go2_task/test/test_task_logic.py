"""Offline tests for mission files and task_preflight. Run: colcon test --packages-select go2_task"""
from pathlib import Path

import pytest

from go2_task import task_preflight
from go2_task.task_logic import Waypoint, load_mission, save_mission

GEOMETRY = Path(__file__).resolve().parents[2] / 'go2_bringup' / 'config' / 'geometry.yaml'


def test_round_trip(tmp_path):
    wps = [Waypoint('door', 1.0, 2.0, 0.4), Waypoint('desk', -1.5, 0.5, 0.6, yaw=1.57)]
    path = tmp_path / 'm.yaml'
    save_mission(path, wps)
    assert load_mission(path, 0.4) == wps


def test_defaults_and_errors(tmp_path):
    path = tmp_path / 'm.yaml'
    path.write_text('waypoints:\n  - {x: 1, y: 2}\n')
    assert load_mission(path, 0.4) == [Waypoint('wp1', 1.0, 2.0, 0.4)]
    path.write_text('waypoints:\n  - {x: 1, y: 2, radius: 0.3}\n')
    with pytest.raises(ValueError, match='radius'):
        load_mission(path, 0.4)
    path.write_text('waypoints: []\n')
    with pytest.raises(ValueError):
        load_mission(path, 0.4)


def test_preflight_format_only_exit_codes(tmp_path):
    good = tmp_path / 'good.yaml'
    good.write_text('waypoints:\n  - {x: 1, y: 2}\n')
    small = tmp_path / 'small.yaml'
    small.write_text('waypoints:\n  - {x: 1, y: 2, radius_m: 0.01}\n')
    base = ['--format-only', '--geometry', str(GEOMETRY), '--missions-dir', str(tmp_path)]
    assert task_preflight.main(['good', *base]) == 0
    assert task_preflight.main(['small', *base]) == 1
    assert task_preflight.main(['small', '--force', *base]) == 0
    assert task_preflight.main(['missing', *base]) == 2


def test_mission_paths_round_trip_and_checks(tmp_path):
    wps = [Waypoint('P1', 2.0, 0.0, 0.4, path=[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]), Waypoint('end', 3.0, 0.0, 0.4)]
    path = tmp_path / 'm.yaml'
    save_mission(path, wps)
    assert load_mission(path, 0.4) == wps
    path.write_text('waypoints:\n  - {x: 1, y: 2, path: [[0, 0]]}\n')
    with pytest.raises(ValueError, match='path'):
        load_mission(path, 0.4)
    req = task_preflight.request_from(wps, start=(0.5, 0.5))
    assert list(req.path_off) == [0, 3, 3] and list(req.path_x) == [0.0, 1.0, 2.0] and req.use_start
    assert list(task_preflight.request_from([wps[1]]).path_off) == []


def test_rules():
    from go2_task.task_logic import Hold, Progress, WaypointRun, WpState, LADDERS, EXPLICIT_REASONS
    p = Progress(10.0)
    p.reset(0.0)
    assert p.update(0.0, 5.0) and p.update(9.0, 4.95)       # 5 cm better is not progress (eps 0.1)
    assert not p.update(10.5, 4.95)
    assert p.update(11.0, 4.0)                              # real progress resets the window
    h = Hold(5.0)
    assert not h.update(0, True) and not h.update(4.9, True) and h.update(5.0, True)
    assert not h.update(6.0, True) and not h.update(9.9, True)   # needs 5 s since the last decision too
    r = WaypointRun(0, 30.0, 2, state=WpState.APPROACH)
    r.tick(10.0, paused=True)
    assert r.used_s == 0.0                                  # LOST pauses the budget
    r.tick(10.0, paused=False)
    assert [r.next_recovery('blocked') for _ in range(2)] == list(LADDERS['blocked']) and r.out_of_budget()
    assert 'no_progress' not in EXPLICIT_REASONS and 'blocked' in EXPLICIT_REASONS


def test_ladders_follow_the_reason():
    from go2_task.task_logic import WaypointRun, LADDERS, DEFAULT_LADDER
    r = WaypointRun(0, 90.0, 99)
    assert r.next_recovery('blocked') == 'backup'           # evidence-map memory exists: back off first
    assert LADDERS['start_blocked'][0] == 'turn' and 'backup' not in LADDERS['start_blocked']    # global map: no voxel memory
    assert r.next_recovery('no_path') == 'wait'             # evidence may change: wait first
    assert r.next_recovery('goal_blocked') == 'alt_position'
    assert r.next_recovery('blocked') == 'turn'             # each reason walks its own ladder
    assert [r.next_recovery('no_nav') for _ in range(3)] == ['wait'] * 3
    assert r.next_recovery('something_new') == DEFAULT_LADDER[0]
    assert all(step in ('wait', 'backup', 'alt_position', 'turn', 'relocate') for lad in LADDERS.values() for step in lad)
    assert 'replan' not in {s for lad in LADDERS.values() for s in lad}      # mot_nav replans by itself
