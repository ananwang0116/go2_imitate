"""Offline tests for safe_gate's rules. Run: colcon test --packages-select go2_safe"""
import math
import time
from pathlib import Path

import numpy as np

from go2_common import geometry
from go2_safe.safe_logic import checks_for, limit, rotation_blocked, safe_speed, scan_points, sweep_gap

G = geometry.load(Path(__file__).resolve().parents[2] / 'go2_bringup' / 'config' / 'geometry.yaml')
MAXV = (0.5, 0.2, 0.6)


def test_safe_speed_zero_inside_stop_gap_and_grows():
    assert safe_speed(0.05, 0.10, 0.15, 0.2) == 0.0
    assert safe_speed(1.0, 0.10, 0.15, 0.2) > safe_speed(0.5, 0.10, 0.15, 0.2) > 0


def test_sweep_gap_front_side_and_inside():
    pts = np.array([[1.35, 0.0], [0.0, 1.0]])
    assert math.isclose(sweep_gap(pts, 1, 0, 0.35, 0.20), 1.0)       # front face at 0.35
    assert math.isclose(sweep_gap(pts, 0, 1, 0.35, 0.20), 0.8)       # left face at 0.20
    assert sweep_gap(pts, -1, 0, 0.35, 0.20) == math.inf             # nothing behind
    assert sweep_gap(np.array([[0.1, 0.0]]), 1, 0, 0.35, 0.2) == 0.0  # inside the body


def test_limit_stops_before_wall_and_keeps_turn():
    wall = np.array([[0.45, y] for y in np.linspace(-1, 1, 41)])     # 0.10 m in front of the body
    vx, vy, wz, gap, d = limit((0.4, 0.0, 0.3), wall, G, MAXV)
    assert abs(vx) < 1e-9 and wz == 0.3 and d == 'front' and math.isclose(gap, 0.10, abs_tol=1e-9)
    vx, *_ = limit((0.4, 0.0, 0.0), np.empty((0, 2)), G, MAXV)
    assert vx == 0.4
    vx, *_ = limit((-0.4, 0.0, 0.0), wall, G, MAXV)
    assert vx == -0.4                                                 # wall is in front, backing up is free


def test_scan_points_and_speed():
    ranges = [1.0] * 720
    ranges[0] = float('inf')
    pts = scan_points(ranges, -math.pi, 2 * math.pi / 720, 0.15, 10.0)
    assert pts.shape == (719, 2)
    t0 = time.perf_counter()
    for _ in range(100):
        limit((0.4, 0.1, 0.0), pts, G, MAXV)
    assert (time.perf_counter() - t0) / 100 < 0.002                  # < 2 ms per decision (latency budget)


def test_min_move_cut_and_back_limit():
    from go2_safe.safe_logic import clamp
    assert clamp((-0.5, 0.0, 0.0), (0.5, 0.2, 0.5, 0.3))[0] == -0.3        # back limited separately
    wall = np.array([[0.35 + 0.13, y] for y in np.linspace(-1, 1, 41)])    # 0.13 m in front: below the ~0.138 m cut point
    vx_cut, *_ = limit((0.5, 0.0, 0.0), wall, G, (0.5, 0.2, 0.5, 0.3), min_move_mps=0.12)
    vx_raw, *_ = limit((0.5, 0.0, 0.0), wall, G, (0.5, 0.2, 0.5, 0.3), min_move_mps=0.0)
    assert vx_cut == 0.0 and 0.0 < vx_raw < 0.12                          # creeping speed is cut to a stop
    far = np.array([[2.0, 0.0]])
    assert limit((0.5, 0.0, 0.0), far, G, (0.5, 0.2, 0.5, 0.3), 0.12)[0] == 0.5


def test_speeds_file():
    from go2_common import speeds
    s = speeds.load(Path(__file__).resolve().parents[2] / 'go2_bringup' / 'config' / 'speeds.yaml')
    assert (s.forward_mps, s.back_mps, s.turn_radps) == (0.5, 0.3, 0.5)


def test_checks_by_mode():
    # hand driving is never obstacle- or localisation-limited, in mapping AND when taking over during navigation (2026-10-26)
    assert checks_for('map', False, True) == ('hand', False, False)
    assert checks_for('nav', False, True) == ('hand', False, False)
    assert checks_for('nav', True, False) == ('hand', False, False)        # manual:=true (map only) behaves the same
    # autonomous commands (navigation): obstacle limiting, scan and localisation checks
    assert checks_for('nav', False, False) == ('auto', True, True)


def test_pure_turn_is_zeroed_only_toward_a_point_entering_the_body():
    corner = np.array([[0.32, 0.22]])          # just outside the front-left corner
    assert rotation_blocked(corner, 0.4, G.body_half_length_m, G.body_half_width_m, 0.3)
    assert not rotation_blocked(corner, -0.4, G.body_half_length_m, G.body_half_width_m, 0.3)
    assert limit((0.0, 0.0, 0.4), corner, G, MAXV)[:3] == (0.0, 0.0, 0.0)
    assert limit((0.0, 0.0, -0.4), corner, G, MAXV)[2] == -0.4


def test_turn_check_ignores_free_space_and_points_already_inside():
    assert not rotation_blocked(np.array([[3.0, 0.0]]), 0.4, G.body_half_length_m, G.body_half_width_m, 0.3)
    assert not rotation_blocked(np.array([[0.1, 0.0]]), 0.4, G.body_half_length_m, G.body_half_width_m, 0.3)
    assert not rotation_blocked(None, 0.4, G.body_half_length_m, G.body_half_width_m, 0.3)
