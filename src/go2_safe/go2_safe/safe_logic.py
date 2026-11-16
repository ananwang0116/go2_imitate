"""safe_gate's rules without ROS (tested offline). All distances come from the shared geometry (spec 3). numpy, vectorised.

The body is the rectangle body_half_length_m x body_half_width_m around base_footprint. For a command, the body is swept
along the commanded linear direction; the nearest scan point inside the swept strip gives the free distance `gap`.
The speed is limited so that braking (reaction + v^2/2a) still leaves safe_stop_gap_m:
    v_max = sqrt((a*t)^2 + 2*a*max(0, gap - safe_stop_gap_m)) - a*t
A pure in-place turn (autonomous only; hand driving is never limited) is limited by rotation_blocked(): the rectangle is
turned ahead by a short horizon in the commanded direction and the command becomes 0 if a scan point would ENTER it. Points
already inside the rectangle are ignored (cannot be judged from range points: own body/legs); this is a scan-point check,
with no height or leg model.
min_move_mps: when obstacle limiting brings the allowed speed below it, the linear command becomes 0. The dog does not
walk below ~0.13 m/s anyway (2026-10-25 wall test: it stalled at 0.10-0.16 m/s commands and "stopped" by its own dead
band ~0.15-0.18 m before the wall, tape 0.2 m, which the user liked); cutting at 0.12 makes that stop ours, not the
floor's: with decel 0.5, reaction 0.2, stop gap 0.10 it happens at gap ~0.14 m.
"""
import math

import numpy as np


def safe_speed(gap_m, safe_stop_gap_m, decel_mps2, reaction_s):
    free = max(0.0, gap_m - safe_stop_gap_m)
    at = decel_mps2 * reaction_s
    return max(0.0, math.sqrt(at * at + 2.0 * decel_mps2 * free) - at)


def sweep_gap(points, ux, uy, half_len, half_wid):
    """Distance the rectangle can move along unit (ux, uy) before touching a point (N x 2 array); inf if none.
    A point already inside the rectangle gives 0."""
    if points is None or len(points) == 0:
        return math.inf
    enter = np.zeros(len(points))
    leave = np.full(len(points), np.inf)
    ok = np.ones(len(points), bool)
    for col, u, h in ((0, ux, half_len), (1, uy, half_wid)):
        p = points[:, col]
        if abs(u) < 1e-9:
            ok &= (p >= -h) & (p <= h)
        else:
            a, b = (p - h) / u, (p + h) / u
            enter = np.maximum(enter, np.minimum(a, b))
            leave = np.minimum(leave, np.maximum(a, b))
    hit = ok & (leave >= enter)
    return float(enter[hit].min()) if hit.any() else math.inf


def rotation_blocked(points, wz, half_len, half_wid, horizon_rad, step_rad=0.05):
    """True if turning the body rectangle by up to horizon_rad in the sign of wz makes a scan point enter it."""
    if points is None or len(points) == 0 or abs(wz) < 1e-3:
        return False
    inside0 = (np.abs(points[:, 0]) <= half_len) & (np.abs(points[:, 1]) <= half_wid)
    pts = points[~inside0]
    if len(pts) == 0:
        return False
    sign = 1.0 if wz > 0 else -1.0
    for k in range(1, int(math.ceil(horizon_rad / step_rad)) + 1):
        a = -sign * min(k * step_rad, horizon_rad)        # rotating the body by +a == rotating the points by -a
        c, s = math.cos(a), math.sin(a)
        x = c * pts[:, 0] - s * pts[:, 1]
        y = s * pts[:, 0] + c * pts[:, 1]
        if np.any((np.abs(x) <= half_len) & (np.abs(y) <= half_wid)):
            return True
    return False


def scan_points(ranges, angle_min, angle_inc, range_min, range_max):
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    keep = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack((r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])))


def clamp(cmd, max_v):
    """max_v: (forward, side, turn[, back])."""
    back = max_v[3] if len(max_v) > 3 else max_v[0]
    return (max(-back, min(max_v[0], cmd[0])), max(-max_v[1], min(max_v[1], cmd[1])), max(-max_v[2], min(max_v[2], cmd[2])))


def checks_for(mode, manual, hand):
    """Which checks a command gets (user decisions 2026-10-25, confirmed 2026-10-26):
      hand driving (teleop fresh, in ANY mode - mapping, or fixing a problem during navigation from the same teleop panel),
      mapping (mode map) and manual:=true: only the speed clamp, the dog's fault and the timeout - never limited;
      autonomous (mot_nav, mot_recover in navigation): + obstacle limiting, the scan check and localisation.
    -> (control name, obstacle_limit, loc_check)"""
    if hand or mode == 'map' or manual:
        return 'hand', False, False
    return 'auto', True, True


def limit(cmd, points, g, max_v, min_move_mps=0.0):
    """cmd: (vx, vy, wz). Returns (vx, vy, wz, gap_m, direction): clamped, linear part scaled to the safe speed."""
    vx, vy, wz = clamp(cmd, max_v)
    speed = math.hypot(vx, vy)
    if speed < 1e-3:
        horizon = max(0.15, abs(wz) * (g.brake_reaction_s + 0.3))
        if rotation_blocked(points, wz, g.body_half_length_m, g.body_half_width_m, horizon):
            return 0.0, 0.0, 0.0, 0.0, 'turn'
        return 0.0, 0.0, wz, math.inf, ''
    ux, uy = vx / speed, vy / speed
    gap = sweep_gap(points, ux, uy, g.body_half_length_m, g.body_half_width_m)
    allowed = safe_speed(gap, g.safe_stop_gap_m, g.brake_decel_mps2, g.brake_reaction_s)
    if allowed < speed and allowed < min_move_mps:
        allowed = 0.0
    scale = min(1.0, allowed / speed)
    direction = 'front' if ux > 0.7 else 'back' if ux < -0.7 else 'left' if uy > 0 else 'right'
    return vx * scale, vy * scale, wz, gap, direction
