"""per_scan's point processing without ROS (tested offline). numpy, vectorised; one 15 Hz cloud (~5k points) ~2-4 ms.

Frames: input points are in base_footprint (x forward, z up, z=0 = where the feet stand).
- ground plane: RANSAC over near-horizontal candidates (all 64 hypotheses evaluated in one batch); heights are
  taken relative to it, so a tilted body or slope does not turn the floor into obstacles (from ~/go2_kilo nav_perception).
- self mask: the body rectangle + self_margin_m (the dog sees its own legs).
- obstacle scan: nearest return per bin with min_h + range_slope*r <= z <= max_h; no return -> inf.
- map scan: same with the mapping height band (walls, not low clutter).
- free scan: per bin, the farthest GROUND return that is nearer than the obstacle in that bin = "the laser saw floor
  up to here" -> per_evidence marks those cells proven free (spec P4: evidence, not the prior map).
"""
import math

import numpy as np

_RNG = np.random.default_rng(0)


def cloud_xyz(data: bytes, point_step: int, offsets: tuple, count: int) -> np.ndarray:
    """x, y, z float32 fields of a PointCloud2 buffer -> N x 3 float64, NaN rows removed."""
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3, 'offsets': list(offsets), 'itemsize': point_step})
    a = np.frombuffer(data, dtype=dt, count=count)
    pts = np.column_stack((a['x'], a['y'], a['z'])).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def transform(points, rot3x3, trans3):
    return points @ np.asarray(rot3x3).T + np.asarray(trans3)


def ground_plane(points, hypotheses=64, tol_m=0.012):
    """z = a*x + b*y + c of the floor, or (0, 0, 0) when no convincing floor is found."""
    r = np.hypot(points[:, 0], points[:, 1])
    cand = points[(r > 0.5) & (r < 6.0) & (np.abs(points[:, 2]) < 0.18)]
    if len(cand) < 100:
        return np.zeros(3)
    cand = cand[::max(1, len(cand) // 1500)]
    idx = _RNG.integers(0, len(cand), size=(hypotheses, 3))
    trios = cand[idx]                                                  # H x 3 x 3
    A = np.concatenate((trios[:, :, :2], np.ones((hypotheses, 3, 1))), axis=2)
    det = np.linalg.det(A)
    good = np.abs(det) > 0.01
    if not good.any():
        return np.zeros(3)
    planes = np.linalg.solve(A[good], trios[good][:, :, 2])            # G x 3
    sane = (np.linalg.norm(planes[:, :2], axis=1) <= 0.15) & (np.abs(planes[:, 2]) <= 0.12)
    planes = planes[sane]
    if not len(planes):
        return np.zeros(3)
    err = np.abs(cand[:, 2][None, :] - planes[:, 0:1] * cand[:, 0][None, :] - planes[:, 1:2] * cand[:, 1][None, :] - planes[:, 2:3])
    inl = err < tol_m
    counts = inl.sum(axis=1)
    best = int(np.argmax(counts))
    if counts[best] < max(100, 0.25 * len(cand)):
        return np.zeros(3)
    g = cand[inl[best]]
    if np.ptp(g[:, 0]) < 2.0 or np.ptp(g[:, 1]) < 2.0:                 # a box top is not the floor
        return np.zeros(3)
    return np.linalg.lstsq(np.column_stack((g[:, :2], np.ones(len(g)))), g[:, 2], rcond=None)[0]


def relative_to_plane(points, plane):
    out = points.copy()
    out[:, 2] -= points[:, 0] * plane[0] + points[:, 1] * plane[1] + plane[2]
    return out


def outside_body(points, half_len, half_wid, margin):
    return ~((np.abs(points[:, 0]) <= half_len + margin) & (np.abs(points[:, 1]) <= half_wid + margin))


def bin_index(points, bins):
    a = np.arctan2(points[:, 1], points[:, 0])
    return (np.floor((a + math.pi) / (2 * math.pi / bins)).astype(int)) % bins


def nearest_per_bin(points, bins, keep):
    ranges = np.full(bins, np.inf)
    if keep.any():
        p = points[keep]
        np.minimum.at(ranges, bin_index(p, bins), np.hypot(p[:, 0], p[:, 1]))
    return ranges


def scans(points, g, cfg):
    """points: base_footprint, relative to the floor. Returns (obstacle, map, free) range arrays."""
    r = np.hypot(points[:, 0], points[:, 1])
    z = points[:, 2]
    out = outside_body(points, g.body_half_length_m, g.body_half_width_m, cfg['self_margin_m']) & (r <= cfg['max_range_m'])
    obst_keep = out & (z >= cfg['obstacle_min_height_m'] + cfg['range_slope'] * r) & (z <= cfg['obstacle_max_height_m']) \
        & (r >= cfg['obstacle_min_range_m'])
    map_keep = out & (z >= cfg['map_min_height_m']) & (z <= cfg['map_max_height_m']) & (r >= cfg['map_min_range_m'])
    obstacle = nearest_per_bin(points, cfg['obstacle_bins'], obst_keep)
    mapping = nearest_per_bin(points, cfg['map_bins'], map_keep)
    ground_keep = out & (np.abs(z) < cfg['ground_tol_m'] + cfg['range_slope'] * r)
    free = np.zeros(cfg['obstacle_bins'])
    if ground_keep.any():
        p = points[ground_keep]
        b = bin_index(p, cfg['obstacle_bins'])
        rr = np.hypot(p[:, 0], p[:, 1])
        before = rr < obstacle[b]                                      # floor seen in front of the obstacle only
        np.maximum.at(free, b[before], rr[before])
    return obstacle, mapping, free


DEFAULT_CFG = dict(self_margin_m=0.02, max_range_m=10.0, obstacle_min_height_m=0.05, obstacle_max_height_m=0.60,
                   obstacle_min_range_m=0.15, range_slope=0.012, map_min_height_m=0.15, map_max_height_m=0.60,
                   map_min_range_m=0.35, ground_tol_m=0.03, obstacle_bins=720, map_bins=360)
