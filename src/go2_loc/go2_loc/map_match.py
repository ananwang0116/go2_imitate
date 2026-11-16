"""Laser-vs-map scoring and search without ROS (tested offline), shared by loc_map_check and loc_align (spec 4).

DistanceField: distance (m) from every map cell to the nearest occupied cell (scipy EDT, computed once per map).
score(points_in_map) = share of points within tol_m of a wall (points outside the map count as misses).
search(): coarse grid (step_xy_m, step_yaw_rad) over the window, then a fine grid around the best; all candidate poses
of one yaw are scored in one numpy batch. Returns the best pose, its score and the best score >= min_sep_m away
(a high second score means the place is ambiguous).
"""
import math

import numpy as np
from scipy import ndimage


class DistanceField:
    def __init__(self, grid, resolution, origin_xy, occupied_threshold=65):
        self.res = resolution
        self.ox, self.oy = origin_xy
        self.h, self.w = grid.shape
        occ = grid >= occupied_threshold
        self.dist = ndimage.distance_transform_edt(~occ) * resolution if occ.any() else np.full(grid.shape, np.inf)

    def lookup(self, xy):
        """xy: ... x 2 array in map frame -> distance (inf outside the map)."""
        i = np.floor((xy[..., 0] - self.ox) / self.res).astype(np.int64)
        j = np.floor((xy[..., 1] - self.oy) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        out = np.full(i.shape, np.inf)
        out[inside] = self.dist[j[inside], i[inside]]
        return out


def to_map(points_body, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + c * points_body[:, 0] - s * points_body[:, 1], y + s * points_body[:, 0] + c * points_body[:, 1]))


def score(field, points_map, tol_m):
    if len(points_map) == 0:
        return float('nan')
    return float(np.mean(field.lookup(points_map) <= tol_m))


def _grid_scores(field, pts, xs, ys, yaw, tol_m):
    c, s = math.cos(yaw), math.sin(yaw)
    rx = c * pts[:, 0] - s * pts[:, 1]
    ry = s * pts[:, 0] + c * pts[:, 1]
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    cand = np.stack((X.ravel()[:, None] + rx[None, :], Y.ravel()[:, None] + ry[None, :]), axis=-1)   # C x N x 2
    sc = np.mean(field.lookup(cand) <= tol_m, axis=1)
    return X.ravel(), Y.ravel(), sc


def search(field, points_body, hint, win_xy_m, win_yaw_rad, tol_m, step_xy_m=0.1, step_yaw_rad=math.radians(3),
           fine_xy_m=0.02, fine_yaw_rad=math.radians(0.5), min_sep_m=0.5, max_points=400):
    pts = points_body
    if len(pts) > max_points:
        pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(int)]
    if len(pts) < 20:
        return None
    hx, hy, hyaw = hint
    xs = np.arange(hx - win_xy_m, hx + win_xy_m + 1e-9, step_xy_m)
    ys = np.arange(hy - win_xy_m, hy + win_xy_m + 1e-9, step_xy_m)
    rows = []
    for yaw in np.arange(hyaw - win_yaw_rad, hyaw + win_yaw_rad + 1e-9, step_yaw_rad):
        X, Y, sc = _grid_scores(field, pts, xs, ys, yaw, tol_m)
        rows.append(np.column_stack((X, Y, np.full(len(X), yaw), sc)))
    allc = np.vstack(rows)
    b = allc[np.argmax(allc[:, 3])]
    far = np.hypot(allc[:, 0] - b[0], allc[:, 1] - b[1]) >= min_sep_m
    second = float(allc[far, 3].max()) if far.any() else 0.0
    best = (b[0], b[1], b[2], b[3])
    fx = np.arange(b[0] - step_xy_m, b[0] + step_xy_m + 1e-9, fine_xy_m)
    fy = np.arange(b[1] - step_xy_m, b[1] + step_xy_m + 1e-9, fine_xy_m)
    for yaw in np.arange(b[2] - step_yaw_rad, b[2] + step_yaw_rad + 1e-9, fine_yaw_rad):
        X, Y, sc = _grid_scores(field, pts, fx, fy, yaw, tol_m)
        k = int(np.argmax(sc))
        if sc[k] > best[3]:
            best = (X[k], Y[k], yaw, sc[k])
    return {'pose': (float(best[0]), float(best[1]), float(math.atan2(math.sin(best[2]), math.cos(best[2])))),
            'score': float(best[3]), 'second': second, 'candidates': len(allc)}
