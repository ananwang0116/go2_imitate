"""per_evidence's grid rules without ROS (tested offline). Incremental: each scan touches only the cells on its rays.

score (int8 per cell of the prior /map grid): a ray's floor cells -free_step, its obstacle cell +hit_step, clamped.
  proven occupied: score >= occ_threshold      proven free: score <= free_threshold      else: no evidence
Positive evidence with no new hit returns to score 0 (unknown/prior), never proven free, after a lifetime that depends on how long
the cell has been seen as occupied (2026-11-12, replaces the single 30 s occupied_ttl_s):
  DYNAMIC  seen < static_min_s (3 s) in total (a person walking by, a stray return): gone dynamic_ttl_s (5 s) after the last hit
  STATIC   seen >= static_min_s (a thing that stays while the dog walks up/past it):  gone static_ttl_s (90 s) after the last hit
A cell still in view is cleared much sooner by free rays through it (score), whatever its kind; the lifetimes only matter for cells
that left the view. No object recognition: only the time a cell has been continuously occupied (first_hit .. last_hit).
Evidence map (/per_evidence_map): -1 no evidence, 0 proven free, 100 proven occupied (static), 99 proven occupied (dynamic).
Both 99 and 100 block the planner; the split only drives the web colours.
Nav map (/per_nav_map, mot_nav's costmap input, spec P4 "a wall nobody has looked at is a cost, not a wall"):
  proven occupied -> 100; proven free -> 0; prior wall without evidence -> prior_wall_cost (default 60: passable but
  expensive and kept at a distance by mot_nav - it may have been a thing that stood there while mapping, the laser decides);
  everything else -> the prior value; a wall drawn by hand (hard, /per_hard_map) -> 100 whatever the laser says (glass);
  the cells under the dog's own body -> 0 (it stands there).
"""
import math
import time

import numpy as np


def frame_changed(prev, cur, tol_m=0.05, tol_rad=0.03):
    """True when the transform map -> odom (x, y, yaw) moved more than the tolerance: the evidence grid lives in `map`, so everything
    it holds was written at the OLD place. -> (changed, shift_m, turn_rad)."""
    shift = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
    turn = abs(math.atan2(math.sin(cur[2] - prev[2]), math.cos(cur[2] - prev[2])))
    return shift > tol_m or turn > tol_rad, shift, turn


class EvidenceGrid:
    def __init__(self, width, height, resolution, origin_xy, cfg):
        self.w, self.h, self.res = width, height, resolution
        self.ox, self.oy = origin_xy
        self.cfg = cfg
        self.score = np.zeros((height, width), dtype=np.int8)
        # float32 relative monotonic seconds: 1.4 MB for a 660x555 map (score itself .37 MB).
        self.epoch = time.monotonic()
        self.last_hit = np.full((height, width), -np.inf, dtype=np.float32)
        self.first_hit = np.full((height, width), -np.inf, dtype=np.float32)    # when this occupied episode began
        self.changed = False

    def reset(self):
        """Forget everything seen. Returns how many cells held evidence."""
        n = int((self.score != 0).sum())
        self.score[:] = 0
        self.last_hit[:] = -np.inf
        self.first_hit[:] = -np.inf
        self.changed = True
        return n

    def cells(self, xy):
        ij = np.floor((xy - (self.ox, self.oy)) / self.res).astype(np.int64)
        keep = (ij[:, 0] >= 0) & (ij[:, 0] < self.w) & (ij[:, 1] >= 0) & (ij[:, 1] < self.h)
        return ij[keep]

    def update(self, pose, angles, obstacle, free, now=None):
        """pose: (x, y, yaw) of base_footprint in map. angles/obstacle/free: per scan bin (base_footprint)."""
        c = self.cfg
        now = time.monotonic() if now is None else now
        x0, y0, yaw = pose
        a = angles + yaw
        dx, dy = np.cos(a), np.sin(a)
        # floor cells: sample each ray from body edge to its free range
        reach = np.minimum(free, c['max_range_m'])
        steps = np.arange(c['start_m'], c['max_range_m'], self.res * 0.7)
        d = steps[None, :]
        mask = d <= reach[:, None] - self.res                         # stop one cell before the last floor point
        fx = (x0 + dx[:, None] * d)[mask]
        fy = (y0 + dy[:, None] * d)[mask]
        if len(fx):
            ij = np.unique(self.cells(np.column_stack((fx, fy))), axis=0)
            s = self.score[ij[:, 1], ij[:, 0]].astype(np.int16) - c['free_step']
            new = np.clip(s, c['score_min'], c['score_max']).astype(np.int8)
            self.score[ij[:, 1], ij[:, 0]] = new
            gone = new <= 0                                     # no longer positive: the occupied episode is over
            self.first_hit[ij[gone, 1], ij[gone, 0]] = -np.inf
        hit = np.isfinite(obstacle) & (obstacle <= c['max_range_m'])
        if hit.any():
            hx, hy = x0 + dx[hit] * obstacle[hit], y0 + dy[hit] * obstacle[hit]
            ij = np.unique(self.cells(np.column_stack((hx, hy))), axis=0)
            before = self.score[ij[:, 1], ij[:, 0]]
            s = before.astype(np.int16) + c['hit_step']
            self.score[ij[:, 1], ij[:, 0]] = np.clip(s, c['score_min'], c['score_max']).astype(np.int8)
            starting = (before <= 0) | ~np.isfinite(self.first_hit[ij[:, 1], ij[:, 0]])
            self.first_hit[ij[starting, 1], ij[starting, 0]] = now - self.epoch
            self.last_hit[ij[:, 1], ij[:, 0]] = now - self.epoch
        self.changed = True

    def expire(self, now=None):
        """No hit for the cell's lifetime (dynamic_ttl_s or static_ttl_s): positive evidence becomes UNKNOWN (score 0), never proven free.
        Fusion restores the prior; hard walls are applied afterwards and cannot expire. Free observations still clear earlier.
        """
        now = time.monotonic() if now is None else now
        rows, cols = np.nonzero(self.score > 0)
        if not len(rows):
            return 0
        seen = self.last_hit[rows, cols]-self.first_hit[rows, cols]          # how long this cell has been occupied
        ttl = np.where(seen >= self.cfg['static_min_s'], self.cfg['static_ttl_s'], self.cfg['dynamic_ttl_s'])
        old = (now-self.epoch-self.last_hit[rows, cols]) >= ttl
        n = int(old.sum())
        if n:
            r, c = rows[old], cols[old]
            self.score[r, c] = 0
            self.last_hit[r, c] = -np.inf
            self.first_hit[r, c] = -np.inf
            self.changed = True
        return n

    def dynamic_mask(self):
        """Occupied cells that have not (yet) been seen long enough to count as static."""
        out = np.zeros(self.score.shape, dtype=bool)
        rows, cols = np.nonzero(self.score >= self.cfg['occ_threshold'])
        if len(rows):
            seen = self.last_hit[rows, cols]-self.first_hit[rows, cols]
            out[rows, cols] = seen < self.cfg['static_min_s']
        return out

    def evidence(self):
        out = np.full(self.score.shape, -1, dtype=np.int8)
        out[self.score <= self.cfg['free_threshold']] = 0
        out[self.score >= self.cfg['occ_threshold']] = 100
        out[self.dynamic_mask()] = 99
        return out

    def nav(self, prior, body_cells=None, hard=None):
        """prior: int8 grid (-1/0..100); hard: bool grid of hand-drawn walls or None. Returns the fused grid for mot_nav."""
        ev = self.evidence()
        out = prior.copy()
        wall = prior >= 65
        out[wall] = self.cfg['prior_wall_cost']
        out[ev == 0] = 0
        out[ev >= 99] = 100                    # static (100) and dynamic (99) obstacles both block
        if body_cells is not None and len(body_cells):
            out[body_cells[:, 1], body_cells[:, 0]] = 0
        if hard is not None:
            out[hard] = 100
        return out


DEFAULT_CFG = dict(max_range_m=6.0, start_m=0.25, free_step=1, hit_step=3, score_min=-5, score_max=10,
                   free_threshold=-2, occ_threshold=3, prior_wall_cost=60,
                   dynamic_ttl_s=5.0, static_ttl_s=90.0, static_min_s=3.0)
