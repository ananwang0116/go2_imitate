"""Offline tests for per_scan's point processing. Run: colcon test --packages-select go2_per"""
import math
import time
from pathlib import Path

import numpy as np

from go2_common import geometry
from go2_per import per_logic as pl

G = geometry.load(Path(__file__).resolve().parents[2] / 'go2_bringup' / 'config' / 'geometry.yaml')


def test_old_person_evidence_expires_to_unknown_and_restores_prior_not_free():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    g = EvidenceGrid(100, 100, .05, (-2.5, -2.5), DEFAULT_CFG)
    t = g.epoch
    g.update((0., 0., 0.), np.array([0.]), np.array([1.5]), np.array([0.]), now=t)
    assert g.evidence()[50, 80] == 99 and (g.evidence() == 99).sum() == 1      # one glance: dynamic (99), still blocks
    assert g.expire(now=t+4.9) == 0
    prior = np.full((100, 100), -1, dtype=np.int8)
    prior[50, 80] = 100                      # old saved wall should become PRIOR (60), not confirmed free
    assert g.expire(now=t+5.1) > 0 and g.evidence()[50, 80] == -1
    assert g.nav(prior)[50, 80] == 60
    prior[50, 80] = -1
    assert g.nav(prior)[50, 80] == -1
    hard = np.zeros((100, 100), bool)
    hard[50, 80] = True
    assert g.nav(prior, hard=hard)[50, 80] == 100


def test_reobserved_obstacle_refreshes_expiry_and_reset_forgets_time():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    g = EvidenceGrid(100, 100, .05, (-2.5, -2.5), DEFAULT_CFG)
    t = g.epoch
    for dt in [0., 4.]:                      # seen for 4 s in total: static (>= static_min_s 3 s)
        g.update((0., 0., 0.), np.array([0.]), np.array([1.5]), np.array([0.]), now=t+dt)
    assert g.evidence()[50, 80] == 100
    assert g.expire(now=t+4+89.9) == 0 and g.evidence()[50, 80] == 100
    assert g.expire(now=t+4+90.1) == 1
    g.update((0., 0., 0.), np.array([0.]), np.array([1.5]), np.array([0.]), now=t+100.)
    g.reset()
    assert np.isneginf(g.last_hit).all() and np.isneginf(g.first_hit).all()


def test_dynamic_and_static_obstacles_have_their_own_lifetimes_and_colours_code():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    cfg = dict(DEFAULT_CFG)
    g = EvidenceGrid(100, 100, .05, (-2.5, -2.5), cfg)
    t = g.epoch
    ray = lambda a, now, r=1.5: g.update((0., 0., 0.), np.array([a]), np.array([r]), np.array([0.]), now=now)
    ray(0., t)                                        # a passer-by: one glance at 1.5 m
    ray(.5, t); ray(.5, t+1.); ray(.5, t+2.); ray(.5, t+3.5)      # a box: the same cell seen over 3.5 s
    ev = g.evidence()
    cells = {(int(j), int(i)) for j, i in zip(*np.nonzero(ev >= 99))}
    assert any(ev[j, i] == 99 for j, i in cells) and any(ev[j, i] == 100 for j, i in cells)
    nav = g.nav(np.zeros((100, 100), dtype=np.int8))
    assert all(nav[j, i] == 100 for j, i in cells)    # both kinds block the planner
    g.expire(now=t+3.5+5.1)                            # dynamic gone after 5 s, static still there
    ev = g.evidence()
    assert (ev == 99).sum() == 0 and (ev == 100).sum() >= 1
    g.expire(now=t+3.5+90.1)
    assert (g.evidence() >= 99).sum() == 0


def test_lifetimes_are_live_adjustable_through_the_shared_cfg_and_free_rays_end_an_episode():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    cfg = dict(DEFAULT_CFG)
    g = EvidenceGrid(100, 100, .05, (-2.5, -2.5), cfg)
    t = g.epoch
    g.update((0., 0., 0.), np.array([0.]), np.array([1.5]), np.array([0.]), now=t)
    cfg['dynamic_ttl_s'] = 20.                         # the layer panel slider changes the dict the grid reads
    assert g.expire(now=t+10.) == 0 and g.expire(now=t+20.1) == 1
    for k in range(3):                                 # hit, then free rays through the cell: a moving thing left
        g.update((0., 0., 0.), np.array([0.]), np.array([1.5]), np.array([0.]), now=t+30.+k)
    for k in range(14):
        g.update((0., 0., 0.), np.array([0.]), np.array([3.0]), np.array([2.9]), now=t+40.+k*.1)
    assert g.evidence()[50, 80] == 0 and np.isneginf(g.first_hit[50, 80])      # proven free again, episode forgotten


def floor(n=4000, tilt=0.03, seed=1):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-6, 6, size=(n, 2))
    z = tilt * xy[:, 0] + rng.normal(0, 0.003, n)
    return np.column_stack((xy, z))


def test_cloud_xyz_parses_and_drops_nan():
    pts = np.array([[1, 2, 3, 9], [np.nan, 0, 0, 9]], dtype='<f4')
    out = pl.cloud_xyz(pts.tobytes(), 16, (0, 4, 8), 2)
    assert out.shape == (1, 3) and tuple(out[0]) == (1, 2, 3)


def test_ground_plane_finds_tilted_floor():
    plane = pl.ground_plane(floor())
    assert abs(plane[0] - 0.03) < 0.005 and abs(plane[2]) < 0.01


def test_scans_box_wall_floor_and_self():
    f = floor(tilt=0.0)
    box = np.array([[1.0, y, 0.2] for y in np.linspace(-0.1, 0.1, 5)])      # 1 m ahead, 20 cm tall
    leg = np.array([[0.2, 0.1, 0.1]])                                       # inside the body: ignored
    pts = np.vstack((f, box, leg))
    obst, mapping, free = pl.scans(pts, G, pl.DEFAULT_CFG)
    ahead = 360                                                             # bin of angle 0 for 720 bins
    assert math.isclose(obst[ahead], 1.0, abs_tol=0.01)
    assert mapping[ahead * 360 // 720] < 1.1                                 # 20 cm is inside the map band too
    assert 0 < free[ahead] < 1.0                                            # floor seen only up to the box
    assert np.isinf(obst[0])                                                # nothing behind


def test_speed_budget():
    pts = np.vstack((floor(5000), np.random.default_rng(2).uniform(-5, 5, size=(1000, 3)) * [1, 1, 0.1] + [0, 0, 0.3]))
    t0 = time.perf_counter()
    for _ in range(20):
        plane = pl.ground_plane(pts)
        pl.scans(pl.relative_to_plane(pts, plane), G, pl.DEFAULT_CFG)
    assert (time.perf_counter() - t0) / 20 < 0.015                          # < 15 ms per cloud at 6k points


def test_evidence_grid_free_hit_and_fusion():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    g = EvidenceGrid(100, 100, 0.05, (-2.5, -2.5), DEFAULT_CFG)          # 5 x 5 m, robot at the centre
    angles = np.array([0.0])
    for _ in range(3):
        g.update((0.0, 0.0, 0.0), angles, np.array([1.5]), np.array([1.4]))
    ev = g.evidence()
    assert ev[50, 50 + 20] == 0                                           # 1.0 m ahead: floor seen 3x -> proven free
    assert ev[50, 50 + 30] >= 99                                          # 1.5 m: obstacle hit 3x -> proven occupied (99: not yet seen 3 s)
    assert ev[60, 50] == -1                                               # never looked at
    prior = np.zeros((100, 100), dtype=np.int8)
    prior[50, 50 + 20] = 100                                              # old map wall where the floor was seen
    prior[80, 80] = 100                                                   # old wall nobody looked at
    nav = g.nav(prior)
    assert nav[50, 70] == 0 and nav[50, 80] == 100 and nav[80, 80] == DEFAULT_CFG['prior_wall_cost']
    hard = np.zeros((100, 100), dtype=bool)
    hard[50, 70] = hard[10, 10] = True                                    # drawn by hand: the laser saw floor there (glass)
    nav = g.nav(prior, hard=hard)
    assert nav[50, 70] == 100 and nav[10, 10] == 100 and nav[80, 80] == DEFAULT_CFG['prior_wall_cost']


def test_evidence_update_speed():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    g = EvidenceGrid(600, 600, 0.05, (-15, -15), DEFAULT_CFG)
    a = -math.pi + 2 * math.pi / 720 * np.arange(720)
    obst = np.full(720, 4.0)
    free = np.full(720, 3.9)
    t0 = time.perf_counter()
    for _ in range(10):
        g.update((0.0, 0.0, 0.3), a, obst, free)
    assert (time.perf_counter() - t0) / 10 < 0.10                         # worst case (720 full rays); 58 ms seen once under load; real node ~19 % CPU
