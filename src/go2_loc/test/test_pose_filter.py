"""Offline tests for loc_pose's jump filter. Run: colcon test --packages-select go2_loc"""
from go2_loc.loc_pose import PoseFilter


def test_normal_motion_and_old_samples():
    f = PoseFilter(2.0, 6.0, 3)
    assert f.feed(0.0, 0, 0, 0) == 'ok'
    assert f.feed(0.1, 0.03, 0, 0.05) == 'ok'       # 0.3 m/s
    assert f.feed(0.1, 0.5, 0, 0) == 'old'           # same stamp


def test_single_glitch_dropped_new_place_accepted():
    f = PoseFilter(2.0, 6.0, 3)
    f.feed(0.0, 0, 0, 0)
    assert f.feed(0.1, 5, 0, 0) == 'glitch'          # 50 m/s
    assert f.feed(0.2, 0.02, 0, 0) == 'ok'           # back to normal, baseline unchanged
    assert f.feed(0.3, 5, 0, 0) == 'glitch'
    assert f.feed(0.4, 5.01, 0, 0) == 'glitch'
    assert f.feed(0.5, 5.02, 0, 0) == 'rebase'       # stayed for 3 samples
    assert f.last[1] == 5.02


def test_map_to_odom_from_click():
    """map->odom must put base_footprint (odom pose) exactly at the clicked map pose."""
    import math
    from go2_loc.loc_align import map_to_odom_from
    px, py, pyaw, ox, oy, oyaw = 3.0, -1.0, 1.2, 0.5, 0.2, 0.3
    x, y, yaw = map_to_odom_from(px, py, pyaw, ox, oy, oyaw)
    c, s = math.cos(yaw), math.sin(yaw)
    assert abs(x + c * ox - s * oy - px) < 1e-9 and abs(y + s * ox + c * oy - py) < 1e-9
    assert abs(math.atan2(math.sin(yaw + oyaw - pyaw), math.cos(yaw + oyaw - pyaw))) < 1e-9


def test_hysteresis():
    from go2_loc.loc_pose import Hysteresis
    h = Hysteresis(3.0)
    assert not h.update(0, True, False) and not h.update(2.9, True, False)
    assert h.update(3.0, True, False)                       # bad for 3 s -> on
    assert h.update(4.0, False, False)                      # in between: stays on
    assert h.update(5.0, False, True) and not h.update(8.0, False, True)   # good for 3 s -> off


def test_map_match_search_recovers_offset():
    import math
    import numpy as np
    from go2_loc.map_match import DistanceField, search, score, to_map
    grid = np.zeros((200, 200), dtype=np.int8)                  # 10 x 10 m room with an inner wall
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    grid[60:140, 120] = 100
    field = DistanceField(grid, 0.05, (0.0, 0.0))
    true = (4.0, 5.0, 0.3)
    # simulate a scan: walls seen from the true pose, in the body frame
    walls = np.argwhere(grid >= 65)[:, ::-1] * 0.05 + 0.025
    c, s = math.cos(-true[2]), math.sin(-true[2])
    d = walls - true[:2]
    body = np.column_stack((c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]))
    body = body[np.hypot(body[:, 0], body[:, 1]) < 6][::3]
    assert score(field, to_map(body, true), 0.1) > 0.95
    r = search(field, body, (4.5, 4.6, 0.3 + math.radians(15)), 1.0, 0.52, 0.1)
    assert r['score'] > 0.9 and math.hypot(r['pose'][0] - 4.0, r['pose'][1] - 5.0) < 0.06
    assert abs(r['pose'][2] - 0.3) < math.radians(1.5)
