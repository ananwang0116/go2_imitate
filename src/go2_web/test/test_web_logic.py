import struct

import cv2
import numpy as np
import pytest

from go2_web import web_logic as wl


def test_recent_cloud_expires_people_but_refreshes_visible_wall():
    a = wl.RecentCloudAccumulator(ttl_s=5)
    person = np.array([[1., 0., .2], [1., 0., .4]], dtype=np.float32)
    wall = np.array([[3., 0., .2], [3., 0., .4]], dtype=np.float32)
    a.add(np.vstack((person, wall)), now=0.)
    a.add(wall, now=4.)
    rev = a.rev
    assert a.expire(now=5.1) == 1 and a.rev > rev
    assert len(a.pts) == 2 and np.allclose(a.pts[:, 0], 3.)
    a.expire(now=9.1)                         # no new frames are needed to expire the last ones
    assert len(a.pts) == 0
    a.add(person, now=10.)
    a.clear()
    assert not len(a.pts) and not a.batches
    full = wl.CloudAccumulator()
    full.add(person)
    full.expire(now=1e20)
    assert len(full.pts) == 2                 # mapping stays cumulative


def test_recent_cloud_input_and_output_are_bounded_and_voxels_unique():
    a = wl.RecentCloudAccumulator(ttl_s=5., cap=20)
    p = np.array([[x, 0., .2] for x in range(10)], dtype=np.float32)
    for i in range(30):
        a.add(p, now=i * .01)
    assert sum(len(p) for _, p in a.batches) <= 20
    assert len(a.pts) == 10


def test_cloud_accumulation_requires_fresh_usable_localisation():
    assert wl.cloud_pose_usable(0, .1) and wl.cloud_pose_usable(1, .1)
    assert not wl.cloud_pose_usable(2, .1)
    assert not wl.cloud_pose_usable(None, 0)
    assert not wl.cloud_pose_usable(0, 2.)


def make_map(tmp_path, w=100, h=80, res=0.05, origin=(-2.0, -1.0)):
    d = tmp_path / 'm'
    d.mkdir()
    img = np.full((h, w), 254, np.uint8)
    cv2.imwrite(str(d / 'map.pgm'), img)
    (d / 'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0]\n'
                                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return d


def test_names_reject_paths():
    assert wl.check_name('lab_1-东侧') == 'lab_1-东侧'
    for bad in ('', '../x', 'a/b', 'a.b', 'x' * 41, 5):
        with pytest.raises(ValueError):
            wl.check_name(bad)


def test_world_pixel_roundtrip_matches_grid_convention(tmp_path):
    d = make_map(tmp_path)
    meta, _ = wl.load_meta(d)
    col, row = wl.world_to_px(meta, -2.0, -1.0)        # origin = bottom-left corner of the image
    assert col == pytest.approx(0) and row == pytest.approx(meta['height'] - 1)
    col, row = wl.world_to_px(meta, -2.0, -1.0 + 79 * 0.05)
    assert row == pytest.approx(0)                      # largest y = top row


def test_wall_and_erase_and_restore(tmp_path):
    d = make_map(tmp_path)
    wl.save_edits(d, [{'type': 'wall', 'x1': -1.0, 'y1': 0.0, 'x2': 1.0, 'y2': 0.0}])
    meta, img = wl.load_meta(d)
    c, r = wl.world_to_px(meta, 0.0, 0.0)
    assert img[round(r), round(c)] == 0 and img[0, 0] == 254
    wl.save_edits(d, [{'type': 'erase', 'x': 0.0, 'y': 0.0, 'r': 0.3}])
    _, img = wl.load_meta(d)
    assert img[round(r), round(c)] == 254
    assert len(list((d / 'backup').glob('*.pgm'))) >= 1
    wl.restore_last(d)                                  # newest backup = the wall version
    _, img = wl.load_meta(d)
    assert img[round(r), round(c)] == 0
    with pytest.raises(ValueError):
        wl.apply_edits(img, meta, [{'type': 'erase', 'x': 0, 'y': 0, 'r': 99}])
    with pytest.raises(ValueError):
        wl.apply_edits(img, meta, [{'type': 'nope'}])


def test_drawn_walls_go_into_the_hard_layer(tmp_path):
    d = make_map(tmp_path)
    assert not (d / 'hard.pgm').exists()
    wl.save_edits(d, [{'type': 'wall', 'x1': -1.0, 'y1': 0.0, 'x2': 1.0, 'y2': 0.0}])
    meta, img = wl.load_meta(d)
    c, r = wl.world_to_px(meta, 0.0, 0.0)
    hard = wl.load_hard(d, img.shape)
    assert hard[round(r), round(c)] == 0 and hard[0, 0] == 255           # the drawn wall is hard
    wl.save_edits(d, [{'type': 'erase', 'x': 0.0, 'y': 0.0, 'r': 0.3}])  # the eraser clears both layers
    assert wl.load_hard(d, img.shape)[round(r), round(c)] == 255
    wl.restore_last(d)                                                   # back to "wall drawn": hard again
    assert wl.load_hard(d, img.shape)[round(r), round(c)] == 0
    wl.restore_last(d)                                                   # back to before any edit: no hard layer at all
    assert not (d / 'hard.pgm').exists()
    _, img = wl.load_meta(d)
    assert img[round(r), round(c)] == 254


def test_list_maps_and_png(tmp_path):
    d = make_map(tmp_path)
    assert [m['name'] for m in wl.list_maps(tmp_path)] == ['m']
    meta, png = wl.map_png(d)
    assert png[:4] == b'\x89PNG' and meta['width'] == 100


def test_pcd_binary_and_compressed(tmp_path):
    pts = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], np.float32)
    head = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA {}\n')
    (tmp_path / 'b.pcd').write_bytes(head.format('binary').encode() + pts.tobytes())
    np.testing.assert_allclose(wl.read_pcd_xyz(tmp_path / 'b.pcd'), pts)
    body = pts.T.copy().tobytes()                       # column-major, stored as literal lzf runs (<= 32 bytes each)
    comp = b''.join(bytes([len(body[i:i + 32]) - 1]) + body[i:i + 32] for i in range(0, len(body), 32))
    (tmp_path / 'c.pcd').write_bytes(head.format('binary_compressed').encode() + struct.pack('<II', len(comp), len(body)) + comp)
    np.testing.assert_allclose(wl.read_pcd_xyz(tmp_path / 'c.pcd'), pts)


def test_accumulator_voxel_cap_and_order():
    acc = wl.CloudAccumulator(voxel=0.1, cap=5)
    acc.add(np.array([[0.01, 0, 0], [0.02, 0, 0], [1, 0, 0]], np.float32))       # first two share a voxel
    assert len(acc.pts) == 2
    acc.add(np.array([[x, 0, 0] for x in (2, 3, 4, 5, 6)], np.float32))
    assert len(acc.pts) == 5 and acc.pts[-1][0] == 6 and acc.pts[0][0] == 2      # 7 points, cap 5: the two oldest (0, 1) dropped
    acc.add(np.array([[0, 0, 99]], np.float32))                                    # outside z_range
    assert len(acc.pts) == 5
    r = acc.rev
    acc.clear()
    assert len(acc.pts) == 0 and acc.rev == r + 1


def test_accumulator_incremental_equals_the_full_recompute_and_is_fast():
    import time
    rng = np.random.default_rng(3)
    fast, ref = wl.CloudAccumulator(voxel=0.05, cap=10_000_000), []
    for _ in range(40):
        batch = (rng.random((2000, 3)) * [8, 6, 2]).astype(np.float32)
        fast.add(batch)
        ref.append(batch)
    allp = np.concatenate(ref)
    _, idx = np.unique(wl.voxel_keys(allp, 0.05), return_index=True)                  # the old way: first point per voxel
    assert len(fast.pts) == len(idx) and set(map(tuple, fast.pts.round(4))) == set(map(tuple, allp[np.sort(idx)].round(4)))
    big = wl.CloudAccumulator(voxel=0.05, cap=400_000)
    big.add((rng.random((400_000, 3)) * [40, 30, 3]).astype(np.float32))
    t0 = time.perf_counter()
    big.add((rng.random((2000, 3)) * [40, 30, 3]).astype(np.float32))                 # one 3 Hz batch on a full-size cloud
    assert time.perf_counter() - t0 < 0.25


def test_transform_and_cloud2():
    q = (0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))   # +90 deg about z
    out = wl.transform_points(np.array([[1, 0, 0]], np.float32), (10, 0, 0), q)
    np.testing.assert_allclose(out, [[10, 1, 0]], atol=1e-5)
    data = np.array([[1, 2, 3, 0.5], [np.nan, 0, 0, 0]], np.float32).tobytes()
    pts = wl.parse_cloud2([('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7)], 16, data, 2)
    np.testing.assert_allclose(pts, [[1, 2, 3]])
    assert wl.parse_cloud2([('x', 0, 7)], 16, data, 2).shape == (0, 3)


def test_mission_rows():
    rows = wl.mission_rows([{'x': 1, 'y': 2, 'yaw': 0.5, 'name': 'a'}, {'x': 3.14159, 'y': 0}])
    assert rows[0] == {'name': 'a', 'x': 1.0, 'y': 2.0, 'yaw': 0.5}
    assert rows[1]['name'] == 'wp2' and rows[1]['x'] == 3.142 and 'yaw' not in rows[1]
    for bad in ([], None, [{'x': float('nan'), 'y': 0}], [{'y': 0}]):
        with pytest.raises((ValueError, KeyError)):
            wl.mission_rows(bad)


def test_map_health_from_the_real_failed_run():
    # the events of runs/20261026_173201_map (the pose jumped 5.7 m while the dog stood, 1 s after an IMU hole)
    gap = wl.map_health_item('dev', 'IMU_GAP', {'count': '2', 'longest_ms': '175', 'late_ms': '1075.336'}, 1793007137.6)
    jump = wl.map_health_item('loc', 'JUMP', {'result': 'glitch', 'count': '1', 'x_m': '-4.036', 'y_m': '-4.291', 'from_x_m': '-0.041',
                                              'from_y_m': '-0.222'}, 1793007137.9)
    assert gap['level'] == 'warn' and gap['kind'] == 'imu_gap' and '175' in gap['text']
    assert jump['level'] == 'bad' and jump['kind'] == 'jump' and '5.7 m' in jump['text']
    h = wl.map_health([gap, jump])
    assert h['bad'] and h['n'] == 2 and h['first'] is jump          # the first RED one: where the map went wrong
    assert wl.map_health([gap])['first'] is gap
    assert not wl.map_health([gap])['bad'] and wl.map_health([]) == {'bad': False, 'n': 0, 'first': None, 'items': []}
    # small IMU holes (the dog's normal 30-100 ms hiccups) and ordinary events are not reported
    assert wl.map_health_item('dev', 'IMU_GAP', {'longest_ms': '60'}, 0) is None
    assert wl.map_health_item('dev', 'LINK_UP', {'topic': 'lowstate'}, 0) is None
    lost = wl.map_health_item('loc', 'STATE', {'from_state': 'OK', 'to': 'LOST', 'reason': 'kilo_stale'}, 0)
    assert lost['level'] == 'bad'
    assert wl.map_health_item('loc', 'STATE', {'from_state': 'LOST', 'to': 'OK', 'reason': 'fresh'}, 0) is None
    stall = wl.map_health_item('dev', 'STALL', {'longest_ms': '1450', 'slowest_cb': 'publish_state'}, 0)
    assert stall['level'] == 'warn' and 'publish_state' in stall['text']


def test_pose_is_fresh():
    from go2_web.web_server import pose_is_fresh
    s = 10**9
    assert pose_is_fresh(100 * s, 101 * s, 2.0)
    assert not pose_is_fresh(100 * s, 105 * s, 2.0)       # the previous run's last pose
    assert pose_is_fresh(0, 500 * s, 2.0)                  # static-only chain


def test_cyan_evidence_overlay_tracks_expiry_and_world_y():
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG
    grid = EvidenceGrid(20, 20, .1, (-1., -1.), DEFAULT_CFG)
    grid.update((0., 0., 0.), np.array([0.]), np.array([.5]), np.array([0.]), now=grid.epoch)
    image = cv2.imdecode(np.frombuffer(wl.evidence_png(grid.evidence(), 20, 20), np.uint8), cv2.IMREAD_UNCHANGED)
    # A cell seen once is DYNAMIC (yellow). Occupied world cell (15, 10) is image row 9.
    assert tuple(image[9, 15]) == (0, 196, 255, 230)
    assert np.count_nonzero(image[:, :, 3]) == 9  # display halo; planner grid is unchanged
    grid.update((0., 0., 0.), np.array([0.]), np.array([.5]), np.array([0.]), now=grid.epoch + 4.)
    image = cv2.imdecode(np.frombuffer(wl.evidence_png(grid.evidence(), 20, 20), np.uint8), cv2.IMREAD_UNCHANGED)
    assert tuple(image[9, 15]) == (212, 188, 0, 220)      # seen 4 s: STATIC (cyan)
    assert grid.expire(now=grid.epoch + 4. + 89.9) == 0
    grid.expire(now=grid.epoch + 4. + 90.1)
    image = cv2.imdecode(np.frombuffer(wl.evidence_png(grid.evidence(), 20, 20), np.uint8), cv2.IMREAD_UNCHANGED)
    assert not image[:, :, 3].any()


def test_initialpose_feedback_requires_current_event_and_new_map_tf():
    feedback=wl.AlignmentFeedback()
    rid=feedback.request(10_000,7,1.)
    feedback.event('ALIGN_OK',{'score':'1.0'},9_000)
    assert feedback.advance(8,1.1)=='pending'
    feedback.event('ALIGN_OK',{'score':'.9'},11_000)
    assert feedback.advance(7,1.2)=='pending'  # result arrived before its static TF
    assert feedback.advance(8,1.3)=='ok'
    assert feedback.status()['request_id']==rid
    assert feedback.status()['fields']['score']=='.9'


def test_initialpose_feedback_weak_failure_timeout_and_session_reset():
    feedback=wl.AlignmentFeedback()
    feedback.request(100,1,0.)
    feedback.event('ALIGN_WEAK',{'reason':'ambiguous'},101)
    assert feedback.advance(2,.3)=='weak'
    feedback.request(200,2,1.)
    feedback.event('ALIGN_FAIL',{'reason':'no_loc_odom'},201)
    assert feedback.status()['state']=='failed'
    feedback.request(300,2,2.)
    assert feedback.advance(2,10.1)=='failed'
    assert feedback.status()['reason']=='alignment_feedback_timeout'
    old=feedback.sequence
    feedback.reset()
    assert feedback.status()['state']=='unaligned'
    assert feedback.request(1,0,11.)>old


def test_initialpose_does_not_accept_another_requests_late_result():
    feedback=wl.AlignmentFeedback()
    feedback.request(100,3,0.)
    feedback.event('ALIGN_OK',{'request_stamp_ns':'90'},150)
    assert feedback.advance(4,.2)=='pending'
    feedback.event('ALIGN_WEAK',{'request_stamp_ns':'100','reason':'ambiguous'},160)
    assert feedback.advance(4,.3)=='weak'


def test_task_markers_carry_the_heading_for_the_arrow():
    import threading
    from types import SimpleNamespace as NS
    from go2_web.web_server import WebNode
    n = object.__new__(WebNode)
    n._accept_inputs, n.session_id, n.lock, n.markers = True, 's', threading.Lock(), []

    def marker(ns, i, x=0., y=0., scale=1., text='', z=0., w=1.):
        return NS(ns=ns, id=i, action=0, ADD=0, pose=NS(position=NS(x=x, y=y), orientation=NS(z=z, w=w)),
                  scale=NS(x=scale), text=text)
    import math
    n.on_markers(NS(markers=[marker('wp', 0, 1., 2., 1.2), marker('wp', 1, text='1 PENDING'),
                             marker('wp_yaw', 0, z=math.sin(.6), w=math.cos(.6))]))
    assert len(n.markers) == 1 and n.markers[0]['state'] == 'PENDING' and abs(n.markers[0]['yaw'] - 1.2) < 1e-9
    assert abs(n.markers[0]['r'] - .6) < 1e-9
