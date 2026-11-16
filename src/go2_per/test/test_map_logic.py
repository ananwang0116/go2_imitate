import cv2
import numpy as np
import pytest

from go2_per.map_logic import load_map


def write(tmp_path, img, extra=''):
    cv2.imwrite(str(tmp_path / 'map.pgm'), img)
    (tmp_path / 'map.yaml').write_text(f'image: map.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0]\n{extra}')
    return tmp_path / 'map.yaml'


def test_trinary_rule_and_orientation(tmp_path):
    img = np.full((3, 4), 254, np.uint8)
    img[0, 0] = 0          # top-left of the image = occupied
    img[2, 3] = 205        # bottom-right = the tool's "unknown" (p = 0.19608)
    img[1, 1] = 100        # p = 0.608: neither free nor occupied -> unknown
    data, res, origin = load_map(write(tmp_path, img, 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'))
    assert data.shape == (3, 4) and res == 0.05 and origin == (-1.0, -2.0)
    assert data[2, 0] == 100          # image row 0 is the TOP: grid row 2 (largest y)
    assert data[0, 3] == -1           # 205 = unknown with free_thresh 0.196 (ROS map_saver's value, the tool writes it)
    assert data[0, 0] == 0            # 254 = free
    assert data[1, 1] == -1
    assert data.dtype == np.int8
    old, _, _ = load_map(write(tmp_path, img, 'free_thresh: 0.25\n'))
    assert old[0, 3] == 0             # the old maps' 0.25 read unknown as FREE (76 % of my_map_merged, fixed 2026-10-26)
    default, _, _ = load_map(write(tmp_path, img))
    assert default[0, 3] == -1


def test_hard_layer(tmp_path):
    from go2_per.map_logic import load_hard
    img = np.full((3, 4), 254, np.uint8)
    y = write(tmp_path, img)
    assert not load_hard(y, (3, 4)).any()                    # no hard.pgm = nothing hard
    hard = np.full((3, 4), 255, np.uint8)
    hard[0, 1] = 0                                           # top row in the image = largest y = grid row 2
    cv2.imwrite(str(tmp_path / 'hard.pgm'), hard)
    h = load_hard(y, (3, 4))
    assert h[2, 1] and h.sum() == 1
    with pytest.raises(ValueError):
        load_hard(y, (5, 5))


def test_errors(tmp_path):
    (tmp_path / 'map.yaml').write_text('image: nope.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    with pytest.raises(ValueError):
        load_map(tmp_path / 'map.yaml')
    (tmp_path / 'map.yaml').write_text('image: map.pgm\n')
    with pytest.raises(ValueError):
        load_map(tmp_path / 'map.yaml')


def test_frame_change_detection_and_reset():
    """map -> odom moved after the user's initial pose: the evidence written at the old place must be dropped (2026-11-09 nav run)."""
    import math
    import numpy as np
    from go2_per.evidence_logic import EvidenceGrid, DEFAULT_CFG, frame_changed
    assert frame_changed((0, 0, 0), (0, 0, 0))[0] is False
    assert frame_changed((0, 0, 0), (0.03, 0.0, 0.01))[0] is False            # jitter
    ch, shift, turn = frame_changed((0, 0, 0), (1.433, 3.897, math.radians(150.6)))
    assert ch and shift > 4.0 and turn > 2.0
    assert frame_changed((0, 0, math.pi - 0.01), (0, 0, -math.pi + 0.01))[0] is False      # wrap-around is not a turn
    g = EvidenceGrid(100, 100, 0.05, (0.0, 0.0), dict(DEFAULT_CFG))
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    g.update((2.5, 2.5, 0.0), angles, np.full(72, 1.0), np.full(72, 0.8))       # a ring of walls at 1 m
    assert (g.evidence() >= 99).sum() > 0
    n = g.reset()
    assert n > 0 and (g.evidence() == -1).all() and g.changed
