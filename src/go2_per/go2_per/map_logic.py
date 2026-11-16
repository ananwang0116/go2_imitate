"""Saved 2D map (map.yaml + image) -> occupancy grid data, without ROS. The same rule as the map_server it replaces
(trinary, negate 0): p = (255 - pixel) / 255; p > occupied_thresh -> 100, p < free_thresh -> 0, else -1; image row 0 = top,
the grid's row 0 = bottom (y = origin_y). tools/run_to_2d_map.py writes unknown as 205 (p = 0.19608): with free_thresh 0.196
(what ROS map_saver writes, and the default here) it is UNKNOWN. Until 2026-10-26 the maps said free_thresh 0.25, which read
76 % of my_map_merged (everything never seen, also outside the building) as FREE (docs/02_联调记录.md 2026-10-26 第 12 次).
load_hard(): maps/<name>/hard.pgm, the walls drawn by hand in the web map editor (0 = hard), same orientation.
"""
from pathlib import Path

import numpy as np
import yaml


def load_map(yaml_path):
    """-> (data int8 [h, w] bottom-up, resolution, (origin_x, origin_y))."""
    import cv2
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    for k in ('image', 'resolution', 'origin'):
        if k not in meta:
            raise ValueError(f'{yaml_path}: missing key {k}')
    img = cv2.imread(str(yaml_path.parent / meta['image']), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f'{yaml_path}: cannot read image {meta["image"]}')
    p = (255 - img.astype(np.float64)) / 255.0
    if int(meta.get('negate', 0)):
        p = 1.0 - p
    data = np.full(img.shape, -1, dtype=np.int8)
    data[p > float(meta.get('occupied_thresh', 0.65))] = 100
    data[p < float(meta.get('free_thresh', 0.196))] = 0
    return np.flipud(data).copy(), float(meta['resolution']), (float(meta['origin'][0]), float(meta['origin'][1]))


def load_hard(yaml_path, shape):
    """-> bool [h, w] bottom-up: True = a wall drawn by hand (hard.pgm next to map.yaml); all False when there is none."""
    import cv2
    f = Path(yaml_path).parent / 'hard.pgm'
    img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) if f.is_file() else None
    if img is None:
        return np.zeros(shape, dtype=bool)
    if img.shape != tuple(shape):
        raise ValueError(f'{f}: size {img.shape} differs from the map {tuple(shape)}')
    return np.flipud(img < 128).copy()
