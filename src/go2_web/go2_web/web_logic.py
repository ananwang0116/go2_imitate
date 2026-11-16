"""Web console logic without ROS (spec 6.6): map files, map edits, point clouds, mission files. Tested offline.

Maps: maps/<name>/map.yaml + map.pgm (+ cloud.pcd, the 3D map of the run that made it). Edits touch map.pgm and hard.pgm and
always keep a backup first (maps/<name>/backup/<time>.pgm, + <time>.hard); restore = put the newest backup back.
hard.pgm (same size, 0 = hard wall): the walls drawn by hand here (glass the laser never sees). Unlike the walls of the scan they
can never be erased by the laser in navigation (per_map publishes them as /per_hard_map, per_evidence keeps them at 100);
the eraser removes both kinds. Decision 2026-10-26: scanned map walls may have been things that left, drawn walls are meant.
Pixel <-> world: row 0 of the image is the TOP (largest y): col = (x - ox) / res, row = H - 1 - (y - oy) / res
(same as go2_per map_logic.load_map, which flips the image).
"""
import math
import collections
import re
import shutil
import struct
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

NAME_RE = re.compile(r'^[0-9A-Za-z_\-一-鿿]{1,40}$')
FREE, OCC = 254, 0
HARD_FILE, HARD, NO_HARD = 'hard.pgm', 0, 255


def check_name(name):
    """Map / mission names become folder / file names: no path characters, no dots."""
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ValueError('name: 1-40 letters, digits, _ - or Chinese characters')
    return name


# ---------------- maps ----------------
def list_maps(maps_dir):
    out = []
    for d in sorted(Path(maps_dir).iterdir()) if Path(maps_dir).is_dir() else []:
        if (d / 'map.yaml').is_file() and (d / 'map.pgm').is_file():
            out.append({'name': d.name, 'cloud': (d / 'cloud.pcd').is_file(),
                        'backups': len(list((d / 'backup').glob('*.pgm'))) if (d / 'backup').is_dir() else 0})
    return out


def load_meta(map_dir):
    meta = yaml.safe_load((Path(map_dir) / 'map.yaml').read_text())
    img = cv2.imread(str(Path(map_dir) / meta['image']), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f'cannot read the image of {map_dir}')
    h, w = img.shape
    return {'resolution': float(meta['resolution']), 'origin': [float(meta['origin'][0]), float(meta['origin'][1])],
            'width': w, 'height': h, 'negate': int(meta.get('negate', 0)),
            'occupied_thresh': float(meta.get('occupied_thresh', 0.65)), 'free_thresh': float(meta.get('free_thresh', 0.25)),
            'image': meta['image']}, img


def load_hard(map_dir, shape):
    """hard.pgm as a uint8 image (0 = hard wall), or an empty one (all NO_HARD) when there is none."""
    f = Path(map_dir) / HARD_FILE
    img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) if f.is_file() else None
    if img is None or img.shape != tuple(shape):
        return np.full(shape, NO_HARD, np.uint8)
    return img


def map_png(map_dir):
    """RGBA PNG for the page: occupied dark, free light, unknown transparent, hand-drawn hard walls teal."""
    meta, img = load_meta(map_dir)
    # by pixel value: tools/run_to_2d_map.py writes unknown as 205 (free_thresh 0.196 reads it as unknown); the page shows
    # unknown as transparent like the design: ink walls on paper.
    rgba = np.zeros(img.shape + (4,), np.uint8)
    rgba[img >= 250] = (241, 247, 249, 255)    # free, BGRA (OpenCV order): #f9f7f1, a touch lighter than the page paper #f3efe6
    rgba[img <= 128] = (48, 34, 28, 255)       # occupied #1c2230
    rgba[load_hard(map_dir, img.shape) < 128] = (109, 122, 15, 255)   # hard (drawn) wall #0f7a6d
    ok, buf = cv2.imencode('.png', rgba)
    if not ok:
        raise ValueError('png encode failed')
    return meta, buf.tobytes()


def world_to_px(meta, x, y):
    return (x - meta['origin'][0]) / meta['resolution'], meta['height'] - 1 - (y - meta['origin'][1]) / meta['resolution']


def apply_edits(img, meta, edits):
    """Edits on the grey image: wall = occupied line 0.1 m thick, erase = occupied cells in a circle become free."""
    return edit_images(img, None, meta, edits)[0]


def edit_images(img, hard, meta, edits):
    """-> (map image, hard image). A wall goes into both (the map shows it, the hard layer keeps it in navigation); erase
    clears both inside the circle. hard None = no hard layer is kept."""
    out = img.copy()
    hout = None if hard is None else hard.copy()
    res = meta['resolution']
    for e in edits:
        kind = e.get('type')
        if kind == 'wall':
            a = world_to_px(meta, float(e['x1']), float(e['y1']))
            b = world_to_px(meta, float(e['x2']), float(e['y2']))
            cv2.line(out, (round(a[0]), round(a[1])), (round(b[0]), round(b[1])), OCC, max(1, round(0.1 / res)))
            if hout is not None:
                cv2.line(hout, (round(a[0]), round(a[1])), (round(b[0]), round(b[1])), HARD, max(1, round(0.1 / res)))
        elif kind == 'erase':
            r = float(e['r'])
            if not 0 < r <= 5:
                raise ValueError('erase radius must be in (0, 5] m')
            c = world_to_px(meta, float(e['x']), float(e['y']))
            mask = np.zeros(out.shape, np.uint8)
            cv2.circle(mask, (round(c[0]), round(c[1])), max(1, round(r / res)), 1, -1)
            out[(mask == 1) & (out < 128)] = FREE
            if hout is not None:
                hout[mask == 1] = NO_HARD
        else:
            raise ValueError(f'unknown edit type {kind!r}')
    return out, hout


def save_edits(map_dir, edits):
    """Backup first, then write. Returns the backup path."""
    map_dir = Path(map_dir)
    meta, img = load_meta(map_dir)
    new, hard = edit_images(img, load_hard(map_dir, img.shape), meta, edits)
    bdir = map_dir / 'backup'
    bdir.mkdir(exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    n = 1
    while (bdir / (stamp + '.pgm')).exists():            # two saves in one second must not overwrite the first backup
        n += 1
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{n}'
    backup = bdir / (stamp + '.pgm')
    shutil.copy2(map_dir / meta['image'], backup)
    if (map_dir / HARD_FILE).is_file():                  # no <stamp>.hard = there was no hard layer before this save
        shutil.copy2(map_dir / HARD_FILE, bdir / (stamp + '.hard'))
    if not cv2.imwrite(str(map_dir / meta['image']), new):
        raise ValueError('cannot write the map image')
    if (hard < 128).any() or (map_dir / HARD_FILE).is_file():
        if not cv2.imwrite(str(map_dir / HARD_FILE), hard):
            raise ValueError('cannot write the hard wall layer')
    return backup


def restore_last(map_dir):
    map_dir = Path(map_dir)
    backups = sorted((map_dir / 'backup').glob('*.pgm')) if (map_dir / 'backup').is_dir() else []
    if not backups:
        raise ValueError('no backup to restore')
    meta, _ = load_meta(map_dir)
    shutil.copy2(backups[-1], map_dir / meta['image'])
    hard_backup = backups[-1].with_suffix('.hard')
    if hard_backup.is_file():
        shutil.copy2(hard_backup, map_dir / HARD_FILE)
        hard_backup.unlink()
    elif (map_dir / HARD_FILE).is_file():
        (map_dir / HARD_FILE).unlink()                   # that version had no hard layer
    backups[-1].unlink()
    return backups[-1].name


# ---------------- point clouds ----------------
def lzf_decompress(data, out_len):
    out = bytearray()
    i = 0
    while i < len(data):
        c = data[i]
        i += 1
        if c < 32:
            out += data[i:i + c + 1]
            i += c + 1
        else:
            ln = c >> 5
            if ln == 7:
                ln += data[i]
                i += 1
            ref = len(out) - ((c & 0x1f) << 8) - data[i] - 1
            i += 1
            for _ in range(ln + 2):
                out.append(out[ref])
                ref += 1
    if len(out) != out_len:
        raise ValueError('lzf: wrong output length')
    return bytes(out)


def read_pcd_xyz(path):
    """x y z of a .pcd (ascii, binary, binary_compressed) as float32 Nx3."""
    raw = Path(path).read_bytes()
    head_end = raw.index(b'DATA ')
    head_end = raw.index(b'\n', head_end) + 1
    head = raw[:head_end].decode()
    fields = head.split('FIELDS')[1].split('\n')[0].split()
    sizes = [int(s) for s in head.split('SIZE')[1].split('\n')[0].split()]
    types = head.split('TYPE')[1].split('\n')[0].split()
    n = int(re.search(r'POINTS (\d+)', head).group(1))
    kind = re.search(r'DATA (\w+)', head).group(1)
    for f in ('x', 'y', 'z'):
        if f not in fields:
            raise ValueError(f'{path}: no field {f}')
    dtype = np.dtype([(f, {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('I', 1): 'i1',
                           ('I', 2): '<i2', ('I', 4): '<i4'}[(t, s)]) for f, s, t in zip(fields, sizes, types)])
    if kind == 'ascii':
        arr = np.loadtxt(path, skiprows=head.count('\n'), ndmin=2)
        xyz = arr[:, [fields.index('x'), fields.index('y'), fields.index('z')]]
    else:
        if kind == 'binary':
            body = raw[head_end:head_end + n * dtype.itemsize]
            rec = np.frombuffer(body, dtype=dtype, count=n)
        elif kind == 'binary_compressed':
            comp, uncomp = struct.unpack('<II', raw[head_end:head_end + 8])
            body = lzf_decompress(raw[head_end + 8:head_end + 8 + comp], uncomp)
            rec, off = {}, 0                                # column-major: all x, then all y, ...
            for f, s, t in zip(fields, sizes, types):
                rec[f] = np.frombuffer(body, dtype=dtype[f], count=n, offset=off)
                off += n * s
        else:
            raise ValueError(f'{path}: unknown DATA {kind}')
        xyz = np.stack([rec['x'], rec['y'], rec['z']], axis=1)
    xyz = xyz.astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_keys(pts, voxel):
    q = np.floor(pts / voxel).astype(np.int64)
    return (q[:, 0] + (1 << 20)) | ((q[:, 1] + (1 << 20)) << 21) | ((q[:, 2] + (1 << 20)) << 42)


def voxel_downsample(pts, voxel):
    if len(pts) == 0:
        return pts
    _, idx = np.unique(voxel_keys(pts, voxel), return_index=True)
    return pts[np.sort(idx)]


def thin(pts, max_points):
    """At most max_points, evenly spread (not the first ones)."""
    if len(pts) <= max_points:
        return pts
    return pts[np.linspace(0, len(pts) - 1, max_points).astype(np.int64)]


class CloudAccumulator:
    """The live lidar cloud in the map (or odom) frame, one point per voxel, oldest dropped when over `cap`.
    Incremental (2026-10-27): only the NEW voxels of a batch are looked up (sorted keys + searchsorted) and appended, instead of
    sorting the whole cloud on every batch; voxel 0.05 m / cap 400 000 (was 0.1 m / 250 000, which made walls a sparse dotted
    line: the Go2's lidar gives only ~1.4-2 k points per frame, so what the page shows is limited by how few of them we throw away).
    Size (measured 2026-10-27): 400 000 points = 11 MB held here (points 4.8 + two key arrays 3.2 each), 22 MB peak while filling,
    one /api/cloud body 4.8 MB (12 bytes a point); the page keeps one copy (4.8 MB) plus the same on the GPU."""

    def __init__(self, voxel=0.05, cap=400_000, z_range=(-1.0, 4.0)):
        self.voxel, self.cap, self.z_range = voxel, cap, z_range
        self.pts = np.zeros((0, 3), np.float32)
        self.keys = np.zeros(0, np.int64)            # voxel key of every point of self.pts, same order
        self.sorted_keys = np.zeros(0, np.int64)     # the same keys sorted, for the membership test
        self.rev = 0

    def add(self, pts):
        pts = pts[(pts[:, 2] > self.z_range[0]) & (pts[:, 2] < self.z_range[1])]
        if len(pts) == 0:
            return
        k = voxel_keys(pts, self.voxel)
        k, first = np.unique(k, return_index=True)               # one point per voxel inside the batch
        pts = pts[first].astype(np.float32)
        if len(self.sorted_keys):
            at = np.minimum(np.searchsorted(self.sorted_keys, k), len(self.sorted_keys) - 1)
            new = self.sorted_keys[at] != k                      # the older point of a voxel wins
            k, pts = k[new], pts[new]
        if len(k) == 0:
            return
        self.pts = np.concatenate([self.pts, pts])
        self.keys = np.concatenate([self.keys, k])
        if len(self.pts) > self.cap:
            self.pts, self.keys = self.pts[-self.cap:], self.keys[-self.cap:]
            self.sorted_keys = np.sort(self.keys)
        else:
            self.sorted_keys = np.union1d(self.sorted_keys, k)
        self.rev += 1

    def clear(self):
        self.pts = np.zeros((0, 3), np.float32)
        self.keys = self.sorted_keys = np.zeros(0, np.int64)
        self.rev += 1

    def expire(self, now=None):
        return 0                         # mapping keeps the complete accumulated cloud


class RecentCloudAccumulator(CloudAccumulator):
    """Navigation DISPLAY only: unique voxels from the last ttl_s seconds, including refreshed observations.

    At 2 batches/s x ~18k points x 5 s, retained batches are ~2.2 MB float32; bounded to cap (4.8 MB).
    Rebuild only on a batch arrival/expiry, never on every HTTP request. Mapping uses CloudAccumulator instead.
    """
    def __init__(self, ttl_s=5.0, **kwargs):
        super().__init__(**kwargs)
        self.ttl_s = ttl_s
        self.batches = collections.deque()

    def _rebuild(self):
        self.pts = (voxel_downsample(np.vstack([p for _, p in self.batches]), self.voxel)[-self.cap:]
                    if self.batches else np.zeros((0, 3), np.float32))
        self.rev += 1

    def add(self, pts, now=None):
        now = time.monotonic() if now is None else now
        pts = pts[(pts[:, 2] > self.z_range[0]) & (pts[:, 2] < self.z_range[1])].astype(np.float32)
        while self.batches and now - self.batches[0][0] >= self.ttl_s:
            self.batches.popleft()
        if len(pts):
            self.batches.append((now, voxel_downsample(pts, self.voxel)))
        # Bound retained INPUT as well as output. A dense view must not accumulate unlimited duplicate batches.
        n = sum(len(p) for _, p in self.batches)
        while n > self.cap and self.batches:
            t, p = self.batches.popleft()
            keep = len(p) - (n - self.cap)
            n -= len(p)
            if keep > 0:
                self.batches.appendleft((t, p[-keep:]))
                n += keep
        self._rebuild()

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        n = 0
        while self.batches and now - self.batches[0][0] >= self.ttl_s:
            self.batches.popleft()
            n += 1
        if n:
            self._rebuild()
        return n

    def clear(self):
        self.batches.clear()
        super().clear()


def cloud_pose_usable(loc_state, loc_age_s):
    """No cloud should be accumulated using the last frozen pose after localisation has stopped."""
    return loc_state in (0, 1) and loc_age_s <= 1.0     # LocStatus OK / DEGRADED; LOST=2


def cloud_bytes(pts):
    """Binary body for the page: float32 x y z, little endian."""
    return np.ascontiguousarray(pts, dtype='<f4').tobytes()


def transform_points(pts, t, q):
    """Rotate by quaternion q=(x,y,z,w), then translate by t."""
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return (pts.astype(np.float64) @ r.T + np.asarray(t)).astype(np.float32)


def parse_cloud2(fields, point_step, data, n_points):
    """x y z float32 of a PointCloud2 (fields: [(name, offset, datatype)], datatype 7 = FLOAT32)."""
    off = {name: o for name, o, dt in fields if dt == 7}
    if not all(k in off for k in 'xyz'):
        return np.zeros((0, 3), np.float32)
    buf = np.frombuffer(data, dtype=np.uint8, count=n_points * point_step).reshape(n_points, point_step)
    cols = [np.ascontiguousarray(buf[:, off[k]:off[k] + 4]).view('<f4')[:, 0] for k in 'xyz']
    xyz = np.stack(cols, axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


# ---------------- missions ----------------
def mission_rows(wps):
    """Page waypoints -> rows for task_logic.save_mission; validates numbers."""
    rows = []
    if not isinstance(wps, list) or not wps:
        raise ValueError('need at least one waypoint')
    for i, w in enumerate(wps):
        x, y = float(w['x']), float(w['y'])
        yaw = w.get('yaw')                                # None / missing = no final turn
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'waypoint {i + 1}: x/y not finite')
        row = {'name': str(w.get('name') or f'wp{i + 1}')[:24], 'x': round(x, 3), 'y': round(y, 3)}
        if yaw is not None:
            row['yaw'] = round(float(yaw), 3)
        if w.get('radius_m') is not None:
            row['radius_m'] = round(float(w['radius_m']), 3)
        if w.get('path'):
            row['path'] = [[round(float(a), 3), round(float(b), 3)] for a, b in w['path']]
        rows.append(row)
    return rows


def list_missions(missions_dir):
    return sorted(p.stem for p in Path(missions_dir).glob('*.yaml')) if Path(missions_dir).is_dir() else []


# ---------------- teach routes ----------------
# A route = the path the user drove while mapping (frame of that run's odom = the new map's frame), plus parking marks.
# maps/<map>/routes/<name>.yaml: {name, map, length_m, path: [[x, y], ...], marks: [[x, y], ...]}
def path_length(path):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])))


def route_rows(name, map_name, path, marks, max_points=2000):
    if len(path) < 2:
        raise ValueError('the path is too short (drive first)')
    step = max(1, math.ceil(len(path) / max_points))
    thinned = [[round(float(x), 3), round(float(y), 3)] for x, y in path[::step]]
    last = [round(float(path[-1][0]), 3), round(float(path[-1][1]), 3)]
    path = thinned if thinned[-1] == last else thinned + [last]                # the end point is always there, never twice
    return {'name': check_name(name), 'map': map_name, 'length_m': round(path_length(path), 2), 'path': path,
            'marks': [[round(float(x), 3), round(float(y), 3)] for x, y in marks]}


def route_segments(path, marks, from_xy=None):
    """The taught route as waypoints WITH their taught path (mot_nav follows each segment as it is, no A*):
    one waypoint per parking mark (P1..) and one for the end; each carries the path from the previous stop to it.
    from_xy: the dog is somewhere on / near the route: start at the nearest path point and drop the marks behind it."""
    if len(path) < 2:
        raise ValueError('the path is too short')
    pts = np.asarray(path, dtype=float)
    start = 0 if from_xy is None else int(np.argmin(np.hypot(pts[:, 0] - from_xy[0], pts[:, 1] - from_xy[1])))
    stops = {}
    for n, m in enumerate(marks):
        k = int(np.argmin(np.hypot(pts[:, 0] - m[0], pts[:, 1] - m[1])))
        if start < k < len(pts) - 1:
            stops[k] = f'P{n + 1}'
    stops[len(pts) - 1] = 'end'
    out, prev = [], start
    for k in sorted(stops):
        seg = pts[prev:k + 1]
        if len(seg) >= 2:
            out.append({'name': stops[k], 'x': round(float(pts[k][0]), 3), 'y': round(float(pts[k][1]), 3),
                        'path': [[round(float(x), 3), round(float(y), 3)] for x, y in seg]})
            prev = k
    if not out:
        raise ValueError('nothing left to follow from there')
    return out


def list_routes(maps_dir, map_name):
    d = Path(maps_dir) / check_name(map_name) / 'routes'
    out = []
    for f in sorted(d.glob('*.yaml')) if d.is_dir() else []:
        r = yaml.safe_load(f.read_text()) or {}
        out.append({'name': f.stem, 'length_m': r.get('length_m', 0), 'marks': len(r.get('marks', []))})
    return out


def read_route(maps_dir, map_name, name):
    return yaml.safe_load((Path(maps_dir) / check_name(map_name) / 'routes' / f'{check_name(name)}.yaml').read_text())


# ---------------- mapping health (2026-10-26) ----------------
IMU_GAP_SUSPECT_MS = 150      # a hole this long in the IMU can throw Leg-KILO off (2026-10-26 run: 175 ms + 1.1 s late)


def map_health_item(layer, name, fields, wall_s):
    """One event -> a mapping-health problem or None. Leg-KILO builds the map from the pose it estimates; once that pose has
    jumped or its IMU had a hole, everything mapped afterwards may be bent or doubled, and nothing on the page used to say so
    (the 2026-10-26 run went on for 3 more minutes). fields: the event's key -> value strings.
    Red 'bad': a pose jump (the dog cannot move like that), or the pose lost for good. Amber 'warn': an IMU hole."""
    def num(k):
        try:
            return float(fields.get(k, ''))
        except ValueError:
            return float('nan')
    t = time.strftime('%H:%M:%S', time.localtime(wall_s))
    if layer == 'loc' and name == 'JUMP':
        d = math.hypot(num('x_m') - num('from_x_m'), num('y_m') - num('from_y_m'))
        far = f'{d:.1f} m' if math.isfinite(d) else '很远'
        return {'level': 'bad', 'kind': 'jump', 't': wall_s, 'text': f'{t} 位姿突然跳了 {far}（狗没有这么快）'}
    if layer == 'loc' and name == 'STATE' and fields.get('to') == 'LOST' and fields.get('reason') in ('kilo_stale', 'pose_jump'):
        return {'level': 'bad', 'kind': 'lost', 't': wall_s, 'text': f'{t} 定位丢失（{fields.get("reason")}）'}
    if layer == 'dev' and name == 'IMU_GAP' and num('longest_ms') >= IMU_GAP_SUSPECT_MS:
        return {'level': 'warn', 'kind': 'imu_gap', 't': wall_s, 'text': f'{t} IMU 断了 {num("longest_ms"):.0f} ms'}
    if layer == 'dev' and name == 'STALL':
        return {'level': 'warn', 'kind': 'stall', 't': wall_s,
                'text': f'{t} dev_bridge 卡了 {num("longest_ms"):.0f} ms（{fields.get("slowest_cb", "?")}）'}
    return None


def map_health(items):
    """-> {'bad': bool, 'n': int, 'first': item or None, 'items': last few}. bad = any red item; 'first' = the first RED item
    when there is one (that is the moment the map went wrong), else the first item."""
    bad = [i for i in items if i['level'] == 'bad']
    return {'bad': bool(bad), 'n': len(items), 'first': (bad or items or [None])[0], 'items': items[-4:]}


def evidence_png(data, width, height):
    """Confirmed obstacle grid -> BGRA PNG, top row at maximum world y: static obstacles (100) cyan, dynamic ones (99) yellow.

    Unknown/free cells stay transparent so the supplied map remains visible.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    image = np.zeros((height, width, 4), dtype=np.uint8)
    # Display only: a one-cell halo leaves cyan visible around red live-scan dots.
    # Navigation continues to consume the original evidence cells, without this halo.
    flipped = np.flipud(grid)
    static = cv2.dilate(((flipped >= 65) & (flipped != 99)).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    dynamic = cv2.dilate((flipped == 99).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    image[static] = (212, 188, 0, 220)    # RGB #00bcd4
    image[dynamic] = (0, 196, 255, 230)   # RGB #ffc400, drawn over the static halo
    ok, encoded = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('could not encode obstacle evidence')
    return encoded.tobytes()


class AlignmentFeedback:
    """UI acknowledgement of loc_align decisions, never a localisation judge.

    A result is shown only after both the matching decision event and a newly
    received map->odom transform. All deadlines use monotonic time.
    """
    def __init__(self):
        self.sequence = 0
        self.reset()

    def reset(self):
        self.state = 'unaligned'
        self.request_ns = 0
        self.started = 0.
        self.tf_version = -1
        self.decision = None
        self.fields = {}
        self.reason = ''

    def request(self, ros_ns, tf_version, now):
        self.sequence += 1
        self.state = 'pending'
        self.request_ns, self.started, self.tf_version = ros_ns, now, tf_version
        self.decision, self.fields, self.reason = None, {}, ''
        return self.sequence

    def event(self, name, fields, ros_ns):
        if name not in ('ALIGN_OK','ALIGN_WEAK','ALIGN_FAIL'):
            return
        if self.state == 'pending' and 'request_stamp_ns' in fields and str(fields['request_stamp_ns']) != str(self.request_ns):
            return
        if self.state == 'pending' and ros_ns < self.request_ns:
            return  # an old retained/startup event is not this request's acknowledgement
        state = {'ALIGN_OK':'ok','ALIGN_WEAK':'weak','ALIGN_FAIL':'failed'}[name]
        self.fields = dict(fields)
        self.reason = fields.get('reason','')
        if self.state == 'pending' and state != 'failed':
            self.decision = state
        else:
            self.state = state

    def advance(self, tf_version, now):
        if self.state == 'pending':
            if self.decision and tf_version > self.tf_version:
                self.state = self.decision
            elif now-self.started >= 8.:
                self.state, self.reason = 'failed', 'alignment_feedback_timeout'
        return self.state

    def status(self):
        return {'request_id':self.sequence,'requested':self.started>0.,'state':self.state,'reason':self.reason,'fields':dict(self.fields)}
