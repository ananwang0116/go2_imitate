#!/usr/bin/env python3
"""task_preflight: check a mission before it runs (spec 6.2). The check itself is mot_nav's (service /mot_nav_check: the SAME
costmap and planner that drive the dog), so this tool needs the nav stack running (`mode:=nav`), or the web console's
"检查航点" (same service).

  ros2 run go2_task task_preflight <mission name | path.yaml> [--force] [--no-start]
For every waypoint: radius >= loc_sigma_m; inside the map; area not mostly unknown; a passable position in the area;
connected to the dog (or to the previous waypoint with --no-start); a taught path keeps its clearance.
Exit 0 = all pass, 1 = some fail (the task must not start without --force), 2 = bad input / mot_nav not answering.
`--format-only`: only the file format and the radius vs loc_sigma_m are checked (no stack needed).
"""
import argparse
import os
import sys
import time
from pathlib import Path

from go2_common import geometry, names
from go2_common.node import GEOMETRY_ENV
from go2_task.task_logic import load_mission


def resolve(name_or_path: str, folder: Path) -> Path:
    p = Path(name_or_path)
    return p if p.suffix == '.yaml' and p.is_file() else folder / f'{name_or_path}.yaml'


def format_problems(waypoints, g) -> list:
    return [(w.name, f'radius_m {w.radius_m} < loc_sigma_m {g.loc_sigma_m}') for w in waypoints if w.radius_m + 1e-9 < g.loc_sigma_m]


def request_from(waypoints, start=None):
    """go2_msgs/NavCheck request for the waypoints (paths flattened with path_off)."""
    from go2_msgs.srv import NavCheck
    req = NavCheck.Request()
    req.x = [float(w.x) for w in waypoints]
    req.y = [float(w.y) for w in waypoints]
    req.radius_m = [float(w.radius_m) for w in waypoints]
    off, px, py = [0], [], []
    for w in waypoints:
        for p in (w.path or []):
            px.append(float(p[0]))
            py.append(float(p[1]))
        off.append(len(px))
    req.path_off = off if any(w.path for w in waypoints) else []
    req.path_x, req.path_y = px, py
    if start is not None:
        req.use_start, req.start_x, req.start_y = True, float(start[0]), float(start[1])
    return req


def ask_mot_nav(waypoints, start=None, timeout_s=8.0):
    """-> list of problem texts (one per waypoint, '' = ok) from mot_nav, or None when it does not answer."""
    import rclpy
    from go2_msgs.srv import NavCheck
    rclpy.init()
    node = rclpy.create_node('task_preflight')
    try:
        client = node.create_client(NavCheck, names.MOT_NAV_CHECK)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        fut = client.call_async(request_from(waypoints, start))
        end = time.monotonic() + timeout_s
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        res = fut.result() if fut.done() else None
        if res is None or not res.available:
            return None
        return list(res.reason)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description='check a mission file before running it')
    ap.add_argument('mission')
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--format-only', action='store_true')
    ap.add_argument('--no-start', action='store_true', help='do not check the way from the dog to the first waypoint')
    ap.add_argument('--geometry', default=os.environ.get(GEOMETRY_ENV, ''))
    ws = Path(os.environ.get('GO2_WS', '.'))
    ap.add_argument('--missions-dir', default=os.environ.get('GO2_MISSIONS_DIR', str(ws / 'missions')))
    a = ap.parse_args(argv)
    if not a.geometry:
        a.geometry = str(ws / 'src/go2_bringup/config/geometry.yaml')
    if not Path(a.geometry).is_file():
        print(f'task_preflight: no geometry file (--geometry or env {GEOMETRY_ENV})')
        return 2
    g = geometry.load(a.geometry)
    path = resolve(a.mission, Path(a.missions_dir))
    try:
        waypoints = load_mission(path, g.arrive_radius_m)
    except (OSError, ValueError, KeyError) as e:
        print(f'task_preflight: {e}')
        return 2
    if a.format_only:
        problems = format_problems(waypoints, g)
    else:
        reasons = ask_mot_nav(waypoints)
        if reasons is None:
            print('task_preflight: mot_nav does not answer on', names.MOT_NAV_CHECK, '(is mode:=nav running? use --format-only offline)')
            return 2
        problems = [(w.name, r) for w, r in zip(waypoints, reasons) if r]
    for name, text in problems:
        print(f'FAIL {name}: {text}')
    print(f'{path}: {len(waypoints)} waypoints, {len(problems)} problems' + (' (format and radius only)' if a.format_only else ''))
    return 1 if problems and not a.force else 0


if __name__ == '__main__':
    sys.exit(main())
