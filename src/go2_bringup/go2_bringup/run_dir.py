"""One directory per launch (spec 8.1): runs/<YYYYmmdd_HHMMSS>_<mode>/

  run.yaml      launch arguments, source fingerprint (which code ran), geometry snapshot, map name
  system.txt    the machine at start: cores, governor / frequency, temperatures, load, kernel (for CPU-starvation questions)
  launch.log    symlink to the launch's own log (all node output)
  events.log    every layer's events
  bag/          rosbag (spec 9)
  kilo/         Leg-KILO: run config, global map saved on exit, runtime/ copy of its temp results, its glog files
  ros_log/      ROS node log files (ROS_LOG_DIR)
"""
import os
import re
import hashlib
import shutil
import time
from datetime import datetime
from pathlib import Path

import yaml

from go2_common import geometry


def source_fingerprint(ws: Path) -> dict:
    """Which code ran (the project has no version control, user decision 2026-10-25): a hash over every
    file of our own packages (src/go2_*, without __pycache__) plus the newest-modified file and its time."""
    h, newest, count = hashlib.sha256(), (0.0, ''), 0
    for f in sorted((ws / 'src').glob('go2_*/**/*')):
        if not f.is_file() or '__pycache__' in f.parts:
            continue
        rel = str(f.relative_to(ws))
        h.update(rel.encode() + b'\0' + f.read_bytes() + b'\0')
        count += 1
        newest = max(newest, (f.stat().st_mtime, rel))
    return {'sha256': h.hexdigest()[:16], 'files': count, 'newest_file': newest[1],
            'newest_time': datetime.fromtimestamp(newest[0]).isoformat(timespec='seconds') if count else ''}


def create(ws: Path, mode: str, args: dict, geometry_file: Path, map_name: str) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run = ws / 'runs' / f'{stamp}_{mode}'
    n = 1
    while run.exists():                       # two launches in the same second
        n += 1
        run = ws / 'runs' / f'{stamp}_{mode}_{n}'
    for sub in ('bag', 'kilo', 'ros_log'):
        (run / sub).mkdir(parents=True, exist_ok=True)
    shutil.copy2(geometry_file, run / 'geometry.yaml')
    info = {'mode': mode, 'started': datetime.now().isoformat(timespec='seconds'), 'args': args,
            'source': source_fingerprint(ws), 'geometry_file': str(geometry_file), 'map': map_name,
            'geometry': yaml.safe_load(Path(geometry_file).read_text())}
    metadata=run/'run.yaml.tmp'
    metadata.write_text(yaml.safe_dump(info, sort_keys=False, allow_unicode=True))
    metadata.replace(run/'run.yaml')
    (run / 'events.log').touch()
    try:
        (run / 'system.txt').write_text(system_snapshot())
    except Exception as e:                    # a diagnostic file must never stop a launch
        (run / 'system.txt').write_text(f'failed: {type(e).__name__}: {e}\n')
    return run


def _first_line(path) -> str:
    try:
        return Path(path).read_text().strip().splitlines()[0]
    except (OSError, IndexError):
        return '?'


def system_snapshot() -> str:
    """Text for runs/<run>/system.txt (2026-11-04: IMU samples were being lost on the board; was a core slow or throttled?).
    Never writes credentials: of the DDS environment only the variable names that are set, and the two plain ones."""
    cpu = Path('/sys/devices/system/cpu')
    lines = [f'time: {datetime.now().isoformat(timespec="seconds")}', f'kernel: {" ".join(os.uname())}',
             f'cpus_online: {os.cpu_count()}  affinity: {sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else "?"}',
             f'loadavg: {_first_line("/proc/loadavg")}']
    for c in sorted(cpu.glob('cpu[0-9]*'), key=lambda p: int(p.name[3:])):
        f = c / 'cpufreq'
        lines.append(f'{c.name}: governor={_first_line(f / "scaling_governor")} cur_khz={_first_line(f / "scaling_cur_freq")} '
                     f'max_khz={_first_line(f / "scaling_max_freq")}')
    for z in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
        t = _first_line(z / 'temp')
        lines.append(f'{z.name}: {_first_line(z / "type")} {int(t) / 1000.0:.1f} C' if t.lstrip('-').isdigit() else f'{z.name}: ?')
    lines.append(f'net.core.rmem_max={_first_line("/proc/sys/net/core/rmem_max")} rmem_default={_first_line("/proc/sys/net/core/rmem_default")}')
    lines.append(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "")} RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "")}')
    lines.append('dds env set: ' + ','.join(k for k in ('GO2_DDS_CONFIG', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE') if os.environ.get(k)))
    return '\n'.join(lines) + '\n'


def link_launch_log(run: Path) -> None:
    """launch writes its own complete log (every process's output) to launch_config.log_dir; point to it."""
    try:
        from launch.logging import launch_config
        target = Path(launch_config.log_dir) / 'launch.log'
        (run / 'launch.log').symlink_to(target)
    except Exception as e:                     # never block a start-up on a convenience link
        (run / 'launch.log.missing').write_text(f'could not link the launch log: {e}\n')


def acquire_lock(lock: Path) -> None:
    lock.parent.mkdir(parents=True, exist_ok=True)
    lock.write_text(f'{os.getpid()} {datetime.now().isoformat(timespec="seconds")}\n')


def release_lock(lock: Path) -> None:
    try:
        if lock.read_text().split()[0] == str(os.getpid()):
            lock.unlink()
    except (OSError, IndexError):
        pass


def legkilo_source_dir(legkilo_share: Path) -> Path:
    """Leg-KILO writes result/ and logs/ under its compiled-in ROOT_DIR = its source folder.
    With --symlink-install the FILES in share/legkilo/config are links into the source (the folder is not); follow one."""
    return (legkilo_share / 'config' / 'go2_mid360.yaml').resolve().parent.parent


def _fmt(values) -> str:
    return '[' + ', '.join(f'{v:.6f}'.rstrip('0').rstrip('.') if v != 0 else '0' for v in values) + ']'


def write_kilo_config(run: Path, legkilo_share: Path, geometry_file: Path) -> Path:
    """Per-run copy of go2_mid360.yaml (the source file stays unchanged) with
    auto_save_root = <run>/kilo and extrinsic_T / extrinsic_R from geometry.yaml (the Mid360 mount: lidar -> body `base`,
    the same numbers the static TF base -> mid360_link uses)."""
    text = (legkilo_share / 'config' / 'go2_mid360.yaml').read_text()
    g = geometry.load(geometry_file)
    for key, value, note in (
            ('auto_save_root', str(run / 'kilo'), '            ## set by go2_bringup for this run'),
            ('extrinsic_T', _fmt(geometry.mid360_translation(g)), '   ## set by go2_bringup from geometry.yaml (mid360_x/y/z_m)'),
            ('extrinsic_R', _fmt(geometry.mid360_rotation(g)), '   ## set by go2_bringup from geometry.yaml (mid360 roll/pitch/yaw), row-major')):
        text, n = re.subn(rf'^{key}:.*$', lambda _m: f'{key}: {value}{note}', text, flags=re.M)
        if n != 1:
            raise RuntimeError(f'go2_mid360.yaml: expected exactly one {key} line')
    out = run / 'kilo' / 'go2_mid360.yaml'
    out.write_text(text)
    return out


def write_lio_config(run: Path, template: Path, geometry_file: Path) -> Path:
    """Per-run LIO-SAM parameters (<run>/lio/lio_sam.yaml): the template with extrinsicTrans / Rot / RPY computed from geometry.yaml
    (the Mid360 mount, the same numbers as the static TF and Leg-KILO's extrinsic; see go2_loc.lio_core.lio_extrinsics)."""
    import yaml
    from go2_loc import lio_core
    g = geometry.load(geometry_file)
    trans, rot, rpy = lio_core.lio_extrinsics(geometry.mid360_rotation(g), geometry.mid360_translation(g))
    data = yaml.safe_load(template.read_text())
    params = data['/**']['ros__parameters']
    params['extrinsicTrans'], params['extrinsicRot'], params['extrinsicRPY'] = ([float(v) for v in x] for x in (trans, rot, rpy))
    out = run / 'lio' / 'lio_sam.yaml'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text('# generated by go2_bringup from config/lio_sam.yaml + geometry.yaml; do not edit\n' + yaml.safe_dump(data, sort_keys=False))
    # Explicit direction/closure evidence, independent of frame names; no credentials.
    import numpy as np
    R = np.asarray(rpy).reshape(3, 3)
    t = np.asarray(geometry.mid360_translation(g))
    (out.parent / 'extrinsics_check.yaml').write_text(yaml.safe_dump({
        'imu_topic': params['imuTopic'], 'imu_reference': 'base (physical IMU offset not compensated)',
        'T_base_lidar_translation_m': t.tolist(), 'T_lidar_imu_translation_m': [float(v) for v in trans],
        'consumer': 'world_imu = world_lidar * Pose3(I, extrinsicTrans)',
        'closure_error_m': float(np.linalg.norm(t + R @ np.asarray(trans))),
        'effective_parameters': params,
    }, sort_keys=False))
    return out


def lio_save_complete(run: Path) -> bool:
    import json
    try:
        result = json.loads((run / 'lio' / 'save_result.json').read_text())
        return bool(result['success']) and (run / 'kilo/kilo_lio/global_map/global_map.pcd').is_file()
    except (OSError, ValueError, KeyError):
        return False


def snapshot_external_binaries(run: Path, paths: dict) -> None:
    """The project fingerprint does not cover the owner's executable files. Hash exactly the binaries we launch."""
    rows = {}
    for name, path in paths.items():
        p = Path(path)
        h = hashlib.sha256()
        with p.open('rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                h.update(chunk)
        rows[name] = {'path': str(p.resolve()), 'bytes': p.stat().st_size, 'sha256': h.hexdigest()}
    (run / 'lio' / 'external_binaries.yaml').write_text(yaml.safe_dump(rows, sort_keys=False))


def collect_kilo(run: Path, legkilo_share: Path, started_s: float, temp_folder='go2_run') -> str:
    """After legkilo_node exited: copy its temp results (wiped at the next start) and this run's glog files."""
    src = legkilo_source_dir(legkilo_share)
    notes = []
    temp = src / 'result' / temp_folder
    if temp.is_dir():
        shutil.copytree(temp, run / 'kilo' / 'runtime', dirs_exist_ok=True)
        notes.append('runtime copied')
    logs = src / 'logs'
    if logs.is_dir():
        dst = run / 'kilo' / 'glog'
        dst.mkdir(exist_ok=True)
        copied = 0
        for f in logs.iterdir():
            if f.is_file() and not f.is_symlink() and f.stat().st_mtime >= started_s - 5:
                shutil.copy2(f, dst / f.name)
                copied += 1
        notes.append(f'{copied} glog files')
    return ', '.join(notes) or 'nothing to copy'


def now_s() -> float:
    return time.time()
