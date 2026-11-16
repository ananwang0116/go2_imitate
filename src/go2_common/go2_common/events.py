"""Event text format and the per-run events.log (spec 8).

An event is one decision or state change: "[<layer>] <NAME> key=value key=value".
Periodic data never becomes an event; it goes to topics and the bag.
"""
import math
import os
import re
import time
from datetime import datetime
from pathlib import Path

from .names import LAYERS

RUN_DIR_ENV = 'GO2_RUN_DIR'          # set by go2.launch.py for every process it starts
_NAME_RE = re.compile(r'^[A-Z][A-Z0-9_]*$')
_KEY_RE = re.compile(r'^[a-z][a-z0-9_]*$')


def format_value(v) -> str:
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        if not math.isfinite(v):
            return str(v)
        text = f'{v:.3f}'.rstrip('0')
        return text + '0' if text.endswith('.') else text
    text = str(v)
    return f'"{text}"' if (' ' in text or text == '') else text


def check_event(layer: str, name: str, fields: dict) -> None:
    if layer not in LAYERS:
        raise ValueError(f'unknown layer {layer!r}')
    if not _NAME_RE.match(name):
        raise ValueError(f'event name must be UPPER_CASE, got {name!r}')
    bad = [k for k in fields if not _KEY_RE.match(k)]
    if bad:
        raise ValueError(f'event keys must be lower_case, got {bad}')


def format_event(layer: str, name: str, fields: dict) -> str:
    check_event(layer, name, fields)
    parts = [f'[{layer}] {name}'] + [f'{k}={format_value(v)}' for k, v in fields.items()]
    return ' '.join(parts)


def run_dir():
    """The current run directory, or None when the process was not started by go2.launch.py."""
    value = os.environ.get(RUN_DIR_ENV, '')
    return Path(value) if value else None


def append_event_line(text: str, ros_time_s: float = None, directory=None) -> None:
    """Append one event to <run>/events.log. One write() per line on an O_APPEND file: lines of different processes do not mix.
    directory: the run directory when the process was not started by the launch (the managed web console knows its stack's run)."""
    d = Path(directory) if directory else run_dir()
    if d is None:
        return
    wall = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    ros = f'{ros_time_s:.3f}' if ros_time_s is not None else f'{time.time():.3f}'
    line = f'{wall} t={ros} {text}\n'.encode()
    fd = os.open(d / 'events.log', os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.write(fd, line)
    finally:
        os.close(fd)
