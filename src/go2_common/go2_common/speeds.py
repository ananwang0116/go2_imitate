"""The single speed file config/speeds.yaml (like geometry.yaml for distances)."""
import os
from dataclasses import dataclass, fields
from pathlib import Path

import yaml

SPEEDS_ENV = 'GO2_SPEEDS_FILE'   # set by go2.launch.py; dev_teleop (started by hand) falls back to GO2_WS


@dataclass(frozen=True)
class Speeds:
    forward_mps: float
    back_mps: float
    side_mps: float
    turn_radps: float
    manual_side_mps: float


def default_path() -> str:
    env = os.environ.get(SPEEDS_ENV, '')
    if env:
        return env
    return str(Path(os.environ.get('GO2_WS', '.')) / 'src/go2_bringup/config/speeds.yaml')


def load(path=None) -> Speeds:
    data = yaml.safe_load(Path(path or default_path()).read_text())
    names = [f.name for f in fields(Speeds)]
    missing, extra = [n for n in names if n not in data], [k for k in data if k not in names]
    if missing or extra:
        raise ValueError(f'{path}: missing keys {missing}, unknown keys {extra}')
    s = Speeds(**{n: float(data[n]) for n in names})
    bad = [n for n in names if not 0 < getattr(s, n) <= 1.0]
    if bad:
        raise ValueError(f'{path}: {bad} must be in (0, 1.0]')
    return s
