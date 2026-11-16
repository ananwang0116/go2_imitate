"""The single geometry file config/geometry.yaml (spec 3): loading and the nesting check.

Distances must nest: waypoint clearance > avoid gap > safe stop gap, each including the localisation error.
check() returns every violated relation; the launch refuses to start when the list is not empty.
"""
import math
from dataclasses import dataclass, fields
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Geometry:
    body_half_length_m: float
    body_half_width_m: float
    loc_sigma_m: float
    brake_decel_mps2: float
    brake_reaction_s: float
    safe_stop_gap_m: float
    avoid_gap_m: float
    waypoint_clearance_m: float
    waypoint_margin_m: float
    arrive_radius_m: float
    mid360_x_m: float
    mid360_y_m: float
    mid360_z_m: float
    mid360_roll_deg: float
    mid360_pitch_deg: float
    mid360_yaw_deg: float

    def brake_distance_m(self, speed_mps: float) -> float:
        """Distance to stop from speed_mps: reaction travel + v^2 / 2a."""
        v = abs(speed_mps)
        return v * self.brake_reaction_s + v * v / (2.0 * self.brake_decel_mps2)


def mid360_rotation(g: Geometry) -> list:
    """R_base_lidar as a row-major 3x3 (list of 9): R = Rz(yaw) Ry(pitch) Rx(roll). Column k = lidar axis k in `base`.
    The same numbers go to the static TF base -> mid360_link and to Leg-KILO's extrinsic_R (lidar -> body)."""
    return rotation_rpy_deg(g.mid360_roll_deg, g.mid360_pitch_deg, g.mid360_yaw_deg)


def rotation_rpy_deg(roll, pitch, yaw) -> list:
    """Rz(yaw) Ry(pitch) Rx(roll), row-major. Also used to interpret an external adapter's actual axis rotation."""
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr]


def mid360_translation(g: Geometry) -> list:
    return [g.mid360_x_m, g.mid360_y_m, g.mid360_z_m]


def load(path) -> Geometry:
    data = yaml.safe_load(Path(path).read_text())
    names = [f.name for f in fields(Geometry)]
    missing = [n for n in names if n not in data]
    extra = [k for k in data if k not in names]
    if missing or extra:
        raise ValueError(f'{path}: missing keys {missing}, unknown keys {extra}')
    return Geometry(**{n: float(data[n]) for n in names})


def check(g: Geometry) -> list:
    """Every violated relation as a readable line; empty = consistent."""
    eps = 1e-9
    rules = [
        ('body_half_length_m > 0 and body_half_width_m > 0', g.body_half_length_m > 0 and g.body_half_width_m > 0),
        ('loc_sigma_m >= 0', g.loc_sigma_m >= 0),
        ('brake_decel_mps2 > 0', g.brake_decel_mps2 > 0),
        ('brake_reaction_s >= 0', g.brake_reaction_s >= 0),
        ('safe_stop_gap_m > 0', g.safe_stop_gap_m > 0),
        (f'avoid_gap_m ({g.avoid_gap_m}) >= safe_stop_gap_m + loc_sigma_m ({g.safe_stop_gap_m + g.loc_sigma_m:.3f})',
         g.avoid_gap_m + eps >= g.safe_stop_gap_m + g.loc_sigma_m),
        (f'waypoint_clearance_m ({g.waypoint_clearance_m}) >= avoid_gap_m + waypoint_margin_m '
         f'({g.avoid_gap_m + g.waypoint_margin_m:.3f})',
         g.waypoint_clearance_m + eps >= g.avoid_gap_m + g.waypoint_margin_m),
        ('waypoint_margin_m > 0', g.waypoint_margin_m > 0),
        (f'arrive_radius_m ({g.arrive_radius_m}) >= loc_sigma_m ({g.loc_sigma_m})', g.arrive_radius_m + eps >= g.loc_sigma_m),
    ]
    return [text for text, ok in rules if not ok]
