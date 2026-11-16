"""Sensor-format adaptation only, independent of ROS and ground-truth poses."""
import numpy as np
from go2_per.per_logic import transform, outside_body


def snapshot_points(pts, rotation, translation, geometry):
    """Keep original lidar coordinates; add zero intensity/time for instantaneous rays."""
    body=transform(pts,rotation,translation)
    keep=outside_body(body,geometry.body_half_length_m,geometry.body_half_width_m,.02)
    pts=pts[keep & np.isfinite(pts).all(axis=1)]
    out=np.zeros((len(pts),5),dtype='<f4')
    out[:,:3]=pts
    return out
