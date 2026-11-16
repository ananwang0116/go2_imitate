from types import SimpleNamespace
import numpy as np
from go2_sim.slam_logic import snapshot_points


def test_snapshot_adapter_preserves_lidar_frame_and_does_not_fabricate_scan_time():
    g=SimpleNamespace(body_half_length_m=.35,body_half_width_m=.20)
    rotation=np.diag([-1.,-1.,1.])
    pts=np.array([[.1,0.,0.],[1.,2.,.3],[np.nan,0.,0.]])
    out=snapshot_points(pts,rotation,[0.,0.,.1],g)
    assert out.shape==(1,5)
    np.testing.assert_allclose(out[0,:3],[1.,2.,.3])
    assert (out[:,3:]==0.).all()
    assert out.dtype==np.dtype('<f4')
