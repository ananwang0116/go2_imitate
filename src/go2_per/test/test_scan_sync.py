from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from builtin_interfaces.msg import Time as Stamp
from tf2_ros import TransformException
from go2_per.per_scan import PerScan


def test_scan_waits_for_own_tf_without_latest_fallback():
    obj=object.__new__(PerScan)
    old=NS(header=NS(stamp=Stamp(sec=9,nanosec=500_000_000),frame_id='mid360_link'))
    new=NS(header=NS(stamp=Stamp(sec=9,nanosec=900_000_000),frame_id='mid360_link'))
    obj.no_tf_reported=False;obj.event=Mock();obj.pending=deque([old,new]);obj.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10**10))
    obj.tf_buffer=Mock();obj.process_cloud=Mock()
    transform=object()
    obj.tf_buffer.lookup_transform.side_effect=[TransformException('LIO pending'),NS(transform=transform)]
    obj.drain()
    assert [c.args[2].nanoseconds for c in obj.tf_buffer.lookup_transform.call_args_list]==[9_900_000_000,9_500_000_000]
    obj.process_cloud.assert_called_once_with(old,transform)
    assert list(obj.pending)==[new]
    obj.tf_buffer.lookup_transform.side_effect=TransformException('pending')
    obj.process_cloud.reset_mock();obj.drain();obj.process_cloud.assert_not_called()
    obj.get_clock=lambda:NS(now=lambda:NS(nanoseconds=12*10**9));obj.drain()
    assert not obj.pending


def test_current_frame_does_not_inherit_old_body_frame_floor():
    obj=object.__new__(PerScan);obj.plane=np.array([.01,-.04,-.006])
    obj.geometry=object();obj.cfg=dict(obstacle_min_range_m=.15,map_min_range_m=.35)
    obj.slow_ms=1000;obj.obstacle_pub=Mock();obj.map_pub=Mock();obj.free_pub=Mock()
    obj.scan_msg=Mock(side_effect=lambda stamp,ranges,rmin:(stamp,ranges,rmin))
    msg=NS(fields=[NS(name=k,offset=i*4) for i,k in enumerate('xyz')],is_bigendian=False,
        data=b'',point_step=12,width=2,height=1,header=NS(stamp=Stamp(sec=1)))
    tf=NS(translation=NS(x=0,y=0,z=0),rotation=NS(x=0,y=0,z=0,w=1))
    floor=np.array([[1.,2.7,0.],[2.,2.7,0.]])
    with patch('go2_per.per_scan.per_logic.cloud_xyz',return_value=floor),patch('go2_per.per_scan.per_logic.ground_plane',return_value=np.zeros(3)),patch('go2_per.per_scan.per_logic.scans',return_value=([1.],[2.],[3.])) as scans:
        obj.process_cloud(msg,tf)
    np.testing.assert_allclose(scans.call_args.args[0],floor)
    np.testing.assert_allclose(obj.plane,np.zeros(3))
    for pub in (obj.obstacle_pub,obj.map_pub,obj.free_pub):pub.publish.assert_called_once()
