from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from tf2_ros import TransformException
from go2_loc.loc_map_check import LocMapCheck


def test_score_waits_for_scan_timestamp_tf_and_uses_older_resolvable_scan():
    obj=object.__new__(LocMapCheck)
    old=NS(header=NS(stamp=Time(sec=9,nanosec=400_000_000)))
    new=NS(header=NS(stamp=Time(sec=9,nanosec=900_000_000)))
    obj.field=object();obj.scan=new;obj.scans=deque([old,new]);obj.max_points=360
    obj.wall_tolerance_m=.1;obj.low_score=.45;obj.margin=.05;obj.low=None;obj.since=None
    obj.event=Mock();obj.score_pub=Mock()
    obj.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
    transform=NS(translation=NS(x=0,y=0),rotation=NS(x=0,y=0,z=0,w=1))
    obj.tf_buffer=Mock()
    obj.tf_buffer.lookup_transform.side_effect=[TransformException('not yet'),NS(transform=transform)]
    with patch('go2_loc.loc_map_check.scan_body_points',return_value=[]),patch('go2_loc.loc_map_check.to_map',return_value=[]),patch('go2_loc.loc_map_check.score',return_value=.8):
        obj.check()
    calls=obj.tf_buffer.lookup_transform.call_args_list
    assert [c.args[2].nanoseconds for c in calls]==[9_900_000_000,9_400_000_000]
    assert obj.score_pub.publish.call_args.args[0].data>.79
    obj.score_pub.reset_mock();obj.tf_buffer.lookup_transform.side_effect=TransformException('missing')
    obj.check();obj.score_pub.publish.assert_not_called()
