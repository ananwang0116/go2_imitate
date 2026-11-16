from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from go2_web.web_server import WebTFBuffer, pose_is_fresh


def test_future_transform_from_previous_sim_clock_is_not_fresh():
    assert not pose_is_fresh(90_000_000_000,5_000_000_000,2.)
    assert not pose_is_fresh(1_000_000_000,5_000_000_000,2.)
    assert pose_is_fresh(4_000_000_000,5_000_000_000,2.)


def test_new_web_tf_buffer_contains_neither_old_static_nor_old_dynamic_pose():
    old=WebTFBuffer()
    static=TransformStamped();static.header.frame_id='map';static.child_frame_id='odom'
    static.transform.translation.x=4.;static.transform.rotation.w=1.
    old.set_transform_static(static,'old_fixed_map')
    dynamic=TransformStamped();dynamic.header.frame_id='odom';dynamic.child_frame_id='base_footprint'
    dynamic.header.stamp.sec=90;dynamic.transform.translation.x=3.;dynamic.transform.rotation.w=1.
    old.set_transform(dynamic,'old_loc_pose')
    assert old.lookup_transform('map','base_footprint',Time()).transform.translation.x==7.
    new=WebTFBuffer()
    assert not new.can_transform('map','base_footprint',Time())
    static.transform.translation.x=0.;dynamic.header.stamp.sec=5;dynamic.transform.translation.x=0.
    new.set_transform_static(static,'new_slam_origin');new.set_transform(dynamic,'new_loc_pose')
    assert new.lookup_transform('map','base_footprint',Time()).transform.translation.x==0.
