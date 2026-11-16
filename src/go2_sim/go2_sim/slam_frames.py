"""SLAM full body attitude/height TF and mapping-origin/initial-hint adapter.

No model states or ground-truth pose are read. loc_pose alone judges localisation
and publishes planar odom TF. Body height = configured initial standing floor
height + SLAM's relative z, an explicit initial floor-reference assumption.
"""
import math
import time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from go2_msgs.msg import LocStatus, Event
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, EVENT_QOS, spin


class SlamFrames(Go2Node):
    def __init__(self):
        super().__init__('sim_slam_frames','loc')
        self.mapping=bool(self.declare_parameter('mapping',True).value)
        self.initial_height=float(self.declare_parameter('initial_floor_height_m',.35).value)
        self.seed_x=float(self.declare_parameter('initial_x',0.).value)
        self.seed_y=float(self.declare_parameter('initial_y',0.).value)
        self.seed_yaw=float(self.declare_parameter('initial_yaw',0.).value)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        self.hint_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',CMD_QOS)
        self.ok_since=None;self.last_hint=-1e9;self.aligned=False
        self.create_subscription(Odometry,'/kilo_odom',self.on_odom,SENSOR_QOS)
        self.create_subscription(LocStatus,'/loc_status',self.on_status,CMD_QOS)
        self.create_subscription(Event,'/loc_event',self.on_event,EVENT_QOS)
        if self.mapping:
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
            self.static_tf.sendTransform(t)
        self.event('SLAM_FRAMES_START',mapping=self.mapping,initial_floor_height_m=self.initial_height)

    def on_odom(self,m):
        p=m.pose.pose;q=p.orientation
        values=[p.position.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(x) for x in values) or abs(p.position.z)>1.:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        t=TransformStamped();t.header.stamp=m.header.stamp;t.header.frame_id='base_footprint';t.child_frame_id='base'
        t.transform.translation.z=self.initial_height+p.position.z
        t.transform.rotation.x=cy*q.x+sy*q.y;t.transform.rotation.y=cy*q.y-sy*q.x
        t.transform.rotation.z=cy*q.z-sy*q.w;t.transform.rotation.w=cy*q.w+sy*q.z
        self.tf.sendTransform(t)

    def on_event(self,m):
        if m.name in ('ALIGN_OK','ALIGN_WEAK'):self.aligned=True

    def on_status(self,m):
        now=time.monotonic()
        if m.state!=LocStatus.OK:self.ok_since=None;return
        if self.ok_since is None:self.ok_since=now
        if now-self.ok_since<2.:return
        if self.mapping:
            if now-self.last_hint>2.:
                self.event('ALIGN_OK',reason='legkilo_mapping_origin',source='legkilo')
                self.last_hint=now
        elif not self.aligned and now-self.last_hint>5.:
            m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.pose.pose.position.x=self.seed_x;m.pose.pose.position.y=self.seed_y
            m.pose.pose.orientation.z=math.sin(self.seed_yaw/2);m.pose.pose.orientation.w=math.cos(self.seed_yaw/2)
            m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=.068
            self.hint_pub.publish(m);self.last_hint=now
            self.event('SLAM_INITIAL_HINT',source='configured_spawn_not_live_ground_truth',x=self.seed_x,y=self.seed_y)


def main():spin(SlamFrames)
