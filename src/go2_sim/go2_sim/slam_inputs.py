"""Gazebo ray/IMU adapter for unmodified Leg-KILO.

Ray clouds are instantaneous snapshots: all per-point `time` values are zero.
No simulated pose/model-state topic is consumed. The original sensor stamp and
lidar frame are retained; geometry is used only to remove the physical body.
"""
import array
import json
import time
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Imu
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rclpy.clock import Clock, ClockType
from go2_msgs.msg import Event
from go2_common import geometry
from go2_common.node import Go2Node, SENSOR_QOS, CMD_QOS, EVENT_QOS, spin
from go2_per.per_logic import cloud_xyz
from go2_sim.slam_logic import snapshot_points


class SlamInputs(Go2Node):
    def __init__(self):
        super().__init__('sim_slam_inputs','per')
        self.enabled=False;self.imu_count=self.cloud_count=0
        self.last_imu=self.last_cloud=-1e9
        self.rot=np.asarray(geometry.mid360_rotation(self.geometry)).reshape(3,3)
        self.trans=geometry.mid360_translation(self.geometry)
        self.cloud_pub=self.create_publisher(PointCloud2,'/sim/kilo_points',SENSOR_QOS)
        self.imu_pub=self.create_publisher(Imu,'/sim/kilo_imu',SENSOR_QOS)
        self.status_pub=self.create_publisher(String,'/sim/kilo_inputs',CMD_QOS)
        self.create_subscription(Event,'/dev_event',self.on_event,EVENT_QOS)
        self.create_subscription(Imu,'/go2/imu',self.on_imu,SENSOR_QOS,raw=True)
        self.create_subscription(PointCloud2,'/sim/mid360_points',self.on_cloud,SENSOR_QOS)
        self.create_timer(1.,self.status,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event('SLAM_INPUT_START',source='gazebo_sensors',point_time='instantaneous_zero',waiting='SIM_GAIT_READY')

    def on_event(self,m):
        if m.name=='SIM_GAIT_READY' and not self.enabled:
            self.enabled=True
            self.event('SLAM_INPUT_ENABLED',reason='standing_and_trot_stable')

    def on_imu(self,buf):
        if not self.enabled:return
        if self.imu_count==0:
            m=deserialize_message(buf,Imu)
            a=m.linear_acceleration;w=m.angular_velocity
            if m.header.frame_id!='base' or not np.isfinite([a.x,a.y,a.z,w.x,w.y,w.z]).all():
                self.warn_throttled('imu_frame','IMU must be finite and in base frame; refusing input')
                return
            self.event('SLAM_IMU_FIRST',frame=m.header.frame_id,acc_norm=float(np.linalg.norm([a.x,a.y,a.z])))
        self.imu_pub.publish(buf)
        self.imu_count+=1;self.last_imu=time.monotonic()

    def on_cloud(self,m):
        if not self.enabled or self.imu_count<50:return
        if m.is_bigendian or m.header.frame_id!='mid360_link':
            self.warn_throttled('format','unexpected cloud byte order/frame; refusing SLAM input')
            return
        offsets={f.name:f.offset for f in m.fields}
        if not all(k in offsets for k in ('x','y','z')):return
        pts=cloud_xyz(bytes(m.data),m.point_step,[offsets[k] for k in ('x','y','z')],m.width*m.height)
        data=snapshot_points(pts,self.rot,self.trans,self.geometry)
        if len(data)<100:return
        pts=data[:,:3]
        msg=PointCloud2();msg.header=m.header;msg.height=1;msg.width=len(pts)
        msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(('x','y','z','intensity','time'))]
        msg.point_step=20;msg.row_step=20*len(pts);msg.is_dense=True;msg.is_bigendian=False
        msg.data=array.array('B',data.tobytes())
        self.cloud_pub.publish(msg)
        self.cloud_count+=1;self.last_cloud=time.monotonic()
        if self.cloud_count==1:self.event('SLAM_CLOUD_FIRST',points=len(pts),time_span_s=0.,frame=m.header.frame_id)

    def status(self):
        now=time.monotonic()
        self.status_pub.publish(String(data=json.dumps(dict(enabled=self.enabled,imu_count=self.imu_count,
            cloud_count=self.cloud_count,imu_age_s=now-self.last_imu,cloud_age_s=now-self.last_cloud))))


def main():spin(SlamInputs)
