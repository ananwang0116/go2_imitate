"""Observed-only mapper. Owns /map, never publishes a goal or motion command."""
import array
import json
import math
import os
import time
from pathlib import Path
import cv2
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from go2_msgs.msg import LocStatus, Event
from std_msgs.msg import String
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from go2_common.node import Go2Node, CMD_QOS, SENSOR_QOS, EVENT_QOS, spin
from go2_sim.realtime_logic import ObservedMap
MAP_QOS=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)


class OnlineMapper(Go2Node):
    def __init__(self):
        super().__init__('sim_online_mapper','per')
        self.map=ObservedMap()
        self.obstacles,self.free={},{}
        self.pose=None;self.scan_at=-1e9;self.loc_ok=False
        self.map_dirty=True;self.last_publish=-1e9;self.saved=''
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',MAP_QOS)
        self.status_pub=self.create_publisher(String,'/sim/mapping_status',CMD_QOS)
        self.create_subscription(LaserScan,'/per_obstacle_scan',lambda m:self.store(self.obstacles,m),SENSOR_QOS)
        self.create_subscription(LaserScan,'/per_free_scan',lambda m:self.store(self.free,m),SENSOR_QOS)
        self.create_subscription(LocStatus,'/loc_status',lambda m:setattr(self,'loc_ok',m.state==LocStatus.OK),CMD_QOS)
        self.create_subscription(Event,'/loc_event',self.on_loc_event,EVENT_QOS)
        self.create_subscription(String,'/sim/map_command',self.command,CMD_QOS)
        self.create_timer(.2,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_maps()
        self.event('ONLINE_MAP_START',source='scans_only',width=self.map.w,height=self.map.h,resolution=self.map.res)

    def on_loc_event(self,m):
        if m.name=='JUMP':
            self.map.reset();self.pose=None;self.obstacles.clear();self.free.clear();self.map_dirty=True
            self.event('ONLINE_MAP_RESET',reason='localisation_jump')

    def command(self,m):
        if m.data=='save':
            try:self.save()
            except (OSError,ValueError) as exc:self.event('MAP_SAVE_FAILED',reason=str(exc))

    def tick(self):
        if self.loc_ok:self.consume_scan()
        now=time.monotonic()
        if now-self.last_publish<.5:return
        if self.map_dirty:self.publish_maps()
        self.last_publish=now
        self.status_pub.publish(String(data=json.dumps(dict(pose=self.pose,scan_age_s=now-self.scan_at,
            map_rev=self.map.rev,saved=self.saved))))

    def store(self, store, msg):
        key = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        store[key] = msg
        while len(store)>6:
            store.pop(min(store))

    def consume_scan(self):
        for key in sorted(set(self.obstacles)&set(self.free), reverse=True):
            o, f = self.obstacles[key], self.free[key]
            try:
                t = self.tf_buffer.lookup_transform('map','base_footprint',Time.from_msg(o.header.stamp)).transform
            except TransformException:
                continue
            q=t.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            pose=(t.translation.x,t.translation.y,yaw)
            angles=o.angle_min+np.arange(len(o.ranges))*o.angle_increment
            if len(o.ranges)!=len(f.ranges):
                continue
            self.map.update(pose, angles, np.asarray(o.ranges), np.asarray(f.ranges),
                            (self.geometry.body_half_length_m,self.geometry.body_half_width_m))
            self.map_dirty = True
            self.pose, self.scan_at = pose, time.monotonic()
            for store in (self.obstacles,self.free):
                for old in [k for k in store if k<=key]:
                    store.pop(old)
            return

    def grid_message(self, data):
        m=OccupancyGrid()
        m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.info.width=self.map.w;m.info.height=self.map.h;m.info.resolution=self.map.res
        m.info.origin.position.x=float(self.map.origin[0]);m.info.origin.position.y=float(self.map.origin[1])
        m.info.origin.orientation.w=1.
        m.data=array.array('b',data.tobytes())
        return m

    def publish_maps(self):
        self.map_pub.publish(self.grid_message(self.map.grid()))
        self.map_dirty=False

    def save(self):
        run=Path(os.environ['GO2_RUN_DIR'])
        folder=run/'maps'/str(time.time_ns())
        folder.mkdir(parents=True)
        grid=self.map.grid()
        image=np.where(grid<0,205,np.where(grid>=65,0,254)).astype(np.uint8)
        if not cv2.imwrite(str(folder/'map.pgm'),np.flipud(image)):
            raise ValueError('map_save_failed')
        meta=dict(image='map.pgm',resolution=self.map.res,origin=[*self.map.origin.tolist(),0.],
                  negate=0,occupied_thresh=.65,free_thresh=.196)
        (folder/'map.yaml').write_text(yaml.safe_dump(meta))
        self.saved=str(folder/'map.yaml')
        self.event('REALTIME_MAP_SAVED', path=self.saved)


def main():
    spin(OnlineMapper)
