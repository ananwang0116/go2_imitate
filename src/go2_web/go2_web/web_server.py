#!/usr/bin/env python3
"""Pure Gazebo simulation console: HTTP UI, ROS goals/state and owned launch lifecycle.
The default managed launch is go2_sim/sim.launch.py. Fixed 2D maps are supplied by
users; realtime mode uses scan-built 2D grids. Leg-KILO provides localisation.
Live 3D cloud remains empty; both workspaces share one owned stack.
"""
import collections
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import KeyValue  # noqa: F401  (message type of Event.fields)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from go2_msgs.msg import DevState, Event, LocStatus, NavStatus, SafeStatus, TaskCmd, TaskStatus
from go2_msgs.srv import NavCheck

from go2_common import geometry as geometry_mod
from go2_common import names, speeds as speeds_mod
from go2_common.events import RUN_DIR_ENV, append_event_line, format_event, run_dir
from go2_common.node import CMD_QOS, EVENT_QOS, GEOMETRY_ENV, SENSOR_QOS, TASK_CMD_QOS
from go2_web import web_logic as wl
from go2_web.stack_manager import StackManager

TASK_COMMANDS = ('run_once', 'run_loop', 'cancel', 'load', 'clear')
WARN_EVENTS = {'SKIP', 'TROUBLE_BUDGET', 'ROUND_RETRY', 'SCORE_LOW', 'LINK_LOST', 'CMD_REJECT', 'GOAL_REJECT', 'CMD_STALE', 'LOOP_STOP', 'FAULT', 'NO_TF',
               'ALIGN_WEAK', 'PAUSE', 'LIMIT', 'IMU_GAP', 'JUMP', 'STALL'}
SCORE_LOW = 0.45                       # loc_map_check's low_score (the gauge mark); the judge stays loc_pose
JSON_HEADERS = {'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store'}


def clean(v):
    """JSON without NaN / inf."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [clean(x) for x in v]
    return v


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))



POSE_MAX_AGE_S = 2.0


def pose_is_fresh(stamp_ns, now_ns, max_age_s):
    """False for a transform that is older than max_age_s. The web console outlives the stacks it starts and keeps its TF buffer:
    after a restart the buffer still holds the previous run's LAST pose (board 2026-11-04: x=3.56 y=1.43) until the new run
    publishes, so the first track point / robot arrow was drawn there and a straight line ran from it to the new origin, which
    looked like a pose jump although /kilo_odom had moved 1.5 cm. A static-only chain has stamp 0 and counts as fresh."""
    return stamp_ns == 0 or 0 <= (now_ns - stamp_ns) * 1e-9 <= max_age_s

class WebTFBuffer(Buffer):
    """Track delivery of map->odom, so UI acknowledgement waits for TF too."""
    def __init__(self):
        super().__init__()
        self.map_version = 0

    def set_transform_static(self, transform, authority):
        super().set_transform_static(transform, authority)
        if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
            self.map_version += 1


class WebNode(Node):
    def __init__(self):
        super().__init__('web_server', start_parameter_services=False)     # fewer executor entities, see go2_common/node.py
        p = self.declare_parameter
        self.host = p('host', '127.0.0.1').value
        self.port = int(p('port', 8080).value)
        launch_mode = p('mode', '').value
        self.managed = launch_mode == ''
        self.mode = 'idle' if self.managed else launch_mode
        self.loaded_map = p('map', '').value
        self.ws = Path(p('workspace', os.environ.get('GO2_WS', '.')).value)
        self.launch_cmd = p('launch_cmd', 'ros2 launch go2_sim sim.launch.py').value.split()
        self.simulation = 'go2_sim' in self.launch_cmd
        self.offline = bool(p('offline', False).value)
        share = Path(get_package_share_directory('go2_bringup')) / 'config'
        os.environ.setdefault(GEOMETRY_ENV, str(share / 'geometry.yaml'))
        os.environ.setdefault(speeds_mod.SPEEDS_ENV, str(share / 'speeds.yaml'))
        self.stack = StackManager(self.ws, self.launch_cmd, kill_after_s=55.0) if self.managed else None
        self.maps_dir = Path(p('maps_dir', str(self.ws / 'maps')).value)
        self.missions_dir = Path(p('missions_dir', str(self.ws / 'missions')).value)
        self.web_dir = Path(p('web_dir', '').value or Path(__file__).resolve().parent.parent / 'web')
        self.speeds = speeds_mod.load()
        gfile = os.environ.get(GEOMETRY_ENV, '')
        self.geometry = geometry_mod.load(gfile) if gfile else None
        self.run = run_dir()
        self.run_args = {}
        self.run_args_run = None
        if self.run and (self.run / 'run.yaml').is_file():
            self.run_args = (yaml.safe_load((self.run / 'run.yaml').read_text()) or {}).get('args', {})
            self.run_args_run = self.run
        self.track, self.marks = [], []              # session-owned actual path in all modes and mapping parking marks
        self.aligned, self.last_align = False, None  # loc_align reported ALIGN_OK / ALIGN_WEAK in this run (task_executor's rule)
        self.health = []                             # mapping health: pose jumps / IMU holes of THIS mapping run (web_logic.map_health_item)
        self.last_stack_state = 'idle'
        self.session_id = ''
        self._pending_tf_reset = False
        self._accept_inputs = not self.managed
        self.alignment = wl.AlignmentFeedback()

        self.lock = threading.Lock()
        self.latest = {'loc': None, 'score': None, 'dev': None, 'safe': None, 'task': None, 'scan': None, 'nav': None}
        self.realtime = None
        self.realtime_stamp = 0.
        self.live_map = None
        self.live_map_image = b''
        self.live_map_rev = 0
        self.evidence = None
        self.evidence_image = b''
        self.evidence_rev = 0
        self.nav_path = []
        self.evidence_ttl = None     # {dynamic_ttl_s, static_ttl_s, static_min_s} in force in per_evidence (echo of /per_evidence_ttl)
        self.route_preview = []      # later legs of the running route (display only), from the original /mot_nav_check
        self._preview_future = None
        self.stamp = {k: 0.0 for k in self.latest}
        self.markers = []
        self.events = collections.deque(maxlen=80)
        self.nav_cloud_ttl_s = float(p('nav_cloud_ttl_s', 5.0).value)
        self.cloud = wl.RecentCloudAccumulator(ttl_s=self.nav_cloud_ttl_s) if self.mode == 'nav' else wl.CloudAccumulator()
        self.cloud_frame = ''
        self.map_odom_seen = None
        self.cloud_n = 0
        self.cloud_q = collections.deque(maxlen=4)    # clouds waiting for the pose of THEIR stamp: (stamp, frame_id, points, t0)
        self.pose = None
        self.scan_n = 0
        self.scan_q = collections.deque(maxlen=6)

        self.tf_buffer = WebTFBuffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        cloud_group = MutuallyExclusiveCallbackGroup()
        sub = self.create_subscription
        sub(LocStatus, names.LOC_STATUS, lambda m: self.keep('loc', m), CMD_QOS)
        sub(Float32, names.LOC_MAP_SCORE, lambda m: self.keep('score', float(m.data)), CMD_QOS)
        # High-rate topics are taken as raw CDR bytes and turned into messages only when the page asks (/dev_state is 50 Hz, the page polls
        # at 5 Hz) or for the every-n-th one that is used: rclpy builds a Python message for EVERY sample otherwise, and the measured
        # cost of one 50 Hz DevState subscription was ~70 % of a core on the desk (docs/02 session 34).
        sub(DevState, names.DEV_STATE, lambda b: self.keep('dev', b), CMD_QOS, raw=True)
        sub(SafeStatus, names.SAFE_STATUS, lambda m: self.keep('safe', m), CMD_QOS)
        sub(TaskStatus, names.TASK_STATUS, lambda m: self.keep('task', m), CMD_QOS)
        sub(MarkerArray, names.VIZ_TASK_MARKERS, self.on_markers, CMD_QOS)
        sub(String, names.PER_EVIDENCE_TTL, self.on_evidence_ttl, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.evidence_ttl_pub = self.create_publisher(String, names.PER_EVIDENCE_TTL_CMD, CMD_QOS)
        sub(NavStatus, names.MOT_NAV_STATUS, lambda m: self.keep('nav', m), CMD_QOS)
        sub(NavPath, names.MOT_NAV_PATH, self.on_nav_path, CMD_QOS)
        sub(LaserScan, names.PER_OBSTACLE_SCAN, self.on_scan, SENSOR_QOS, raw=True)
        sub(OccupancyGrid, names.PER_EVIDENCE_MAP, self.on_evidence,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(PointCloud2, names.DEV_LIDAR_CLOUD, self.on_cloud, SENSOR_QOS, callback_group=cloud_group, raw=True)
        sub(OccupancyGrid, '/map', self.on_live_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(String, '/sim/realtime_status', self.on_realtime, CMD_QOS)
        self.realtime_pub = self.create_publisher(String, '/sim/realtime_cmd', CMD_QOS)
        self.realtime_heartbeat_pub = self.create_publisher(String, '/sim/realtime_heartbeat', CMD_QOS)
        self.create_timer(0.1, self.drain_clouds, callback_group=cloud_group, clock=Clock(clock_type=ClockType.STEADY_TIME))      # 10 Hz (was 20): clouds are used at 2 Hz
        for topic in names.EVENT_TOPICS:
            sub(Event, topic, self.on_event, EVENT_QOS)
        self.teleop_pub = self.create_publisher(Twist, names.DEV_TELEOP_CMD_VEL, CMD_QOS)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, names.INITIALPOSE, CMD_QOS)
        self.goal_pub = self.create_publisher(PoseStamped, names.GOAL_POSE, CMD_QOS)
        self.cmd_pub = self.create_publisher(TaskCmd, names.TASK_CMD, TASK_CMD_QOS)
        self.stop_client = self.create_client(Trigger, '/dev_stop')
        self.check_client = self.create_client(NavCheck, names.MOT_NAV_CHECK)
        self.create_timer(0.1, self.update_pose, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(2.0, self.refresh_preview, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.start_s = time.time()
        self.note('WEB_START', host=self.host, port=self.port, mode=self.mode, map=self.loaded_map or '-')

    # ---------- helpers ----------
    def note(self, name, /, **fields):
        """A web console event: terminal + the current run's events.log (managed mode: the run of the stack it started, so
        WEB_ESTOP / WEB_STACK_* / WEB_GOAL land next to the stack's own events; before 2026-10-26 they were lost there)."""
        text = format_event('run', name, fields)
        self.get_logger().info(text)
        append_event_line(text, directory=self.run)

    def keep(self, key, value, session=None):
        if not self._accept_inputs:
            return
        with self.lock:
            if not self._accept_inputs or (session is not None and session!=self.session_id):
                return
            self.latest[key] = value
            self.stamp[key] = time.time()

    def update_pose(self):
        self.sync_stack()
        if self._pending_tf_reset:
            reset_session=self.session_id
            # On the ROS executor thread: replace, rather than reuse, all old
            # dynamic AND static transforms and rejoin transient-local TF.
            self.tf_listener.unregister()
            self.tf_buffer = WebTFBuffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._pending_tf_reset = self.session_id != reset_session
            self._accept_inputs = not self._pending_tf_reset and self.mode != 'idle' and not (self.managed and self.stack.stopping)
        with self.lock:
            state = self.alignment.advance(self.tf_buffer.map_version,time.monotonic())
            self.aligned = state in ('ok','weak')
        if not self._accept_inputs or state == 'pending':
            with self.lock:self.pose = None
            return
        with self.lock:
            loc = self.latest['loc']
            loc_fresh = time.time()-self.stamp['loc'] < 2.
        if loc is None or not loc_fresh or loc.state == LocStatus.LOST:
            with self.lock:self.pose = None
            return
        pose_session=self.session_id
        for frame in ((names.FRAME_MAP,) if self.mode in ('nav','realtime') else (names.FRAME_ODOM,)):
            try:
                ts = self.tf_buffer.lookup_transform(frame, names.FRAME_BASE_FOOTPRINT, Time())
                t, t_stamp_ns = ts.transform, ts.header.stamp.sec * 10**9 + ts.header.stamp.nanosec
            except TransformException:
                continue
            if not pose_is_fresh(t_stamp_ns, self.get_clock().now().nanoseconds, POSE_MAX_AGE_S):
                continue                                 # the previous run's last pose is still in our TF buffer (see pose_is_fresh)
            with self.lock:
                if pose_session!=self.session_id or self._pending_tf_reset or not self._accept_inputs:
                    self.pose=None
                    return
                self.pose = (t.translation.x, t.translation.y, yaw_of(t.rotation), frame)
                if len(self.track) < 40000:
                    x, y = t.translation.x, t.translation.y
                    if not self.track or math.hypot(x - self.track[-1][0], y - self.track[-1][1]) >= 0.05:
                        self.track.append([x, y])
            self.drain_scans()
            return
        with self.lock:
            self.pose = None

    # ---------- inputs ----------
    def on_event(self, m):
        if not self._accept_inputs:
            return
        try:
            text = format_event(m.layer, m.name, {kv.key: kv.value for kv in m.fields}).replace('"', '')
        except ValueError:
            return
        if self.mode == 'map':
            item = wl.map_health_item(m.layer, m.name, {kv.key: kv.value for kv in m.fields}, time.time())
            if item is not None:
                with self.lock:
                    if len(self.health) < 200:
                        self.health.append(item)
        if m.layer == 'loc' and m.name.startswith('ALIGN_') and m.name != 'ALIGN_DEFAULT':
            self.cloud.clear()                              # the map -> odom shift changed: old points are in the wrong place
            with self.lock:
                self.alignment.event(m.name,{kv.key:kv.value for kv in m.fields},
                                     m.header.stamp.sec*10**9+m.header.stamp.nanosec)
                self.aligned = self.alignment.advance(self.tf_buffer.map_version,time.monotonic()) in ('ok','weak')
            self.last_align = {'t': time.time(), 'name': m.name, 'fields': {kv.key: kv.value for kv in m.fields}}
        with self.lock:
            self.events.append({'t': time.time(), 'layer': m.layer, 'name': m.name, 'text': text,
                                'warn': m.name in WARN_EVENTS})

    def on_markers(self, msg):
        if not self._accept_inputs:
            return
        receive_session=self.session_id
        rows = {}
        for m in msg.markers:
            if m.ns == 'wp_yaw' and m.action == m.ADD:         # the heading the user gave (drawn as an arrow in the circle)
                rows.setdefault(m.id, {'i': m.id})['yaw'] = 2*math.atan2(m.pose.orientation.z, m.pose.orientation.w)
                continue
            if m.ns != 'wp' or m.action != m.ADD:
                continue
            k = m.id // 2
            row = rows.setdefault(k, {'i': k})
            if m.id % 2 == 0:
                row.update(x=m.pose.position.x, y=m.pose.position.y, r=m.scale.x / 2)
            else:
                parts = m.text.split(' ', 1)[1] if ' ' in m.text else ''
                state, _, problem = parts.partition(': ')
                row.update(state=state, problem=problem)
        with self.lock:
            if receive_session!=self.session_id or not self._accept_inputs:
                return
            self.markers = [rows[k] for k in sorted(rows) if 'x' in rows[k]]

    def refresh_preview(self):
        """Display only: the A* legs between the CURRENT target and the later waypoints of the running route, from the
        original planner service (one judge). Never used for control; cleared when the task is not running."""
        from types import SimpleNamespace
        from go2_task.task_preflight import request_from
        if not self._accept_inputs:
            return
        fut = self._preview_future
        if fut is not None and not fut.done():
            return
        with self.lock:
            task, markers, session = self.latest.get('task'), list(self.markers), self.session_id
        if task is None or task.mode not in ('once', 'loop'):
            with self.lock:
                self.route_preview = []
            return
        rest = [m for m in markers if m['i'] >= task.waypoint_index and m.get('state') in ('APPROACH', 'RECOVER', 'PENDING')]
        if len(rest) < 2 or not self.check_client.service_is_ready():
            with self.lock:
                self.route_preview = []
            return
        req = request_from([SimpleNamespace(x=m['x'], y=m['y'], radius_m=max(m['r'], 0.2), path=None) for m in rest])

        def done(f, session=session):
            try:
                res = f.result()
            except Exception:
                return
            legs = []
            if res is not None and res.available and len(res.leg_path_off) == len(rest)+1:
                for k in range(1, len(rest)):
                    a, b = res.leg_path_off[k], res.leg_path_off[k+1]
                    pts = [[float(x), float(y)] for x, y in zip(res.leg_path_x[a:b], res.leg_path_y[a:b])]
                    if len(pts) > 1:
                        step = max(1, len(pts) // 80)
                        legs.append(pts[::step] + ([pts[-1]] if (len(pts)-1) % step else []))
            with self.lock:
                if session == self.session_id:
                    self.route_preview = legs
        self._preview_future = self.check_client.call_async(req)
        self._preview_future.add_done_callback(done)

    def on_evidence_ttl(self, m):
        if not self._accept_inputs:
            return
        try:
            d = json.loads(m.data)
            ttl = {k: float(d[k]) for k in ('dynamic_ttl_s', 'static_ttl_s', 'static_min_s')}
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            self.evidence_ttl = ttl

    def set_evidence_ttl(self, dynamic_s, static_s):
        """Layer panel: how long dynamic (yellow) / static (cyan) obstacles stay. per_evidence validates and echoes it."""
        dynamic_s, static_s = float(dynamic_s), float(static_s)
        if not (0.5 <= dynamic_s <= 60. and 1. <= static_s <= 3600.):
            raise ValueError('动态 0.5–60 秒，静态 1–3600 秒')
        self.evidence_ttl_pub.publish(String(data=json.dumps({'dynamic_ttl_s': dynamic_s, 'static_ttl_s': static_s})))
        self.note('WEB_EVIDENCE_TTL', dynamic_s=dynamic_s, static_s=static_s)

    def on_nav_path(self, m):
        if not self._accept_inputs:
            return
        receive_session=self.session_id
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in m.poses]
        with self.lock:
            if receive_session!=self.session_id or not self._accept_inputs:
                return
            self.nav_path = pts[::max(1, len(pts) // 300)]

    def on_scan(self, buf):
        if not self._accept_inputs:
            return
        session=self.session_id
        self.scan_n+=1
        if self.scan_n%2:return
        m=deserialize_message(buf,LaserScan)
        with self.lock:
            if session!=self.session_id or not self._accept_inputs:return
            self.scan_q.append((m,time.monotonic(),session))
        self.drain_scans()

    def drain_scans(self):
        """LIO pose follows the cloud: wait briefly for the scan's OWN TF."""
        if not self._accept_inputs:return
        now=time.monotonic()
        for m,received,session in reversed(list(self.scan_q)):
            if session!=self.session_id or now-received>1.:continue
            try:
                tf=self.tf_buffer.lookup_transform('map','base_footprint',Time.from_msg(m.header.stamp)).transform
            except TransformException:
                continue
            r=np.asarray(m.ranges,dtype=np.float64)
            angles=m.angle_min+np.arange(len(r))*m.angle_increment
            ok=np.isfinite(r)&(r>m.range_min)&(r<m.range_max)
            a=angles[ok]+yaw_of(tf.rotation)
            px=tf.translation.x+r[ok]*np.cos(a);py=tf.translation.y+r[ok]*np.sin(a)
            step=max(1,len(px)//360)
            points=[[round(float(x),2),round(float(y),2)] for x,y in zip(px[::step],py[::step])]
            self.keep('scan',points,session)
            with self.lock:
                if session!=self.session_id:return
                used=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                while self.scan_q:
                    first=self.scan_q[0][0].header.stamp
                    if first.sec*10**9+first.nanosec>used:break
                    self.scan_q.popleft()
            return
        with self.lock:
            while self.scan_q and now-self.scan_q[0][1]>1.:self.scan_q.popleft()

    def on_live_map(self, msg):
        if not self._accept_inputs:
            return
        receive_session=self.session_id
        if self.mode != 'realtime':
            return
        import cv2
        i = msg.info
        grid = np.asarray(msg.data, np.int8).reshape(i.height, i.width)
        pixels = np.flipud(np.where(grid < 0, 205, np.where(grid >= 65, 30, 250)).astype(np.uint8))
        ok, image = cv2.imencode('.png', pixels)
        if not ok:
            return
        with self.lock:
            if receive_session!=self.session_id or not self._accept_inputs:
                return
            self.live_map_rev += 1
            self.live_map_image = image.tobytes()
            self.live_map = dict(rev=self.live_map_rev,width=i.width,height=i.height,resolution=i.resolution,
                                 origin=[i.origin.position.x,i.origin.position.y])

    def on_realtime(self, msg):
        if not self._accept_inputs:
            return
        receive_session=self.session_id
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            if receive_session!=self.session_id or not self._accept_inputs:
                return
            self.realtime, self.realtime_stamp = data, time.monotonic()

    def realtime_command(self, body):
        self.sync_stack()
        if self.mode != 'realtime':
            raise ValueError('start realtime navigation first')
        action = body.get('action')
        if action not in ('manual','point','explore','pause','stop','save'):
            raise ValueError('invalid realtime action')
        if action in ('point','explore') and not 0<len(str(body.get('client','')))<=80:
            raise ValueError('explicit control client required; reload the page')
        if action == 'point':
            if not all(math.isfinite(float(body[k])) for k in ('x','y')) or (body.get('yaw') is not None and not math.isfinite(float(body['yaw']))):
                raise ValueError('finite target required')
        self.realtime_pub.publish(String(data=json.dumps(body)))
        self.note('WEB_REALTIME', action=action)

    def on_evidence(self, msg):
        if not self._accept_inputs:
            return
        receive_session=self.session_id
        if msg.header.frame_id != 'map':
            return
        i = msg.info
        image = wl.evidence_png(msg.data, i.width, i.height)
        with self.lock:
            if receive_session!=self.session_id or not self._accept_inputs:
                return
            self.evidence_rev += 1
            self.evidence_image = image
            self.evidence = {'rev': self.evidence_rev, 'width': i.width, 'height': i.height,
                             'resolution': i.resolution,
                             'origin': [i.origin.position.x, i.origin.position.y]}

    def on_cloud(self, buf):
        if not self._accept_inputs:
            return
        self.cloud_n += 1
        if self.cloud_n % 5:
            return
        msg = deserialize_message(buf, PointCloud2)
        pts = wl.parse_cloud2([(f.name, f.offset, f.datatype) for f in msg.fields], msg.point_step, bytes(msg.data),
                              msg.width * msg.height)
        if len(pts):
            self.cloud_q.append((Time.from_msg(msg.header.stamp), msg.header.frame_id, pts, time.monotonic()))

    def map_odom_moved(self):
        """True once when map -> odom differs from the last seen by > 3 cm or > 0.6 deg (the user's initial pose aligned the map)."""
        try:
            t = self.tf_buffer.lookup_transform(names.FRAME_MAP, names.FRAME_ODOM, Time()).transform
        except TransformException:
            return False
        cur = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        last, self.map_odom_seen = self.map_odom_seen, cur
        if last is None:
            return False
        dyaw = abs((cur[2] - last[2] + math.pi) % (2 * math.pi) - math.pi)
        return math.hypot(cur[0] - last[0], cur[1] - last[1]) > 0.03 or dyaw > 0.01

    def drain_clouds(self):
        """Use ONLY the cloud's own TF stamp. Wait at most 2 s, then drop; never paint with a stale/latest fallback.
        Navigation display expires old batches after 5 s, even when input stops; mapping retains complete accumulation.
        """
        self.cloud.expire()
        loc = self.latest['loc']
        if not wl.cloud_pose_usable(None if loc is None else loc.state, time.time() - self.stamp['loc']):
            for stamp, frame_id, pts, t0 in self.cloud_q:
                self.trace_cloud(stamp, 0, t0, 'drop_localisation', frame_id, len(pts))
            self.cloud_q.clear()
            return
        keep = []
        for stamp, frame_id, pts, t0 in list(self.cloud_q):
            late = time.monotonic() - t0 > 2.0
            if late:
                self.trace_cloud(stamp, 0, t0, 'drop_tf_timeout', frame_id, len(pts))
                continue
            done = False
            for frame in ((names.FRAME_MAP, names.FRAME_ODOM) if self.mode == 'nav' else (names.FRAME_ODOM,)):
                try:
                    ts = self.tf_buffer.lookup_transform(frame, frame_id, stamp)
                    t = ts.transform
                except TransformException:
                    continue
                if frame != self.cloud_frame:
                    self.cloud.clear()
                    self.cloud_frame = frame
                if frame == names.FRAME_MAP and self.map_odom_moved():
                    self.cloud.clear()           # points added in `map` before the alignment are in the wrong place now (same rule as per_evidence)
                    self.note('CLOUD_RESET', reason='map_to_odom_changed')
                q = t.rotation
                self.cloud.add(wl.transform_points(pts, (t.translation.x, t.translation.y, t.translation.z), (q.x, q.y, q.z, q.w)))
                self.trace_cloud(stamp, ts.header.stamp.sec + ts.header.stamp.nanosec * 1e-9, t0, 'accepted', frame_id, len(pts))
                done = True
                break
            if not done and not late:
                keep.append((stamp, frame_id, pts, t0))
        self.cloud_q.clear()
        self.cloud_q.extend(keep)

    def trace_cloud(self, stamp, tf_stamp_s, queued_at, decision, frame, points):
        """One row per accepted/dropped display batch (~2 Hz), never per point or per TF message."""
        if not self.run:
            return
        path = self.run / 'cloud_trace.csv'
        first = not path.exists()
        with path.open('a') as f:
            w = csv.writer(f)
            if first:
                w.writerow(['receive_s', 'cloud_stamp_s', 'tf_stamp_s', 'wait_s', 'decision', 'source_frame', 'points', 'latest_tf_fallback'])
            w.writerow([time.time(), stamp.nanoseconds * 1e-9, tf_stamp_s, time.monotonic() - queued_at, decision, frame, points, False])

    # ---------- stack (managed mode) ----------
    def sync_stack(self):
        """Mirror the managed stack's mode / map / run directory into the fields the rest of this node reads."""
        if not self.managed:
            return
        st = self.stack.status()
        alive = st['state'] in ('starting', 'stopping')
        mode = st['mode'] if alive else 'idle'
        session = (st.get('session_id') or f"{st.get('pid')}:{st.get('since')}") if alive else ''
        if session != self.session_id:
            if self.run and self.track:
                with self.lock:
                    (self.run/'web_track.json').write_text(json.dumps(dict(session_id=self.session_id,
                        path=self.track,marks=self.marks,frame='odom' if self.mode=='map' else 'map')))
            self.session_id = session
            self._accept_inputs = False
            self._pending_tf_reset = True
            self.cloud = wl.RecentCloudAccumulator(ttl_s=self.nav_cloud_ttl_s) if mode in ('nav','realtime') else wl.CloudAccumulator()
            self.cloud_q.clear();self.cloud_frame='';self.map_odom_seen=None
            self.track,self.marks=[],[]
            with self.lock:
                self.pose=None;self.scan_q.clear();self.nav_path=[];self.markers=[];self.route_preview=[];self._preview_future=None;self.evidence_ttl=None
                self.latest={k:None for k in self.latest}
                self.stamp={k:0. for k in self.stamp}
                self.events.clear();self.health=[]
                self.alignment.reset();self.aligned=False;self.last_align=None
                self.realtime=self.live_map=self.evidence=None
                self.realtime_stamp=0.;self.live_map_image=b'';self.evidence_image=b''
        if st['state']=='stopping':
            self._accept_inputs=False
            with self.lock:self.pose=None;self.latest['scan']=None
        self.mode, self.loaded_map = mode, st['map']
        new_run = self.stack.run_dir() if alive else self.run
        # Directory discovery can precede metadata publication. The run file is
        # atomically published; absence is a startup state, not an HTTP error.
        if new_run and new_run != self.run_args_run:
            try:
                metadata=yaml.safe_load((new_run/'run.yaml').read_text())
            except (OSError,yaml.YAMLError):
                metadata=None
            if isinstance(metadata,dict) and isinstance(metadata.get('args'),dict):
                self.run_args=metadata['args']
                self.run_args_run=new_run
        self.run = new_run
        self.run_args['enable_motion'] = 'true' if st['enable_motion'] else 'false'
        if self.last_stack_state != st['state']:
            self.last_stack_state = st['state']
            self.note('WEB_STACK', state=st['state'], mode=st['mode'] or '-', map=st['map'] or '-')

    def stack_state(self, dev_ok, loc_ok, task_ok):
        if not self.managed:
            return None
        st = self.stack.status()
        if st['state'] == 'starting' and dev_ok and (self.mode == 'map' or (loc_ok and task_ok)):
            if not self.simulation or self.stack.log_has('SIM_READY'):
                st['state'] = 'running'
        return st

    def start_stack(self, mode, map_name, enable_motion):
        self.sync_stack()
        if not self.managed:
            raise PermissionError('the stack was started from a terminal; stop it there (Ctrl-C)')
        if mode in ('nav','realtime'):
            wl.check_name(map_name)
        if self.simulation:
            mode, enable_motion = ('realtime' if mode == 'realtime' else 'nav'), True
        pid = self.stack.start(mode, map_name, bool(enable_motion), self.offline)
        self.note('WEB_STACK_START', mode=mode, map=map_name or '-', enable_motion=bool(enable_motion), pid=pid)
        return pid

    def stop_stack(self):
        if not self.managed:
            raise PermissionError('the stack was started from a terminal; stop it there (Ctrl-C)')
        if self.mode == 'realtime':
            self.realtime_command({'action':'stop'})
        if self.mode in ('nav','realtime'):
            self.send_cmd('cancel', '')                    # stop the task first (the dog stops before the nodes go)
        ok = self.stack.stop()
        self.note('WEB_STACK_STOP', was_running=ok)
        return ok

    # ---------- state for the page ----------
    def state(self):
        self.sync_stack()
        now = time.time()
        with self.lock:
            L, score, dev, safe, task, scan, nav = (self.latest[k] for k in ('loc', 'score', 'dev', 'safe', 'task', 'scan', 'nav'))
            dev = None if dev is None else deserialize_message(dev, DevState)
            age = {k: now - v if v else None for k, v in self.stamp.items()}
            pose, markers, events = self.pose, list(self.markers), list(self.events)
        with self.lock:
            self.alignment.advance(self.tf_buffer.map_version,time.monotonic())
            alignment=self.alignment.status()
            self.aligned=alignment['state'] in ('ok','weak')
        if age['scan'] is None or age['scan']>1.:scan=[]
        out = {'session_id': self.session_id, 'alignment':alignment, 'simulation': self.simulation, 't': now, 'mode': self.mode, 'map': self.loaded_map, 'run': self.run.name if self.run else '',
               'external_slam': False,
               'localisation': self.run_args.get('localisation','legkilo'),
               'evidence': self.evidence if self.mode in ('nav','realtime') else None,
               'live_map': self.live_map if self.mode == 'realtime' else None,
               'realtime': self.realtime if self.mode == 'realtime' and time.monotonic()-self.realtime_stamp<3. else None,
               'enable_motion': str(self.run_args.get('enable_motion', '')).lower() == 'true',
               'speeds': {'forward': self.speeds.forward_mps, 'back': self.speeds.back_mps, 'side': self.speeds.side_mps,
                          'turn': self.speeds.turn_radps},
               'score_low': SCORE_LOW, 'cloud_rev': self.cloud.rev, 'cloud_n': len(self.cloud.pts),
               'cloud_frame': self.cloud_frame,
               'cloud_ttl_s': self.nav_cloud_ttl_s if self.mode == 'nav' else 0,
               'pose': None if pose is None else {'x': pose[0], 'y': pose[1], 'th': pose[2], 'frame': pose[3]},
               'scan': scan or [], 'markers': markers, 'events': events[-40:]}
        out['loc'] = None if L is None or age['loc'] > 3 else {
            'state': ('OK', 'DEGRADED', 'LOST')[min(L.state, 2)], 'reason': L.reason, 'kilo_age_s': L.kilo_age_s,
            'map_score': score if score is not None and age['score'] is not None and age['score'] < 3. else None}
        out['dev'] = None if dev is None or age['dev'] > 3 else {
            'fault': dev.fault, 'reason': dev.fault_reason, 'battery_v': dev.battery_v, 'temp_c': dev.max_motor_temp_c,
            'state_age_s': dev.state_age_s, 'error_code': dev.error_code, 'speed': dev.velocity_mps[0],
            'body_height_m': dev.body_height_m, 'roll_rad': dev.roll_rad, 'pitch_rad': dev.pitch_rad}
        out['safe'] = None if safe is None or age['safe'] > 3 else {
            'limiting': safe.limiting, 'direction': safe.direction, 'reason': safe.reason, 'gap_m': safe.gap_m}
        out['task'] = None if task is None or age['task'] > 3 else {
            'mission': task.mission, 'mode': task.mode, 'index': task.waypoint_index, 'count': task.waypoint_count,
            'wp_state': task.waypoint_state, 'budget_s': task.budget_used_s, 'tries': task.tries,
            'remaining_m': task.remaining_m}
        out['nav'] = None if nav is None or age['nav'] > 3 else {
            'id': nav.id, 'state': ('IDLE', 'FOLLOWING', 'ARRIVED', 'FAILED')[min(nav.state, 3)], 'reason': nav.reason,
            'remaining_m': nav.remaining_m, 'clearance_m': nav.clearance_m, 'plan_age_s': nav.plan_age_s}
        out['nav_path'] = self.nav_path if out['nav'] and out['nav']['state'] in ('FOLLOWING',) else []
        out['evidence_ttl'] = self.evidence_ttl
        out['route_preview'] = self.route_preview if out['task'] and out['task']['mode'] in ('once', 'loop') else []
        alerts = []
        if out['dev'] is None:
            alerts.append({'level': 'bad', 'text': '没有收到狗的状态（/dev_state）'})
        elif out['dev']['fault']:
            alerts.append({'level': 'bad', 'text': '狗报故障：' + out['dev']['reason']})
        if out['loc'] and out['loc']['state'] == 'LOST':
            alerts.append({'level': 'bad', 'text': '定位丢失：' + (out['loc']['reason'] or '-')})
        elif out['loc'] and out['loc']['state'] == 'DEGRADED':
            alerts.append({'level': 'warn', 'text': '定位偏低：' + (out['loc']['reason'] or '-')})
        if out['safe'] and out['safe']['limiting']:
            alerts.append({'level': 'warn', 'text': f'安全限速：{out["safe"]["direction"]} {out["safe"]["reason"]}'})
        for e in events[-40:]:
            if e['warn'] and now - e['t'] < 30 and e['name'] not in ('LIMIT', 'PAUSE', 'JUMP', 'STALL'):
                if e['name'] == 'TROUBLE_BUDGET':
                    alerts.append({'level': 'bad' if 'abandon' in e['text'] else 'warn', 'text':
                                   '处理问题的时间已用满90秒，先停止，再继续尝试一轮（问题处理总共最多180秒）' if 'retry' in e['text'] else
                                   '处理问题总共180秒仍没解决，已放弃这个点'})
                elif e['name'] == 'ROUND_RETRY':
                    alerts.append({'level': 'warn', 'text': '这个点没走成：先停止，再重新尝试一次'})
                else:
                    alerts.append({'level': 'warn', 'text': e['text']})
        with self.lock:
            out['map_health'] = wl.map_health(list(self.health)) if self.mode == 'map' else None
        if out['map_health'] and out['map_health']['bad']:      # first, so the 8-line cut never drops it
            alerts.insert(0, {'level': 'bad', 'text': '建图可能已经坏了：' + out['map_health']['first']['text'] + ' —— 之后的地图会歪或重影，建议停止重做'})
        out['alerts'] = alerts[:1] + alerts[1:][-7:] if out['map_health'] and out['map_health']['bad'] else alerts[-8:]
        out['managed'] = self.managed
        out['stack'] = self.stack_state(out['dev'] is not None, out['loc'] is not None, out['task'] is not None)
        out['track_n'], out['marks_n'] = len(self.track), len(self.marks)
        out['aligned'], out['last_align'] = self.aligned, self.last_align
        out['finish'] = None
        if self.managed and out['stack']['state'] in ('idle', 'failed'):
            out['alerts'] = [a for a in out['alerts'] if '/dev_state' not in a['text']]      # nothing is expected to run
        return clean(out)

    # ---------- commands ----------
    def task_active(self):
        with self.lock:
            t = self.latest['task']
            return t is not None and t.mode in ('once', 'loop') and time.time() - self.stamp['task'] < 3

    def teleop(self, vx, vy, wz):
        if self.mode == 'realtime' and (not self.realtime or self.realtime.get('mode') != 'manual'):
            raise PermissionError('switch to manual mode before driving')
        if self.task_active():
            raise PermissionError('a task is running: cancel it before driving by hand')
        s = self.speeds
        m = Twist()
        m.linear.x = max(-s.back_mps, min(s.forward_mps, float(vx)))
        m.linear.y = max(-s.side_mps, min(s.side_mps, float(vy)))
        m.angular.z = max(-s.turn_radps, min(s.turn_radps, float(wz)))
        self.teleop_pub.publish(m)

    def estop(self):
        if self.mode == 'realtime':
            self.realtime_command({'action':'stop'})
        for _ in range(3):
            self.teleop_pub.publish(Twist())
        self.send_cmd('cancel', '')
        called = False
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
            called = True
        self.note('WEB_ESTOP', dev_stop=called)
        return called

    def send_cmd(self, command, arg):
        m = TaskCmd(command=command, arg=arg)
        m.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(m)

    def initialpose(self, x, y, yaw):
        self.sync_stack()
        if self.mode != 'nav' or not self._accept_inputs:
            raise ValueError('先启动固定地图导航并等待定位数据')
        if self.task_active():
            raise PermissionError('先结束当前任务，再设置初始定位')
        if not all(math.isfinite(float(v)) for v in (x,y,yaw)):
            raise ValueError('initial pose must be finite')
        msg=self.pose_msg(x,y,yaw,PoseWithCovarianceStamped)
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        with self.lock:
            request_id=self.alignment.request(ns,self.tf_buffer.map_version,time.monotonic())
            self.aligned=False;self.pose=None;self.latest['scan']=None
            self.track,self.marks=[],[]
            self.latest['score']=None;self.stamp['score']=0.
        self.init_pub.publish(msg)
        self.note('WEB_INITIALPOSE',request_id=request_id,x_m=float(x),y_m=float(y),yaw_rad=float(yaw))
        return request_id

    def pose_msg(self, x, y, yaw, cls):
        m = cls()
        stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.header.stamp = names.FRAME_MAP, stamp
        pose = m.pose.pose if cls is PoseWithCovarianceStamped else m.pose
        pose.position.x, pose.position.y = float(x), float(y)
        if yaw is not None:                      # no yaw: an all-zero quaternion = no preferred heading (the executor skips the final turn)
            pose.orientation.z, pose.orientation.w = math.sin(float(yaw) / 2), math.cos(float(yaw) / 2)
        if cls is PoseWithCovarianceStamped:
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = 0.068
        return m

    def check_waypoints(self, wps):
        """The check is mot_nav's own planner (service /mot_nav_check): the same judge that drives the dog (P2).
        Returns one list of problem texts per waypoint ([] = ok)."""
        from go2_task.task_preflight import request_from
        rows = wl.mission_rows(wps)
        w = _waypoints(rows, self)
        if not self.check_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('mot_nav does not answer (the stack must be running in navigation mode)')
        pose = self.pose
        done = threading.Event()
        fut = self.check_client.call_async(request_from(w, (pose[0], pose[1]) if pose and pose[3] == names.FRAME_MAP else None))
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(10.0):
            raise RuntimeError('mot_nav did not answer the check within 10 s')
        res = fut.result()
        if res is None or not res.available:
            raise RuntimeError('mot_nav has no costmap yet (wait for the map)')
        return [[r] if r else [] for r in res.reason]


class Handler(BaseHTTPRequestHandler):
    node: WebNode = None
    server_version = 'go2web'

    def log_message(self, fmt, *args):
        pass

    # ----- plumbing -----
    def send(self, code, body=b'', ctype='application/json; charset=utf-8', extra=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (extra or {}).items():
            self.send_header(k, str(v))
        self.end_headers()
        self.wfile.write(body)

    def json(self, obj, code=200):
        self.send(code, json.dumps(clean(obj), ensure_ascii=False).encode())

    def fail(self, code, msg):
        self.json({'error': str(msg)}, code)

    def same_origin(self):
        origin = self.headers.get('Origin')
        return origin is None or urlparse(origin).netloc == self.headers.get('Host')

    def body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n > 2_000_000:
            raise ValueError('body too large')
        return json.loads(self.rfile.read(n) or b'{}')

    # ----- GET -----
    def do_GET(self):
        u = urlparse(self.path)
        parts = [unquote(s) for s in u.path.strip('/').split('/')]
        q = parse_qs(u.query)
        n = self.node
        try:
            if u.path in ('/realtime', '/realtime.html'):
                return self.send(200, (n.web_dir / 'index.html').read_bytes(), 'text/html; charset=utf-8')
            if u.path in ('/', '/index.html'):
                return self.send(200, (n.web_dir / 'index.html').read_bytes(), 'text/html; charset=utf-8')
            if parts[0] != 'api':
                return self.fail(404, 'not found')
            if parts[1:] == ['state']:
                return self.json(n.state())
            if parts[1:] == ['maps']:
                n.sync_stack()
                return self.json({'maps': wl.list_maps(n.maps_dir), 'loaded': n.loaded_map, 'mode': n.mode,
                                  'missions': wl.list_missions(n.missions_dir)})
            if parts[1] == 'map' and len(parts) == 4:
                d = n.maps_dir / wl.check_name(parts[2])
                if parts[3] == 'meta':
                    return self.json(wl.load_meta(d)[0])
                if parts[3] == 'image.png':
                    return self.send(200, wl.map_png(d)[1], 'image/png')
                if parts[3] == 'cloud':
                    pts = wl.thin(wl.voxel_downsample(wl.read_pcd_xyz(d / 'cloud.pcd'), 0.05), int(q.get('max', ['400000'])[0]))
                    return self.send(200, wl.cloud_bytes(pts), 'application/octet-stream', {'X-Count': len(pts)})
            if parts[1:] == ['live-map.png']:
                with n.lock:
                    data = n.live_map_image
                if not data:
                    return self.fail(404, 'waiting for online map')
                return self.send(200, data, 'image/png')
            if parts[1:] == ['evidence.png']:
                with n.lock:
                    data = n.evidence_image
                if not data:
                    return self.fail(404, 'no obstacle evidence yet')
                return self.send(200, data, 'image/png', {'Cache-Control': 'no-store'})
            if parts[1:] == ['cloud']:
                with n.lock:
                    pts = n.cloud.pts
                pts = wl.thin(pts, int(q.get('max', ['400000'])[0]))
                return self.send(200, wl.cloud_bytes(pts), 'application/octet-stream',
                                 {'X-Count': len(pts), 'X-Rev': n.cloud.rev})
            if parts[1:] == ['track']:
                with n.lock:
                    return self.json({'session_id':n.session_id, 'alignment_request_id':n.alignment.status()['request_id'], 'path': n.track[::max(1, len(n.track) // 3000)], 'marks': list(n.marks)})
            if parts[1:] == ['routes']:
                mp = q.get('map', [n.loaded_map])[0]
                return self.json({'routes': wl.list_routes(n.maps_dir, mp) if mp else []})
            if parts[1] == 'route' and len(parts) == 3:
                r = wl.read_route(n.maps_dir, q.get('map', [n.loaded_map])[0], parts[2])
                frm = (float(q['from_x'][0]), float(q['from_y'][0])) if 'from_x' in q and 'from_y' in q else None
                r['waypoints'] = wl.route_segments(r['path'], r['marks'], frm)
                return self.json(r)
            if parts[1] == 'mission' and len(parts) == 3:
                data = yaml.safe_load((n.missions_dir / f'{wl.check_name(parts[2])}.yaml').read_text())
                return self.json(data)
            return self.fail(404, 'not found')
        except (OSError, ValueError, KeyError) as e:
            return self.fail(400, e)

    # ----- POST -----
    def do_POST(self):
        if not self.same_origin() or self.headers.get('X-Go2') != '1':
            return self.fail(403, 'bad origin')
        parts = [unquote(s) for s in urlparse(self.path).path.strip('/').split('/')]
        n = self.node
        try:
            b = self.body()
            if parts[0] != 'api' or len(parts) < 2:
                return self.fail(404, 'not found')
            what = parts[1]
            if what == 'realtime':
                if parts[2:] == ['heartbeat']:
                    if n.mode == 'realtime':
                        client = str(b.get('client',''))
                        if not 0<len(client)<=80:
                            raise ValueError('invalid client lease')
                        n.realtime_heartbeat_pub.publish(String(data=client))
                else:
                    n.realtime_command(b)
                return self.json({'ok': True})
            if what == 'teleop':
                n.teleop(b.get('vx', 0), b.get('vy', 0), b.get('wz', 0))
                return self.json({'ok': True})
            if what == 'estop':
                return self.json({'ok': True, 'dev_stop': n.estop()})
            if what == 'initialpose':
                request_id=n.initialpose(b['x'],b['y'],b['yaw'])
                return self.json({'ok':True,'request_id':request_id,'state':'pending'})
            if what == 'evidence_ttl':
                n.set_evidence_ttl(b['dynamic_s'], b['static_s'])
                return self.json({'ok': True})
            if what == 'goal' and n.mode == 'realtime':
                n.realtime_command(dict(action='point', x=b['x'], y=b['y'], yaw=b.get('yaw'), client=b.get('client','')))
                return self.json({'ok': True})
            if what == 'goal':
                if n.alignment.state in ('pending','failed'):
                    raise PermissionError('等待初始定位完成后再发送目标')
                n.goal_pub.publish(n.pose_msg(b['x'], b['y'], b.get('yaw'), PoseStamped))
                n.note('WEB_GOAL', x_m=float(b['x']), y_m=float(b['y']))
                return self.json({'ok': True})
            if what == 'task':
                if b['command'] in ('run_once','run_loop') and n.alignment.state in ('pending','failed'):
                    raise PermissionError('等待初始定位完成后再执行任务')
                if n.mode == 'realtime':
                    if b['command'] != 'cancel':
                        raise ValueError('use realtime controls for this mode')
                    n.realtime_command({'action':'pause'})
                    return self.json({'ok': True})
                if b['command'] not in TASK_COMMANDS:
                    raise ValueError(f'command {b["command"]!r} not allowed here')
                arg = wl.check_name(b['arg']) if b['command'] == 'load' else ''
                n.send_cmd(b['command'], arg)
                return self.json({'ok': True})
            if what == 'mission' and len(parts) == 3:
                from go2_task.task_logic import save_mission
                name = wl.check_name(parts[2])
                rows = wl.mission_rows(b['waypoints'])
                save_mission(n.missions_dir / f'{name}.yaml', _waypoints(rows, n))
                n.note('WEB_MISSION_SAVED', mission=name, waypoints=len(rows))
                return self.json({'ok': True, 'name': name})
            if what == 'check':
                return self.json({'problems': n.check_waypoints(b['waypoints'])})
            if what == 'map' and len(parts) == 4:
                d = n.maps_dir / wl.check_name(parts[2])
                if parts[3] == 'edit':
                    backup = wl.save_edits(d, b['edits'])
                    n.note('WEB_MAP_EDIT', map=parts[2], edits=len(b['edits']), backup=backup.name)
                    return self.json({'ok': True, 'backup': backup.name, 'reload_needed': parts[2] == n.loaded_map})
                if parts[3] == 'restore':
                    name = wl.restore_last(d)
                    n.note('WEB_MAP_RESTORE', map=parts[2], backup=name)
                    return self.json({'ok': True, 'restored': name, 'reload_needed': parts[2] == n.loaded_map})
            if what == 'map' and parts[2:] == ['finish']:
                raise ValueError('fixed-map simulation does not create SLAM maps')
            if what == 'stack' and parts[2:] == ['start']:
                return self.json({'ok': True, 'pid': n.start_stack(b['mode'], b.get('map', ''), b.get('enable_motion', False))})
            if what == 'stack' and parts[2:] == ['stop']:
                return self.json({'ok': True, 'was_running': n.stop_stack()})
            if what == 'route' and parts[2:] == ['mark']:
                if n.mode != 'map' or n.pose is None:
                    raise PermissionError('marks are only recorded while mapping')
                with n.lock:
                    n.marks.append([n.pose[0], n.pose[1]])
                    return self.json({'ok': True, 'marks': len(n.marks)})
            if what == 'route' and len(parts) == 4 and parts[3] == 'delete':
                f = n.maps_dir / wl.check_name(b['map']) / 'routes' / f'{wl.check_name(parts[2])}.yaml'
                f.unlink()
                n.note('WEB_ROUTE_DELETED', map=b['map'], route=parts[2])
                return self.json({'ok': True})
            return self.fail(404, 'not found')
        except PermissionError as e:
            return self.fail(409, e)
        except (OSError, ValueError, KeyError, RuntimeError, TypeError) as e:
            return self.fail(400, f'{type(e).__name__}: {e}')


def _waypoints(rows, node):
    from go2_task.task_logic import Waypoint
    default = node.geometry.arrive_radius_m if node.geometry else 0.5
    return [Waypoint(r['name'], r['x'], r['y'], r.get('radius_m', default), r.get('yaw'), r.get('path')) for r in rows]


def main():
    rclpy.init()
    node = None
    server = None
    try:
        node = WebNode()
        Handler.node = node
        server = ThreadingHTTPServer((node.host, node.port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.note('WEB_LISTEN', url=f'http://{node.host}:{node.port}/')
        # ONE thread: the MultiThreadedExecutor's thread hand-offs cost a third more CPU on top of rclpy's per-message wait-set rebuild
        # (desk bench 2026-11-09: 96 % -> 60 % of a core with a 50 Hz topic); the HTTP handlers run in their own threads anyway and
        # only read what the callbacks stored, a future from a service call is resolved by this thread while a handler waits.
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and node.stack is not None and node.stack.alive():
            node.stack.stop()                              # the stack never outlives its console (the dog stops getting commands anyway)
            try:
                node.stack.wait_stopped(timeout_s=node.stack.kill_after_s + 10)
            except Exception:
                pass
        if server is not None:
            server.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
