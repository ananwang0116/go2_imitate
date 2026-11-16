"""Frontier candidate selector and UI intent adapter, above task_executor.

Only task_executor publishes /mot_nav_goal. This coordinator asks mot_nav_check
before sending one /goal_pose. Control leases and data health use a steady clock;
scan poses use exact stamp TF. A fixed map frame is provided by SimBridge.
"""
import json
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from go2_msgs.msg import TaskCmd, TaskStatus, LocStatus, Event
from go2_msgs.srv import NavCheck
from go2_common.node import Go2Node, CMD_QOS, EVENT_QOS, TASK_CMD_QOS, spin
from go2_sim.realtime_logic import ObservedMap, reachable_frontiers, known_goal, ClientLease

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Realtime(Go2Node):
    def __init__(self):
        super().__init__('sim_realtime', 'run')
        self.localisation=self.declare_parameter('localisation','legkilo').value
        self.map = ObservedMap()
        self.observed = self.map.grid()
        self.nav_data = np.full(self.observed.shape,100,np.int8)
        self.nav_at = -1e9
        self.pose = None
        self.scan_at = self.loc_at = self.task_at = -1e9
        self.loc_ok = False
        self.task = None
        self.mode, self.phase, self.reason = 'manual', 'idle', 'ready_for_observation'
        self.error = ''
        self.goal = None
        self.frontiers = []
        self.blacklist = []
        self.pending = None
        self.future = None
        self.lease = ClientLease()
        self.goal_at = self.next_select = 0.
        self.accepted_task = False
        self.saved = ''
        # Conservative circumscribed footprint for candidates only; the original
        # planner remains the authority for actual route/footprint feasibility.
        g = self.geometry
        self.radius = math.hypot(g.body_half_length_m, g.body_half_width_m)+g.safe_stop_gap_m
        self.map_command = self.create_publisher(String, '/sim/map_command', CMD_QOS)
        self.status_pub = self.create_publisher(String, '/sim/realtime_status', CMD_QOS)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', CMD_QOS)
        self.cancel_pub = self.create_publisher(TaskCmd, '/task_cmd', TASK_CMD_QOS)
        self.check = self.create_client(NavCheck, '/mot_nav_check')
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, '/per_nav_map', self.on_nav_map, MAP_QOS)
        self.create_subscription(String, '/sim/mapping_status', self.on_mapping_status, CMD_QOS)
        self.create_subscription(LocStatus, '/loc_status', self.on_loc, CMD_QOS)
        self.create_subscription(TaskStatus, '/task_status', self.on_task, CMD_QOS)
        self.create_subscription(Event, '/task_event', self.on_event, EVENT_QOS)
        self.create_subscription(String, '/sim/realtime_cmd', self.on_command, CMD_QOS)
        self.create_subscription(String, '/sim/realtime_heartbeat', self.on_heartbeat, CMD_QOS)
        # Manual input from any console interrupts autonomous task selection.
        self.create_subscription(Twist, '/dev_teleop_cmd_vel', self.on_teleop, CMD_QOS)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event('REALTIME_START', mapping='observed_only',localisation=self.localisation, unknown='blocked', lease_s=3.)

    def on_map(self, m):
        if m.info.width != self.map.w or m.info.height != self.map.h:
            return
        self.observed = np.asarray(m.data,np.int8).reshape(self.map.h,self.map.w)

    def on_nav_map(self, m):
        if m.info.width != self.map.w or m.info.height != self.map.h:
            return
        self.nav_data = np.asarray(m.data,np.int8).reshape(self.map.h,self.map.w)
        self.nav_at = time.monotonic()

    def on_mapping_status(self, m):
        data=json.loads(m.data)
        self.pose=data['pose']
        self.scan_at=time.monotonic()-data['scan_age_s']
        self.map.rev=data['map_rev']
        self.saved=data['saved']

    def on_loc(self, m):
        self.loc_ok = m.state == LocStatus.OK
        self.loc_at = time.monotonic()

    def on_task(self, m):
        self.task, self.task_at = m, time.monotonic()
        if self.goal is not None and m.mode in ('once','loop'):
            self.accepted_task = True

    def on_event(self, m):
        # Events from before dispatch must never complete a new goal.
        stamp = m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        if self.goal is None or stamp < self.dispatch_stamp:
            return
        if m.name == 'ROUND_DONE':
            result = {f.key:f.value for f in m.fields}
            arrived = result.get('arrived') == '1'
            self.finish_goal(arrived, 'arrived' if arrived else 'task_failed')

    def on_heartbeat(self, m):
        # This is a UI lease, separate from ROS navigation goal keepalives.
        self.lease.heartbeat(m.data, time.monotonic())

    def on_teleop(self, m):
        if (m.linear.x or m.linear.y or m.angular.z) and self.mode != 'manual':
            self.stop('manual_takeover', 'manual')

    def stop(self, reason, mode='paused'):
        self.lease.owner = None
        self.goal = self.pending = None
        if self.future is not None:
            self.future.cancel()
            self.future = None
        self.mode, self.phase, self.reason = mode, 'idle', reason
        cmd = TaskCmd(command='cancel')
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cancel_pub.publish(cmd)
        self.event('REALTIME_STOP', reason=reason, mode=mode)

    def healthy(self):
        now = time.monotonic()
        return (self.pose is not None and self.loc_ok and now-self.loc_at<1. and
                now-self.scan_at<1. and now-self.task_at<2. and now-self.nav_at<2.)

    def on_command(self, msg):
        try:
            b = json.loads(msg.data)
            action = b['action']
            self.error = ''
            if action in ('manual','pause','stop'):
                self.stop('user_'+action, 'manual' if action=='manual' else 'paused')
                return
            if action == 'save':
                self.map_command.publish(String(data='save'))
                return
            if action not in ('explore','point'):
                raise ValueError('unknown realtime action')
            if not self.healthy():
                raise ValueError('waiting_for_fresh_pose_scan_task')
            client = str(b.get('client',''))
            if not client or len(client)>80:
                raise ValueError('explicit_control_client_required')
            if not self.lease.fresh(client, time.monotonic()):
                raise ValueError('web_heartbeat_missing')
            if action == 'point':
                xy = (float(b['x']),float(b['y']))
                yaw = None if b.get('yaw') is None else float(b['yaw'])
                if not all(math.isfinite(v) for v in (*xy, *((yaw,) if yaw is not None else ()))):
                    raise ValueError('finite_target_and_heading_required')
                if not known_goal(self.nav(), self.map.res, self.map.origin, xy, self.radius):
                    raise ValueError('target_not_known_free_or_too_close_to_boundary')
            self.stop('switch_to_'+action, action)
            self.lease.owner = client
            self.next_select = time.monotonic()+.6  # allow old task cancellation to be processed
            self.phase, self.reason = 'selecting', 'checking_known_reachable_goal'
            if action == 'point':
                self.pending = {'x':xy[0],'y':xy[1],'yaw':yaw}
        except (ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)
            self.event('REALTIME_REJECT', reason=self.error)

    def nav(self):
        return self.nav_data

    def choose(self):
        self.blacklist=[(xy,until) for xy,until in self.blacklist if until>time.monotonic()]
        self.frontiers=reachable_frontiers(self.observed,self.nav(),self.map.res,self.map.origin,
                                          self.pose,self.radius,[xy for xy,_ in self.blacklist])[:12]
        if not self.frontiers:
            # Absence of reachable safe frontiers is not proof of full-world coverage.
            self.mode,self.phase,self.reason='blocked','idle','no_reachable_frontier'
            self.event('FRONTIER_EMPTY', reason=self.reason)
            return
        self.pending=dict(self.frontiers[0])
        self.event('FRONTIER_SELECTED', x=self.pending['x'], y=self.pending['y'], count=len(self.frontiers))

    def start_check(self):
        if not self.check.service_is_ready():
            self.reason='waiting_for_planner_check'
            return
        req=NavCheck.Request()
        req.x=[self.pending['x']];req.y=[self.pending['y']]
        req.radius_m=[float(self.geometry.arrive_radius_m)]
        req.use_start=True;req.start_x=self.pose[0];req.start_y=self.pose[1]
        self.future=self.check.call_async(req)
        self.check_at=time.monotonic();self.phase='checking'

    def finish_goal(self, ok, reason):
        if self.goal is not None:
            self.blacklist.append(((self.goal['x'],self.goal['y']),time.monotonic()+(15. if ok else 45.)))
        self.event('REALTIME_GOAL_DONE', ok=ok, reason=reason)
        self.goal=None
        self.phase='selecting' if self.mode=='explore' else 'idle'
        self.reason=reason
        if self.mode=='point':
            self.mode='complete' if ok else 'blocked'
        self.next_select=time.monotonic()+1.

    def dispatch(self):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=self.pending['x'];m.pose.position.y=self.pending['y']
        yaw=self.pending.get('yaw')
        if yaw is not None:
            m.pose.orientation.z=math.sin(yaw/2)
            m.pose.orientation.w=math.cos(yaw/2)
        # Otherwise retain task_executor's all-zero no-final-heading convention.
        self.goal=dict(self.pending);self.pending=None
        self.dispatch_stamp=self.get_clock().now().nanoseconds
        self.goal_pub.publish(m)
        self.accepted_task=False;self.goal_at=time.monotonic()
        self.phase,self.reason='navigating','planner_approved'
        self.event('REALTIME_GOAL', mode=self.mode,x=self.goal['x'],y=self.goal['y'])

    def tick(self):
        now=time.monotonic()
        if self.mode in ('point','explore'):
            if not self.lease.valid(now):
                self.stop('web_lease_expired')
            elif not self.healthy():
                self.stop('pose_scan_or_task_stale')
            elif self.goal is not None:
                if not self.accepted_task and now-self.goal_at>5.:
                    old=self.goal
                    self.stop('task_accept_timeout', 'explore' if self.mode=='explore' else 'blocked')
                    self.blacklist.append(((old['x'],old['y']),now+45.))
                    self.next_select=now+1.
            elif self.future is not None:
                if self.future.done():
                    response=self.future.result();self.future=None
                    if response and response.available and response.ok and response.ok[0]:
                        # Recheck current observation; the map may have changed while awaiting the planner.
                        if known_goal(self.nav(),self.map.res,self.map.origin,
                                      (self.pending['x'],self.pending['y']),self.radius):
                            self.dispatch()
                        else:
                            self.reject_pending('goal_changed_during_check')
                    else:
                        why=response.reason[0] if response and response.reason else 'planner_check_unavailable'
                        self.reject_pending(why)
                elif now-self.check_at>3.:
                    self.future.cancel();self.future=None
                    self.reject_pending('planner_check_timeout')
            elif now>=self.next_select:
                if self.pending is None and self.mode=='explore':
                    self.choose()
                if self.pending is not None:
                    self.start_check()
        self.publish_status()

    def reject_pending(self, reason):
        self.blacklist.append(((self.pending['x'],self.pending['y']),time.monotonic()+45.))
        self.pending=None;self.reason=reason
        self.next_select=time.monotonic()+1.
        self.phase='selecting' if self.mode=='explore' else 'idle'
        if self.mode=='point':
            self.mode='blocked';self.error=reason
        self.event('FRONTIER_REJECT', reason=reason)

    def publish_status(self):
        grid=self.observed;js,ins=np.where(grid>=0)
        bounds=None if not len(js) else [float(self.map.origin[0]+ins.min()*self.map.res),
            float(self.map.origin[1]+js.min()*self.map.res),float(self.map.origin[0]+(ins.max()+1)*self.map.res),
            float(self.map.origin[1]+(js.max()+1)*self.map.res)]
        data=dict(mode=self.mode,phase=self.phase,reason=self.reason,error=self.error,goal=self.goal,
                  frontiers=self.frontiers,known_cells=int((grid>=0).sum()),free_cells=int((grid==0).sum()),
                  observed_m2=float((grid>=0).sum()*self.map.res**2),map_rev=self.map.rev,
                  bounds=bounds,width=self.map.w,height=self.map.h,resolution=self.map.res,
                  origin=self.map.origin.tolist(),lease_ok=self.lease.valid(time.monotonic()),
                  healthy=self.healthy(),control_client=self.lease.owner,saved=self.saved,localisation=self.localisation)
        self.status_pub.publish(String(data=json.dumps(data)))



def main():
    spin(Realtime)
