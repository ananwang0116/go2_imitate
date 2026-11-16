# Physical test: launch with test/trot_validation.world, then run this script.
# Independent Gazebo state is used only for validation.
import rclpy,json,math,time,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray,UInt8MultiArray
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('trot_validation');pub=n.create_publisher(Twist,'/cmd_vel',10)
t=0;base=None;rows=[];status=[];contacts=[];joints=[];start=time.monotonic()
def clock(m):
 global t,base
 raw=m.clock.sec+m.clock.nanosec/1e9
 if base is None:base=raw
 t=raw-base
n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
def state(m):
 if 'go2' not in m.name:return
 i=m.name.index('go2');p=m.pose[i];v=m.twist[i];q=p.orientation
 yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
 rows.append([t,p.position.x,p.position.y,p.position.z,roll,pitch,yaw,v.linear.x,v.linear.y,v.angular.z])
n.create_subscription(ModelStates,'/validation/model_states',state,qos_profile_sensor_data)
n.create_subscription(Float64MultiArray,'/go2/balance_status',lambda m:status.append([t]+list(m.data)),10)
n.create_subscription(UInt8MultiArray,'/go2/actual_contacts',lambda m:contacts.append([t]+list(m.data)),10)
n.create_subscription(JointState,'/joint_states',lambda m:joints.append([t]+list(m.velocity)+list(m.effort)),qos_profile_sensor_data)
last=-1
while rclpy.ok() and time.monotonic()-start<240:
 rclpy.spin_once(n,timeout_sec=.02)
 if t-last>.05:
  m=Twist()
  if 12<t<22:m.linear.x=.04
  if 25<t<35:m.linear.x=.14
  if 38<t<48:m.angular.z=.3
  if 52<t<62:m.angular.z=1.0
  if 66<t<76:m.linear.x=.14
  if 82<t<92:m.angular.z=-1.0
  if 96<t<104:m.linear.x=.14
  if t<104:pub.publish(m)
  last=t
 if t>110:break
json.dump(dict(rows=rows,status=status,contacts=contacts,joints=joints),open(sys.argv[1] if len(sys.argv)>1 else '/tmp/trot-results.json','w'))
print('DONE',t,len(rows),len(status));n.destroy_node();rclpy.shutdown()
