"""Every topic, frame and service name of go2_v2 (spec 7). Nodes import names from here; no name is typed twice.

Prefix = layer (dev_ loc_ per_ mot_ safe_ task_ viz_ kilo_), suffix = data type.
ROS conventional names (/tf, /map, /initialpose, /goal_pose, /joint_states) are kept as they are.
"""

LAYERS = ('dev', 'loc', 'per', 'mot', 'safe', 'task', 'run')

# L0 dev
DEV_IMU = '/dev_imu'
DEV_LIDAR_CLOUD = '/dev_lidar_cloud'
DEV_STATE = '/dev_state'
DEV_JOINT_STATES = '/joint_states'           # ROS convention, robot_state_publisher reads it
DEV_TELEOP_CMD_VEL = '/dev_teleop_cmd_vel'
DEV_SERVICES = ('/dev_stand_up', '/dev_stand_down', '/dev_damp', '/dev_recovery', '/dev_stop')

# L1 loc
KILO_ODOM = '/kilo_odom'                     # Leg-KILO, external, not renamed
LOC_ODOM = '/loc_odom'
LOC_SLAM_ODOM = '/loc_slam_odom'            # normalised SLAM output (base pose at our start origin); /kilo_odom remains a compatibility alias
LOC_STATUS = '/loc_status'
LOC_MAP_SCORE = '/loc_map_score'
INITIALPOSE = '/initialpose'                 # initial position hint (web console) -> loc_align

# L2 per
PER_OBSTACLE_SCAN = '/per_obstacle_scan'
PER_MAP_SCAN = '/per_map_scan'
PER_FREE_SCAN = '/per_free_scan'             # per bin: how far the laser saw FLOOR (proven free)
PER_EVIDENCE_MAP = '/per_evidence_map'       # pure evidence: -1 unseen, 0 proven free, 100 proven occupied
PER_NAV_MAP = '/per_nav_map'                 # prior map fused with evidence, the costmap input of mot_nav
PER_EVIDENCE_TTL_CMD = '/per_evidence_ttl_cmd'   # web -> per_evidence (std_msgs/String JSON): how long dynamic / static obstacles stay
PER_EVIDENCE_TTL = '/per_evidence_ttl'           # per_evidence -> web (String JSON, transient-local): the lifetimes in force
PER_HARD_MAP = '/per_hard_map'               # walls drawn by hand in the map editor (maps/<name>/hard.pgm): 100 = never passable

# L3 mot
MOT_NAV_CMD_VEL = '/mot_nav_cmd_vel'         # mot_nav output (go2_nav), into mot_cmd_mux
MOT_RECOVER_CMD_VEL = '/mot_recover_cmd_vel'
MOT_CMD_VEL = '/mot_cmd_vel'                 # mux output: teleop > recover > nav
MOT_RECOVER_CMD = '/mot_recover_cmd'         # go2_msgs/RecoverCmd, task_executor -> mot_recover
MOT_NAV_GOAL = '/mot_nav_goal'               # go2_msgs/NavGoal, task_executor -> mot_nav (own planner + follower, C++, go2_nav)
MOT_NAV_STATUS = '/mot_nav_status'           # go2_msgs/NavStatus, mot_nav -> task_executor (10 Hz)
MOT_NAV_CHECK = '/mot_nav_check'             # go2_msgs/srv/NavCheck: reachability asked of mot_nav's own planner
MOT_NAV_PATH = '/mot_nav_path'               # nav_msgs/Path, the path mot_nav follows (map frame), for the web console

# L4 safe
SAFE_CMD_VEL = '/safe_cmd_vel'               # the only velocity dev_bridge listens to
SAFE_STATUS = '/safe_status'

# L5 task
GOAL_POSE = '/goal_pose'                     # a one-point goal (web console) -> task_executor
TASK_CMD = '/task_cmd'
TASK_STATUS = '/task_status'
VIZ_TASK_MARKERS = '/viz_task_markers'

# frames
FRAME_MAP = 'map'
FRAME_ODOM = 'odom'
FRAME_BASE_FOOTPRINT = 'base_footprint'
FRAME_BASE = 'base'
FRAME_MID360 = 'mid360_link'                 # the lidar's own frame: clouds stay in it, TF base -> mid360_link comes from geometry.yaml


def event_topic(layer: str) -> str:
    if layer not in LAYERS:
        raise ValueError(f'unknown layer {layer!r}, expected one of {LAYERS}')
    return f'/{layer}_event'


EVENT_TOPICS = tuple(event_topic(layer) for layer in LAYERS)
