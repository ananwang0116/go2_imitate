# /mot_nav_status from mot_nav (10 Hz): what it is doing for the goal `id`.
uint8 IDLE=0
uint8 FOLLOWING=1
uint8 ARRIVED=2
uint8 FAILED=3

std_msgs/Header header
string id
uint8 state
string reason                         # FAILED / waiting: no_map | no_pose | no_path | blocked | goal_blocked | start_blocked | ""
float64 remaining_m                   # along the path, NaN when there is no path
float64 clearance_m                   # distance from the body centre to the nearest wall / obstacle, NaN unknown
float64 plan_age_s                    # age of the path being followed, NaN when none
