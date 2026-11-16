# /mot_nav_goal from task_executor to mot_nav (replaces the Nav2 ComputePathToPose / FollowPath actions).
# One goal at a time. Re-send it about once a second as a keep-alive: mot_nav treats a goal that was not refreshed for
# goal_timeout_s as cancelled (the executor died -> the dog stops). A new `id` forces a fresh plan; active=false cancels.
#
# Two kinds of goal:
#   point goal  (follow_path=false): plan with A* from the dog to (x, y).
#   path goal   (follow_path=true):  follow the taught polyline path_x/path_y (map frame) as it is: no A* on it. A far-off dog
#               first joins it (A* to the nearest path point); a blocked stretch is detoured (A* to a later path point).
#               (x, y) is the path's last point.
# After arriving (body centre within radius_m), use_yaw=true makes mot_nav turn on the spot to `yaw` and only then report ARRIVED.
std_msgs/Header header
string id
bool active
float64 x                             # frame map [m]
float64 y
float64 radius_m                      # arrived = body centre within this radius
float64 speed_pct                     # 100 = speeds.yaml limits; the executor lowers it while localisation is DEGRADED
bool use_yaw
float64 yaw                           # frame map [rad], only when use_yaw
bool follow_path
float64[] path_x
float64[] path_y
