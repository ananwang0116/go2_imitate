# /task_status from task_executor.
std_msgs/Header header
string mission                        # mission name, "" for a single goal
string mode                           # idle | collecting | once | loop
int32 waypoint_index                  # -1 when idle
int32 waypoint_count
string waypoint_state                 # PENDING | APPROACH | RECOVER | ARRIVED | SKIPPED
float64 budget_used_s
int32 tries
float64 remaining_m                   # along the path, NaN when unknown
