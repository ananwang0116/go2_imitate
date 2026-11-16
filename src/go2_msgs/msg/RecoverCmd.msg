# /mot_recover_cmd from task_executor to mot_recover (spec 6.3 recovery ladder). Result: event [mot] RECOVER_DONE id=.. result=..
string BACKUP="backup"                # value = distance [m] to back up (positive)
string TURN="turn"                    # value = relative turn [rad] (+ = left)
string TURN_TO="turn_to"              # value = absolute yaw [rad] in frame odom
string STOP="stop"                    # abort the running motion
std_msgs/Header header
string id                             # chosen by the requester, echoed in the result event
string action
float64 value
