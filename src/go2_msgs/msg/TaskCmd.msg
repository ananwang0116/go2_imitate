# /task_cmd from task_cli to task_executor. task_cli only sends; every judgement is in task_executor (spec 6.5).
string COLLECT="collect"              # start collecting waypoints from /goal_pose
string RUN_ONCE="run_once"
string RUN_LOOP="run_loop"
string UNDO="undo"                    # drop the last collected waypoint
string CLEAR="clear"
string CANCEL="cancel"                # stop the current task (the dog stops; the remote is the e-stop)
string SAVE="save"                    # arg = mission name
string LOAD="load"                    # arg = mission name

std_msgs/Header header
string command
string arg
