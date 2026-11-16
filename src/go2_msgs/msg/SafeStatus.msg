# /safe_status from safe_gate: what was limited and why (spec 2, L4). safe_gate only limits, never creates motion.
std_msgs/Header header
bool limiting                         # true while any component of the command is reduced
string direction                      # front | back | left | right | turn | all | ""
string reason                         # obstacle | stale_scan | loc_lost | dev_fault | no_input | ""
float64 gap_m                         # free distance in the limited direction, NaN when unknown
float64 brake_distance_m              # distance needed to stop from the incoming speed
float64[3] cmd_in                     # vx_mps vy_mps wz_radps from /mot_cmd_vel
float64[3] cmd_out                    # vx_mps vy_mps wz_radps on /safe_cmd_vel
