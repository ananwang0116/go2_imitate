# /loc_status from loc_pose: the single judge of "is the localisation trustworthy" (spec 4).
uint8 OK=0
uint8 DEGRADED=1
uint8 LOST=2

std_msgs/Header header
uint8 state
string reason                         # why not OK; empty when OK
float64 kilo_age_s                    # age of the newest /kilo_odom
float64 map_score                     # last /loc_map_score, NaN when unknown
