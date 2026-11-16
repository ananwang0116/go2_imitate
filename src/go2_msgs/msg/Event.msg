# One decision or state change (spec P7 / section 8). Published on /<layer>_event and appended to <run>/events.log.
# Text form: "[<layer>] <name> key=value key=value"
std_msgs/Header header
string layer                          # dev | loc | per | mot | safe | task | run
string name                           # upper-case event name, e.g. LIMIT, SKIP, START
diagnostic_msgs/KeyValue[] fields     # key = value pairs, keys carry units (gap_m, used_s)
