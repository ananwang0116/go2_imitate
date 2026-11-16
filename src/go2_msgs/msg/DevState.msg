# /dev_state: the dog's state from dev_bridge (replaces /bridge_status + /bridge_sport_state, which were
# index-addressed Float64MultiArray arrays and were misread once). Named fields only.
std_msgs/Header header

# health
int32 error_code                      # SportModeState.error_code; 100 is this dog's normal idle value (whitelisted)
float64 state_age_s                   # age of the newest rt/sportmodestate sample
bool fault                            # true = the dog must not be driven
string fault_reason                   # empty when fault == false
float64 battery_v
float64 max_motor_temp_c

# the dog's own estimate (independent reference for Leg-KILO; never used for localisation)
uint8 gait_type
uint8 mode
float64 body_height_m
float64[3] position_m
float64[3] velocity_mps
float64 yaw_speed_radps
float64 roll_rad
float64 pitch_rad
float64 yaw_rad
float64[4] foot_force                 # raw units; always 0 on this dog so far
