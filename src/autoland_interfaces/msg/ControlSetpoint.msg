std_msgs/Header header
string plan_id
bool valid
# True only when guiding toward a stored target during a vision gap.
bool target_held
# Age of the last accepted image observation; publishing does not renew it.
float64 target_age_s
bool terminal_ready
bool terminal_mode
bool target_passed
float64 forward_range_m
float64 time_to_target_s
float64 crossing_error_y_m
float64 crossing_error_z_m
float64 position_std_y_m
float64 position_std_z_m
float64 roll_rad
float64 pitch_rad
float64 yaw_enu_rad
float64 thrust
string reason
