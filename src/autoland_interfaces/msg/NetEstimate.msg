std_msgs/Header header
bool valid
string plan_id
# Changes when the upstream coordinate/calibration track is reset.
uint32 tracking_epoch
string estimation_method
geometry_msgs/Pose pose
float64[36] covariance
# Absolute cosine between viewing ray and surveyed plane normal (1 = head-on).
float64 ray_plane_incidence
float64 score
string reason
