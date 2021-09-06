# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

int64[] x
int64[] y
float32[] depth
float32[] flow_x
float32[] flow_y
float32[] delta_depth
float32[] covariance_diag