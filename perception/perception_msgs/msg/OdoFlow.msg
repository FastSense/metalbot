# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# This message provides 2d optical flow in N points.
# Also depth and delta_depth in these N points.
# Also the diagonal of the covariance matrix (3N numbers)

# x y coordinates of measurement points. Length: N
int64[] x
int64[] y
# z coordinates of measurement points. Length: N
float32[] depth
# measurement: 2d flow (Length: 2N)
float32[] flow_x
float32[] flow_y
# measurement: delta depth (Length: N)
float32[] delta_depth
# Diagonal of the covariance matrix (Length: 3N)
float32[] covariance_diag