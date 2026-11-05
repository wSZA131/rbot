# Estimated map location of a detected object.

std_msgs/Header header

# Detector class label, normally the task target string.
string label

# Pose in the map frame.
geometry_msgs/PoseStamped map_pose

# Detector confidence of the contributing observation.
float32 confidence

# Search region the observation was made from.
string region

# 3x3 row-major position covariance in m^2.
float32[9] position_covariance
