Header header

# Unique ID of track (e.g. Apriltag)
int64 id

# Number of received measurements for this track
int64 n

# Track pose
geometry_msgs/PoseWithCovariance pose

# Track twist
geometry_msgs/TwistWithCovariance twist