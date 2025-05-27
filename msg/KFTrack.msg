std_msgs/Header header

# Unique ID of track
int64 id

# Object classification
string class_name          # Object class
float64 confidence         # Average confidence over track lifetime

# Number of received measurements for this track
int64 n

# Last update timestamp
builtin_interfaces/Time last_measurement_time

# Track state estimates
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel

# 2D Bounding box (YOLO format: center + size)
bool has_2d_bbox
float64 bbox_2d_center_x
float64 bbox_2d_center_y
float64 bbox_2d_width
float64 bbox_2d_height

# 3D Bounding box (center + size + orientation)
bool has_3d_bbox
geometry_msgs/Point bbox_3d_center
geometry_msgs/Vector3 bbox_3d_size
geometry_msgs/Quaternion bbox_3d_orientation

# Track quality metrics
float64 track_score        # Overall track quality score
float64 position_uncertainty  # Position uncertainty magnitude

# Additional metadata
string[] attributes