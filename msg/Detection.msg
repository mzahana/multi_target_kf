std_msgs/Header header

# Object identification
int64 id                    # Unique object ID
string class_name          # Object class (e.g., "person", "car", "bicycle")
float64 confidence         # Detection confidence [0.0, 1.0]

# 3D Position
geometry_msgs/Point position
geometry_msgs/Vector3 position_covariance  # Diagonal elements of position covariance

# Linear velocity (optional - may not always be available)
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 linear_velocity_covariance  # Diagonal elements

# Angular velocity (optional)
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_velocity_covariance  # Diagonal elements

# Linear acceleration (optional)
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 linear_acceleration_covariance  # Diagonal elements

# 2D Bounding Box (YOLO format: center + size)
bool has_2d_bbox
float64 bbox_2d_center_x    # Center x coordinate in image
float64 bbox_2d_center_y    # Center y coordinate in image
float64 bbox_2d_width       # Width of bounding box
float64 bbox_2d_height      # Height of bounding box

# 3D Bounding Box (center + size + orientation)
bool has_3d_bbox
geometry_msgs/Point bbox_3d_center     # Center of 3D bounding box
geometry_msgs/Vector3 bbox_3d_size     # Width, height, depth
geometry_msgs/Quaternion bbox_3d_orientation  # Orientation of the bounding box

# Additional metadata
string[] attributes        # Additional object attributes (e.g., "moving", "stationary")