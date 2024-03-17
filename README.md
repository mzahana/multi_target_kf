# multi_target_kf
A ROS 2 package with an implementation of a linear Kalman filter for multi-target state estimation.

# Dependencies
* Eigen3
* Some ROS packages:  `std_msgs, geometry_msgs, sensor_msgs`

# Usage
* Configure the Kalman filter parameters in the `config/kf_param.yaml` file
* Run the filter using
  ```bash
  ros2 launch multi_target_kf kf_const_vel.launch.py 
  ```
* The node subscribes to measurements (of one or multiple targets) to the topic `measurement/pose_array`, type of msg is `geometry_msgs/PoseArray`
* Estimated states are published in two topics
  * `/kf/good_tracks` positions and velocities (custom KF message `multi_target_kf::msg::KFTracks`)
  * `/kf/good_tracks_pose_array` positions to visulaize in RViz 2 `geometry_msgs::msg::PoseArray`

