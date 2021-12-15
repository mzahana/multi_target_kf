# multi_target_kf
A ROS package with an implementation of a linear Kalman filter for multi-target state estimation.
# Dependencies
* Eigen3
* Some ROS packages:  `std_msgs, geometry_msgs, sensor_msgs`

# Usage
* Configure the Kalman filter parameters in the `config/kf_tracker.yaml` file
* Run the filter using
  ```bash
  roslaunch multi_target_kf kf_tracker.launch 
  ```
* Publish measurements (of one or multiple targets) to the topic `measurement/pose_array`, type of msg is `geometry_msgs/PoseArray`
* Estimated states are published in two topics
  * `/kf/tracks` positions and velocities
  * `/kf/tracks_pose_array` positions
