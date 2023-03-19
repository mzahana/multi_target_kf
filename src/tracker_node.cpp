#include "multi_target_kf/tracker_ros_test.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  TrackerROS *node = new TrackerROS();
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
  return 0;
}