#ifndef TRACKER_ROS_H
#define TRACKER_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "multi_target_kf/msg/kf_track.hpp"
#include "multi_target_kf/msg/kf_tracks.hpp"


#include "multi_target_kf/kf_tracker.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class TrackerROS : public rclcpp::Node
{
public:
    TrackerROS(/* args */);
    ~TrackerROS();
private:
    std::string target_frameid_; /**< Target frame name which will be post-fixed by the target unique number e.g. "tag", post-fixed will be like "tag1" */
    bool listen_tf_; /**< listens to TF to find transofrms of received measurements w.r.t. tracking_frame_ */
    KFTracker *kf_tracker_;

    rclcpp::TimerBase::SharedPtr kf_loop_timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr good_poses_pub_; /**< ROS publisher for KF estimated (certain) tracks positions */
    rclcpp::Publisher<multi_target_kf::msg::KFTracks>::SharedPtr good_tracks_pub_;/**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_poses_pub_;/**< ROS publisher for KF estimated  (ALL) tracks positions */
    rclcpp::Publisher<multi_target_kf::msg::KFTracks>::SharedPtr all_tracks_pub_;/**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
   
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;/**< Subscriber to measurments. */

    /**
    * @brief Measurement ROS Callback. Updates measurement_set_
    * 
    * @param msg Holds PoseArray measurement
    */
   void poseArrayCallback(const geometry_msgs::msg::PoseArray & msg);

   /**
    * @brief Apriltag detections ROS Callback. Updates measurement_set_
    * 
    * @param msg Holds apriltag_ros::AprilTagDetectionArray msg
    */
   // void apriltagsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);


   void publishCertainTracks(void);
   void publishAllTracks(void);

   /**
    * @brief Executes the KF predict and update steps.
    */
   void filterLoop(void);

};

/**
 * @brief Class definition
 */
TrackerROS::TrackerROS(/* args */): Node("tracker_ros")
{
   


   // Define timers
   std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(kf_tracker_->dt_pred_));
   kf_loop_timer_ = this->create_wall_timer(
      duration_ms, std::bind(&TrackerROS::filterLoop, this)); // Define timer for constant loop rate

   // Define subscribers
   pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "measurement/pose_array", 10, std::bind(&TrackerROS::poseArrayCallback, this, _1));
   
   // Define publishers
   good_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/good_tracks_pose_array", 10);
   all_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/all_tracks_pose_array", 10);
   good_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/good_tracks", 10);
   all_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/all_tracks", 10);
}

TrackerROS::~TrackerROS()
{
}

void
TrackerROS::poseArrayCallback(const geometry_msgs::msg::PoseArray & msg)
{
  

   return;

}

void
TrackerROS::publishCertainTracks(void)
{
   
   return;
}

void
TrackerROS::publishAllTracks(void)
{
   
   
   return;
}

/**
 * @brief 
*/
void
TrackerROS::filterLoop(void)
{
   

   return;
}


#endif