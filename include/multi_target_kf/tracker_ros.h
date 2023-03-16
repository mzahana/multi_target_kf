#ifndef TRACKER_ROS_H
#define TRACKER_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <apriltag_ros/AprilTagDetectionArray.h>
#include "multi_target_kf/KFTrack.h" // @todo Need to define these custom messages
#include "multi_target_kf/KFTracks.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "multi_target_kf/kf_tracker.h"

using namespace std::chrono_literals;


class TrackerROS : public rclcpp::Node
{
public:
    TrackerROS(/* args */);
    ~TrackerROS();
private:
    std::string target_frameid_; /**< Target frame name which will be post-fixed by the target unique number e.g. "tag", post-fixed will be like "tag1" */
    tf::TransformListener tf_listener_; /**< TF listener for measurements */
    bool listen_tf_; /**< listens to TF to find transofrms of received measurements w.r.t. tracking_frame_ */
    KFTracker kf_tracker_;

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
   void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

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
   void filterLoop(double t);

};

TrackerROS::TrackerROS(/* args */): Node("tracker_ros")
{
    nh_private_.param<double>("dt_pred", kf_tracker_.dt_pred_, 0.05);
   nh_private_.param<double>("V_max", kf_tracker_.V_max_, 20.0);
   nh_private_.param<double>("V_certain", kf_tracker_.V_certain_, 1.0);
   nh_private_.param<int>("N_meas", kf_tracker_.N_meas_, 20);
   nh_private_.param<double>("l_threshold", kf_tracker_.l_threshold_, -2.0);
   int buff_size;
   nh_private_.param<int>("state_buffer_length", kf_tracker_.buff_size, 40);
   kf_tracker_.state_buffer_size_  = (unsigned int) buff_size;
   RCLCPP_INFO(this->get_logger(),"State buffer length corresponds to %f seconds",
                kf_tracker_.dt_pred_*(double)kf_tracker_.state_buffer_size_);
   nh_private.param<bool>("do_kf_update_step", kf_tracker_.do_update_step_, true);
   nh_private.param<double>("measurement_off_time", kf_tracker_.measurement_off_time_, 2.0);
   nh_private.param<bool>("print_debug_msg", kf_tracker_.debug_, false);
   nh_private.param<std::string>("tracking_frame", kf_tracker_.tracking_frame_, "map");
   nh_private.param<std::string>("target_frameid", target_frameid_, "tag");
   nh_private.param<bool>("listen_tf", listen_tf_, true);
   nh_private.param<bool>("use_track_id", kf_tracker_.use_track_id_, true);
   nh_private.param<double>("dist_threshold", kf_tracker_.dist_threshold_, 2.0);
   nh_private.param<double>("sigma_a", kf_tracker_.sigma_a_, 1.0);
   nh_private.param<double>("sigma_p", kf_tracker_.sigma_p_, 1.0);
   nh_private.param<double>("sigma_v", kf_tracker_.sigma_v_, 1.0);
   if(debug_){
      RCLCPP_INFO(this->get_logger(),"sigma_a: %f, sigma_p: %f, sigma_v: %f", sigma_a_, sigma_p_, sigma_v_);
   }


   if( !nh_private.getParam("q_diag",kf_tracker_. q_diag_) ){
      RCLCPP_ERROR(this->get_logger(),"Failed to get q_diag parameter");
      return;
   }


   if( !nh_private.getParam("r_diag", kf_tracker_.r_diag_) ){
      RCLCPP_ERROR(this->get_logger(),"Failed to get r_diag parameter");
      return;
   }

   kf_tracker_.last_measurement_t_ = ros::Time::now(); // should be double
   kf_tracker_.last_prediction_t_ = ros::Time::now(); // should be double

   RCLCPP_INFO(this->get_logger(),"Initializing Kalman filter...");
   if(!kf_tracker_.initKF()) 
   {
      RCLCPP_ERROR(this->get_logger(),"Could not initialize Kalman filter");
      return;
   }


   std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(kf_tracker_.dt_pred_));
   kf_loop_timer_ = this->create_wall_timer(
      duration_ms, std::bind(&KFTracker::filterLoop, this)); // Define timer for constant loop rate

   pose_array_sub_ = this->create_subscription<geometry_msgs::msg::msg::PoseArray>(
      "measurement/pose_array", 10, std::bind(&KFTracker::poseArrayCallback, this, _1));

   good_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/good_tracks_pose_array", 10);
   all_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/all_tracks_pose_array", 10);
   good_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/good_tracks", 10);
   all_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/all_tracks", 10);
}

TrackerROS::~TrackerROS()
{
}

void
const irobot_create_msgs::msg::InterfaceButtons::SharedPtr
TrackerROS::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
   // if(debug_)
   //    printf("[KFTracker::poseArrayCallback] Thred id: %s", std::this_thread::get_id());

   // measurement_set_mtx_.lock();

   kf_tracker_.measurement_set_.clear();
   
   // Sanity check
   if(msg->poses.empty())
   {
      if(debug_)
         RCLCPP_WARN(this->get_logger(), "[KFTracker::poseArrayCallback]: No measurements received.");

      // measurement_set_mtx_.unlock();
      return;
   }

   for (auto it = msg->poses.begin(); it != msg->poses.end(); it++)
   {
      sensor_measurement z;
      z.time_stamp = msg->header.stamp;
      z.id = 0;
      z.z = Eigen::MatrixXd::Zero(3,1); // 3, because it's always position only, for now!
      z.z(0) = (*it).position.x;
      z.z(1) = (*it).position.y;
      z.z(2) = (*it).position.z;
      auto nrm = z.z.norm();
      if(nrm> 10000.0  || isnan(nrm))
      {
         if(debug_)
            RCLCPP_WARN(this->get_logger(),"[poseArrayCallback] Measurement norm is very large %f", z.z.norm());
         continue;
      }
      measurement_set_.push_back(z);
   }
   if(debug_){
      RCLCPP_INFO(this->get_logger(),"[poseArrayCallback] Size of measurement_set_ : %lu", measurement_set_.size());
      RCLCPP_INFO(this->get_logger(),"[poseArrayCallback] Size of msg->poses : %lu", msg->poses.size());
   }

   // measurement_set_mtx_.unlock();

   return;

}

void
TrackerROS::publishCertainTracks(void)
{
   if(kf_tracker.certain_tracks_.empty()){
      if(debug_)
         RCLCPP_WARN(this->get_logger(),"[KFTracker::publishCertainTracks] certain_tracks_ is empty. No tracks to publish");
      return;
   }

   geometry_msgs::msg::Pose pose;
   geometry_msgs::PoseArray pose_array;
   multi_target_kf::KFTrack track_msg;
   multi_target_kf::KFTracks tracks_msg;
   pose_array.header.stamp = certain_tracks_[0].current_state.time_stamp;
   pose_array.header.frame_id = tracking_frame_;

   for (auto it = certain_tracks_.begin(); it != certain_tracks_.end(); it++)
   {
      pose.position.x = (*it).current_state.x[0];
      pose.position.y = (*it).current_state.x[1];
      pose.position.z = (*it).current_state.x[2];
      pose.orientation.w = 1.0;

      pose_array.poses.push_back(pose);

      track_msg.header.stamp = (*it).current_state.time_stamp;
      track_msg.header.frame_id = tracking_frame_;
      track_msg.id = (*it).id;
      track_msg.n = (*it).n;

      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];

      /* The following are model-dependent ! */
      
      track_msg.twist.twist.linear.x = (*it).current_state.x[3];
      track_msg.twist.twist.linear.y = (*it).current_state.x[4];
      track_msg.twist.twist.linear.z = (*it).current_state.x[5];

      // track_msg.accel.accel.linear.x = (*it).current_state.x[6];
      // track_msg.accel.accel.linear.y = (*it).current_state.x[7];
      // track_msg.accel.accel.linear.z = (*it).current_state.x[8];
      

      tracks_msg.tracks.push_back(track_msg);
   }

   good_poses_pub_.publish(pose_array);
   good_tracks_pub_.publish(tracks_msg);
   
   return;
}

void
TrackerROS::publishAllTracks(void)
{
   if(tracks_.empty()){
      if(debug_)
         RCLCPP_WARN(this->get_logger(),"[KFTracker::publishAllTracks] tracks_ is empty. No tracks to publish");
      return;
   }

   // RCLCPP_WARN(this->get_logger(),"[KFTracker::publishAllTracks] Number of all tracks: %d", tracks_.size());

   geometry_msgs::Pose pose;
   geometry_msgs::PoseArray pose_array;
   multi_target_kf::KFTrack track_msg;
   multi_target_kf::KFTracks tracks_msg;
   pose_array.header.stamp = tracks_[0].current_state.time_stamp;
   pose_array.header.frame_id = tracking_frame_;

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      pose.position.x = (*it).current_state.x[0];
      pose.position.y = (*it).current_state.x[1];
      pose.position.z = (*it).current_state.x[2];
      pose.orientation.w = 1.0;

      pose_array.poses.push_back(pose);

      track_msg.header.stamp = (*it).current_state.time_stamp;
      track_msg.header.frame_id = tracking_frame_;
      track_msg.id = (*it).id;
      track_msg.n = (*it).n;

      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];

      /* The following are model-dependent ! */
      
      // track_msg.twist.twist.linear.x = (*it).current_state.x[3];
      // track_msg.twist.twist.linear.y = (*it).current_state.x[4];
      // track_msg.twist.twist.linear.z = (*it).current_state.x[5];

      // track_msg.accel.accel.linear.x = (*it).current_state.x[6];
      // track_msg.accel.accel.linear.y = (*it).current_state.x[7];
      // track_msg.accel.accel.linear.z = (*it).current_state.x[8];
      

      tracks_msg.tracks.push_back(track_msg);
   }

   all_poses_pub_.publish(pose_array);
   all_tracks_pub_.publish(tracks_msg);
   
   return;
}

/**
 * @param t current time
*/
void
KFTracker::filterLoop(double t)
{
   if(debug_)
      printf("[KFTracker::filterLoop] inside filterLoop...");

   if(tracks_.empty())
   {
      /* Initialize tracks with current measurements. Then, return */
      if(debug_){
         printf("WARN [KFTracker::filterLoop] No tracks to update. Initializing tracks using current measurements.");
      }
      initTracks();

      return;
   }

   // Do prediction step for all tracks.
   // predictTracks();

   double dt = t - kf_tracker_.last_prediction_t_;
   predictTracks(dt);
   kf_tracker_.last_prediction_t_ = t;

   // Do correction step for all tracks using latest measurements.
   kf_tracker_.updateTracks(t);

   // Publish all available tracks
   publishAllTracks();

   // Extrack good tracks, and remove bad ones
   kf_tracker_.updateCertainTracks();

   // Publish state estimates pf ggod tracks as PoseArray, and as custom msg of array (KFTracks.msg) of pose with covariance
   publishCertainTracks();

   // Remove bad tracks
   kf_tracker_.removeUncertainTracks();

   return;
}


#endif