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
    KFTracker *kf_tracker_;
    std::string target_frameid_; /**< Target frame name which will be post-fixed by the target unique number e.g. "tag", post-fixed will be like "tag1" */
    bool listen_tf_; /**< listens to TF to find transofrms of received measurements w.r.t. tracking_frame_ */

    rclcpp::TimerBase::SharedPtr kf_loop_timer_;
    rclcpp::TimerBase::SharedPtr params_timer_;

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

   /**
    * @brief Parameters timer's callback
   */
  void paramsTimerCallback(void);

};

/**
 * @brief Class definition
 */

TrackerROS::~TrackerROS()
{
   /* Destructor */
   delete kf_tracker_;
}

TrackerROS::TrackerROS(/* args */): Node("tracker_ros")
{
   kf_tracker_ = new KFTracker();

   this->declare_parameter("dt_pred", 0.05);
   kf_tracker_->dt_pred_ = this->get_parameter("dt_pred").get_parameter_value().get<double>();

   this->declare_parameter("V_max", 20.0);
   kf_tracker_->V_max_ = this->get_parameter("V_max").get_parameter_value().get<double>();

   this->declare_parameter("V_certain", 1.0);
   kf_tracker_->V_certain_ = this->get_parameter("V_certain").get_parameter_value().get<double>();

   this->declare_parameter("N_meas", 20);
   kf_tracker_->N_meas_ = this->get_parameter("N_meas").get_parameter_value().get<int>();

   this->declare_parameter("l_threshold", -2.0);
   kf_tracker_->l_threshold_ = this->get_parameter("l_threshold").get_parameter_value().get<double>();

   this->declare_parameter("state_buffer_length", 40);
   kf_tracker_->state_buffer_size_ = this->get_parameter("state_buffer_length").get_parameter_value().get<int>();

   RCLCPP_INFO(this->get_logger(),"State buffer length corresponds to %f seconds",
               kf_tracker_->dt_pred_*(double)kf_tracker_->state_buffer_size_);

   this->declare_parameter("do_kf_update_step", true);
   kf_tracker_->do_update_step_ = this->get_parameter("do_kf_update_step").get_parameter_value().get<bool>();

   this->declare_parameter("measurement_off_time", 2.0);
   kf_tracker_->measurement_off_time_ = this->get_parameter("measurement_off_time").get_parameter_value().get<double>();

   this->declare_parameter("print_debug_msg", false);
   kf_tracker_->debug_ = this->get_parameter("print_debug_msg").get_parameter_value().get<bool>();

   this->declare_parameter("tracking_frame", "map");
   kf_tracker_->tracking_frame_ = this->get_parameter("tracking_frame").get_parameter_value().get<std::string>();

   this->declare_parameter("target_frameid", "tag");
   target_frameid_ = this->get_parameter("target_frameid").get_parameter_value().get<std::string>();

   this->declare_parameter("use_track_id", true);
   kf_tracker_->use_track_id_ = this->get_parameter("use_track_id").get_parameter_value().get<bool>();

   this->declare_parameter("dist_threshold", 2.0);
   kf_tracker_->dist_threshold_ = this->get_parameter("dist_threshold").get_parameter_value().get<double>();

   this->declare_parameter("sigma_a", 1.0);
   kf_tracker_->sigma_a_ = this->get_parameter("sigma_a").get_parameter_value().get<double>();

   this->declare_parameter("sigma_p", 1.0);
   kf_tracker_->sigma_p_ = this->get_parameter("sigma_p").get_parameter_value().get<double>();

   this->declare_parameter("sigma_v", 1.0);
   kf_tracker_->sigma_v_ = this->get_parameter("sigma_v").get_parameter_value().get<double>();
   if(kf_tracker_->debug_){
      RCLCPP_INFO(this->get_logger(),"sigma_a: %f, sigma_p: %f, sigma_v: %f", kf_tracker_->sigma_a_, kf_tracker_->sigma_p_, kf_tracker_->sigma_v_);
   }

   this->declare_parameter("sigma_omega", 0.1);
   kf_tracker_->sigma_omega_ = this->get_parameter("sigma_omega").as_double();

   this->declare_parameter("sigma_theta", 0.2);
   kf_tracker_->sigma_theta_ = this->get_parameter("sigma_theta").as_double();

   this->declare_parameter("sigma_gamma", 0.15);
   kf_tracker_->sigma_gamma_ = this->get_parameter("sigma_gamma").as_double();

   std::vector<double> q_diag {0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ,0.1 , 0.1, 0.1};
   this->declare_parameter("q_diag", q_diag);
   kf_tracker_->q_diag_ = this->get_parameter("q_diag").get_parameter_value().get<std::vector<double>>();

   std::vector<double> r_diag {0.01, 0.01, 0.01};
   this->declare_parameter("r_diag", r_diag);
   kf_tracker_->r_diag_ = this->get_parameter("r_diag").get_parameter_value().get<std::vector<double>>();

   this->declare_parameter("r_diag_x", 0.01);
   this->declare_parameter("r_diag_y", 0.01);
   this->declare_parameter("r_diag_z", 0.01);

   this->declare_parameter("track_mesurement_timeout", 3.0);
   kf_tracker_->track_mesurement_timeout_ = this->get_parameter("track_mesurement_timeout").get_parameter_value().get<double>();

   kf_tracker_->last_measurement_t_ = 0.0; //this->now().seconds();
   kf_tracker_->last_prediction_t_ = 0.0; //this->now().seconds();

   RCLCPP_INFO(this->get_logger(),"Initializing Kalman filter...");
   if(!kf_tracker_->initKF()) 
   {
      RCLCPP_ERROR(this->get_logger(),"Could not initialize Kalman filter");
      return;
   }
   RCLCPP_INFO(this->get_logger(),"Kalman filter is initialized. Waiting for measurements...");


   // Define timers
   std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(kf_tracker_->dt_pred_));
   kf_loop_timer_ = this->create_wall_timer(
      duration_ms, std::bind(&TrackerROS::filterLoop, this)); // Define timer for constant loop rate

   params_timer_ = this->create_wall_timer(
      1000ms, std::bind(&TrackerROS::paramsTimerCallback, this));

   // Define subscribers
   pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "measurement/pose_array", 10, std::bind(&TrackerROS::poseArrayCallback, this, _1));

   // Define publishers
   good_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/good_tracks_pose_array", 10);
   all_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/all_tracks_pose_array", 10);
   good_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/good_tracks", 10);
   all_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/all_tracks", 10);
}


void
TrackerROS::poseArrayCallback(const geometry_msgs::msg::PoseArray & msg)
{
   // if(debug_)
   //    printf("[KFTracker::poseArrayCallback] Thred id: %s", std::this_thread::get_id());

   kf_tracker_->measurement_set_mtx_.lock();
   kf_tracker_->measurement_set_.clear();
   kf_tracker_->measurement_set_mtx_.unlock();
   
   // Sanity check
   if(msg.poses.empty())
   {
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[KFTracker::poseArrayCallback]: No measurements received.");

      // measurement_set_mtx_.unlock();
      return;
   }

   // auto now = this->now().seconds();
   kf_tracker_->measurement_set_mtx_.lock();
   for (auto it = msg.poses.begin(); it != msg.poses.end(); it++)
   {
      sensor_measurement z;
      
      // Use the time stamp from the message header
      double msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

      z.time_stamp = msg_time; //now; //static_cast<double>(msg.header.stamp.nanosec/1e9) ;
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[KFTracker::poseArrayCallback]: Measurement time = %f.", z.time_stamp);
      z.id = 0;
      z.z = Eigen::MatrixXd::Zero(3,1); // 3, because it's always position only, for now!
      z.z(0) = (*it).position.x;
      z.z(1) = (*it).position.y;
      z.z(2) = (*it).position.z;
      auto nrm = z.z.norm();
      if(nrm> 10000.0  || isnan(nrm))
      {
         if(kf_tracker_->debug_)
            RCLCPP_WARN(this->get_logger(),"[poseArrayCallback] Measurement norm is very large %f", z.z.norm());
         continue;
      }
      
      kf_tracker_->measurement_set_.push_back(z);
   }

   if(kf_tracker_->debug_){
      RCLCPP_INFO(this->get_logger(),"[poseArrayCallback] Size of measurement_set_ : %lu", kf_tracker_->measurement_set_.size());
      RCLCPP_INFO(this->get_logger(),"[poseArrayCallback] Size of msg->poses : %lu", msg.poses.size());
   }

   kf_tracker_->measurement_set_mtx_.unlock();

   return;

}

void
TrackerROS::publishCertainTracks(void)
{
   if(kf_tracker_->certain_tracks_.empty()){
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(),"[KFTracker::publishCertainTracks] certain_tracks_ is empty. No tracks to publish");
      return;
   }

   geometry_msgs::msg::Pose pose;
   geometry_msgs::msg::PoseArray pose_array;
   multi_target_kf::msg::KFTrack track_msg;
   multi_target_kf::msg::KFTracks tracks_msg;
   // rclcpp::Time time;
   // double timeInSeconds = kf_tracker_->certain_tracks_[0].current_state.time_stamp;
   // time.seconds(static_cast<int>(timeInSeconds));
   // time.nanoseconds(static_cast<int>((timeInSeconds - static_cast<int>(timeInSeconds)) * 1e9));
   pose_array.header.stamp = rclcpp::Time(static_cast<uint64_t>(kf_tracker_->certain_tracks_[0].current_state.time_stamp * 1e9));
   pose_array.header.frame_id = kf_tracker_->tracking_frame_;
   tracks_msg.header = pose_array.header;

   for (auto it = kf_tracker_->certain_tracks_.begin(); it != kf_tracker_->certain_tracks_.end(); it++)
   {
      pose.position.x = (*it).current_state.x[0];
      pose.position.y = (*it).current_state.x[1];
      pose.position.z = (*it).current_state.x[2];
      pose.orientation.w = 1.0;

      pose_array.poses.push_back(pose);

      track_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>((*it).current_state.time_stamp*1e9));
      track_msg.header.frame_id = kf_tracker_->tracking_frame_;
      track_msg.id = (*it).id;
      track_msg.n = (*it).n;

      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];

      /* The following are model-dependent ! */
      
      // for the constatn velocity model
      // track_msg.twist.twist.linear.x = (*it).current_state.x[3];
      // track_msg.twist.twist.linear.y = (*it).current_state.x[4];
      // track_msg.twist.twist.linear.z = (*it).current_state.x[5];

      // for constant acceleration model
      // track_msg.accel.accel.linear.x = (*it).current_state.x[6];
      // track_msg.accel.accel.linear.y = (*it).current_state.x[7];
      // track_msg.accel.accel.linear.z = (*it).current_state.x[8];

      // for cooridnated turn model

      // Extract state variables
      double v = (*it).current_state.x[3];
      double theta = (*it).current_state.x[4];
      double gamma = (*it).current_state.x[5];

      // Compute velocity components
      double vx = v * cos(theta) * cos(gamma);
      double vy = v * sin(theta) * cos(gamma);
      double vz = v * sin(gamma);

      // Assign to twist message
      track_msg.twist.twist.linear.x = vx;
      track_msg.twist.twist.linear.y = vy;
      track_msg.twist.twist.linear.z = vz;

      // Optionally, assign angular velocities
      track_msg.twist.twist.angular.z = (*it).current_state.x[6]; // Turn rate omega

      // Fill the PoseWithCovariance covariance (position covariance)
      for (int i = 0; i < 3; ++i) {
         for (int j = 0; j < 3; ++j) {
            track_msg.pose.covariance[i*6 + j] = (*it).current_state.P(i, j);
         }
      }

      // Since computing the covariance of the velocities is complex, you may set default values
      // Alternatively, set diagonal elements based on the variances of v, theta, gamma
      track_msg.twist.covariance[0] = 0.0; (*it).current_state.P(3,3); // Variance of v
      track_msg.twist.covariance[7] = 0.0; (*it).current_state.P(4,4); // Variance of theta
      track_msg.twist.covariance[14] = 0.0; (*it).current_state.P(5,5); // Variance of gamma
      

      tracks_msg.tracks.push_back(track_msg);
   }

   good_poses_pub_->publish(pose_array);
   good_tracks_pub_->publish(tracks_msg);
   
   return;
}

void
TrackerROS::publishAllTracks(void)
{
   if(kf_tracker_->tracks_.empty()){
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(),"[KFTracker::publishAllTracks] tracks_ is empty. No tracks to publish");
      return;
   }

   // RCLCPP_WARN(this->get_logger(),"[KFTracker::publishAllTracks] Number of all tracks: %d", tracks_.size());

   geometry_msgs::msg::Pose pose;
   geometry_msgs::msg::PoseArray pose_array;
   multi_target_kf::msg::KFTrack track_msg;
   multi_target_kf::msg::KFTracks tracks_msg;
   
   pose_array.header.stamp = rclcpp::Time(static_cast<uint64_t>(kf_tracker_->tracks_[0].current_state.time_stamp*1e9));
   pose_array.header.frame_id = kf_tracker_->tracking_frame_;
   tracks_msg.header = pose_array.header;

   for (auto it = kf_tracker_->tracks_.begin(); it != kf_tracker_->tracks_.end(); it++)
   {
      pose.position.x = (*it).current_state.x[0];
      pose.position.y = (*it).current_state.x[1];
      pose.position.z = (*it).current_state.x[2];
      pose.orientation.w = 1.0;

      pose_array.poses.push_back(pose);

      track_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>((*it).current_state.time_stamp*1e9)); // @todo need to convert to ros time
      track_msg.header.frame_id = kf_tracker_->tracking_frame_;
      track_msg.id = (*it).id;
      track_msg.n = (*it).n;

      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];

      /* The following are model-dependent ! */
      
      // For constatnt vel model
      // track_msg.twist.twist.linear.x = (*it).current_state.x[3];
      // track_msg.twist.twist.linear.y = (*it).current_state.x[4];
      // track_msg.twist.twist.linear.z = (*it).current_state.x[5];

      // For constant accel model
      // track_msg.accel.accel.linear.x = (*it).current_state.x[6];
      // track_msg.accel.accel.linear.y = (*it).current_state.x[7];
      // track_msg.accel.accel.linear.z = (*it).current_state.x[8];

      // For cooridnated turn model
      // Extract state variables
      double v = (*it).current_state.x[3];
      double theta = (*it).current_state.x[4];
      double gamma = (*it).current_state.x[5];

      // Compute velocity components
      double vx = v * cos(theta) * cos(gamma);
      double vy = v * sin(theta) * cos(gamma);
      double vz = v * sin(gamma);

      // Assign to twist message
      track_msg.twist.twist.linear.x = vx;
      track_msg.twist.twist.linear.y = vy;
      track_msg.twist.twist.linear.z = vz;

      // Optionally, assign angular velocities
      track_msg.twist.twist.angular.z = (*it).current_state.x[6]; // Turn rate omega

      // Fill the PoseWithCovariance covariance (position covariance)
      for (int i = 0; i < 3; ++i) {
         for (int j = 0; j < 3; ++j) {
            track_msg.pose.covariance[i*6 + j] = (*it).current_state.P(i, j);
         }
      }

      // Since computing the covariance of the velocities is complex, you may set default values
      // Alternatively, set diagonal elements based on the variances of v, theta, gamma
      track_msg.twist.covariance[0] = 0.0; (*it).current_state.P(3,3); // Variance of v
      track_msg.twist.covariance[7] = 0.0; (*it).current_state.P(4,4); // Variance of theta
      track_msg.twist.covariance[14] = 0.0; (*it).current_state.P(5,5); // Variance of gamma
      

      tracks_msg.tracks.push_back(track_msg);
   }

   all_poses_pub_->publish(pose_array);
   all_tracks_pub_->publish(tracks_msg);
   
   return;
}

/**
 * @brief 
*/
void
TrackerROS::filterLoop(void)
{
   if(kf_tracker_->debug_)
      RCLCPP_INFO(this->get_logger(), "[TrackerROS::filterLoop] inside filterLoop...");

   kf_tracker_->filterLoop(this->now().seconds());

   // Publish all available tracks
   publishAllTracks();

   // Publish state estimates pf ggod tracks as PoseArray, and as custom msg of array (KFTracks.msg) of pose with covariance
   publishCertainTracks();

   return;
}

void
TrackerROS::paramsTimerCallback()
{
   kf_tracker_->kf_model_.setSigmaA(this->get_parameter("sigma_a").as_double());
   // RCLCPP_INFO(this->get_logger(), "sigma_a %f", kf_tracker_->sigma_a_);
   
   double r_diag_x = this->get_parameter("r_diag_x").as_double();
   double r_diag_y = this->get_parameter("r_diag_y").as_double();
   double r_diag_z = this->get_parameter("r_diag_z").as_double();
   std::vector<double> r_diag = {r_diag_x, r_diag_y, r_diag_z};
   
   if (! kf_tracker_->kf_model_.R(r_diag))
      RCLCPP_ERROR(this->get_logger(), "[paramsTimerCallback] Could not set r_diag");

   kf_tracker_->V_max_ =  this->get_parameter("V_max").as_double();
   kf_tracker_->V_certain_ =  this->get_parameter("V_certain").as_double();
   int n =  this->get_parameter("N_meas").as_int();
   if (n>0)
      kf_tracker_->N_meas_ = n;
      
   kf_tracker_->l_threshold_ =  this->get_parameter("l_threshold").as_double();
}

#endif