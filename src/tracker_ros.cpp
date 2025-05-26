// src/tracker_ros.cpp
#include "multi_target_kf/tracker_ros.h"

TrackerROS::~TrackerROS()
{
   /* Destructor */
   delete kf_tracker_;
}

TrackerROS::TrackerROS() : Node("tracker_ros")
{
   // Load parameters from ROS parameter server
   loadParameters();
   
   // Create the KF tracker with the loaded configuration
   kf_tracker_ = new KFTracker(config_);
   
   // Initialize the KF tracker
   if (!kf_tracker_->initKF()) {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize Kalman filter");
      return;
   }
   
   RCLCPP_INFO(this->get_logger(), "Kalman filter is initialized with model type %s. Waiting for measurements...",
               ModelFactory::getModelName(config_.model_type));
   
   // Define timers
   std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(kf_tracker_->dt_pred_));
   
   kf_loop_timer_ = this->create_wall_timer(
      duration_ms, std::bind(&TrackerROS::filterLoop, this));

   params_timer_ = this->create_wall_timer(
      1000ms, std::bind(&TrackerROS::paramsTimerCallback, this));

   // Define subscribers
   pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "measurement/pose_array", 10, std::bind(&TrackerROS::poseArrayCallback, this, _1));

   // Detections subscriber
   detections_sub_ = this->create_subscription<multi_target_kf::msg::Detections>(
      "detections", 10, std::bind(&TrackerROS::detectionsCallback, this, _1));

   // Define publishers (unified tracks contain all information now)
   good_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/good_tracks_pose_array", 10);
   all_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("kf/all_tracks_pose_array", 10);
   good_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/good_tracks", 10);
   all_tracks_pub_ = this->create_publisher<multi_target_kf::msg::KFTracks>("kf/all_tracks", 10);
   
   // Remove enhanced publishers - unified approach now
}

void TrackerROS::loadParameters()
{

   // Model type selection
   this->declare_parameter("model_type", static_cast<int>(CONSTANT_VELOCITY));
   config_.model_type = static_cast<ModelType>(this->get_parameter("model_type").get_parameter_value().get<int>());
   
   // Common parameters
   this->declare_parameter("dt_pred", config_.dt_pred);
   config_.dt_pred = this->get_parameter("dt_pred").get_parameter_value().get<double>();
   
   this->declare_parameter("V_max", config_.V_max);
   config_.V_max = this->get_parameter("V_max").get_parameter_value().get<double>();
   
   this->declare_parameter("V_certain", config_.V_certain);
   config_.V_certain = this->get_parameter("V_certain").get_parameter_value().get<double>();
   
   this->declare_parameter("N_meas", config_.N_meas);
   config_.N_meas = this->get_parameter("N_meas").get_parameter_value().get<int>();
   
   this->declare_parameter("l_threshold", config_.l_threshold);
   config_.l_threshold = this->get_parameter("l_threshold").get_parameter_value().get<double>();
   
   this->declare_parameter("dist_threshold", config_.dist_threshold);
   config_.dist_threshold = this->get_parameter("dist_threshold").get_parameter_value().get<double>();

   // Fix the issue with state_buffer_length by using int instead of unsigned int
    this->declare_parameter("state_buffer_length", static_cast<int>(config_.state_buffer_size));
    config_.state_buffer_size = static_cast<unsigned int>(this->get_parameter("state_buffer_length").get_parameter_value().get<int>());
   
   this->declare_parameter("do_kf_update_step", config_.do_update_step);
   config_.do_update_step = this->get_parameter("do_kf_update_step").get_parameter_value().get<bool>();
   
   this->declare_parameter("measurement_off_time", config_.measurement_off_time);
   config_.measurement_off_time = this->get_parameter("measurement_off_time").get_parameter_value().get<double>();
   
   this->declare_parameter("print_debug_msg", config_.debug);
   config_.debug = this->get_parameter("print_debug_msg").get_parameter_value().get<bool>();
   
   this->declare_parameter("tracking_frame", config_.tracking_frame);
   config_.tracking_frame = this->get_parameter("tracking_frame").get_parameter_value().get<std::string>();
   
   this->declare_parameter("target_frameid", "tag");
   target_frameid_ = this->get_parameter("target_frameid").get_parameter_value().get<std::string>();
   
   this->declare_parameter("use_track_id", config_.use_track_id);
   config_.use_track_id = this->get_parameter("use_track_id").get_parameter_value().get<bool>();
   
   this->declare_parameter("track_measurement_timeout", config_.track_measurement_timeout);
   config_.track_measurement_timeout = this->get_parameter("track_measurement_timeout").get_parameter_value().get<double>();
   
   // Model parameters
   this->declare_parameter("sigma_a", config_.sigma_a);
   config_.sigma_a = this->get_parameter("sigma_a").get_parameter_value().get<double>();
   
   this->declare_parameter("sigma_p", config_.sigma_p);
   config_.sigma_p = this->get_parameter("sigma_p").get_parameter_value().get<double>();
   
   this->declare_parameter("sigma_v", config_.sigma_v);
   config_.sigma_v = this->get_parameter("sigma_v").get_parameter_value().get<double>();
   
   this->declare_parameter("sigma_j", config_.sigma_j);
   config_.sigma_j = this->get_parameter("sigma_j").get_parameter_value().get<double>();
   
   // R and Q diagonal elements
   std::vector<double> q_diag {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
   this->declare_parameter("q_diag", q_diag);
   config_.q_diag = this->get_parameter("q_diag").get_parameter_value().get<std::vector<double>>();
   
   std::vector<double> r_diag {0.01, 0.01, 0.01};
   this->declare_parameter("r_diag", r_diag);
   config_.r_diag = this->get_parameter("r_diag").get_parameter_value().get<std::vector<double>>();
   
   // Individual R diagonal elements (for convenience)
   this->declare_parameter("r_diag_x", 0.01);
   this->declare_parameter("r_diag_y", 0.01);
   this->declare_parameter("r_diag_z", 0.01);

   // UKF specific parameters
   if (config_.model_type == ADAPTIVE_ACCEL_UKF)
   {
        this->declare_parameter("alpha", config_.alpha);
        config_.alpha = this->get_parameter("alpha").get_parameter_value().get<double>();
        
        this->declare_parameter("beta", config_.beta);
        config_.beta = this->get_parameter("beta").get_parameter_value().get<double>();
        
        this->declare_parameter("kappa", config_.kappa);
        config_.kappa = this->get_parameter("kappa").get_parameter_value().get<double>();
        
        this->declare_parameter("jerk_std", config_.jerk_std);
        config_.jerk_std = this->get_parameter("jerk_std").get_parameter_value().get<double>();
        
        this->declare_parameter("jerk_adaptive_max", config_.jerk_adaptive_max);
        config_.jerk_adaptive_max = this->get_parameter("jerk_adaptive_max").get_parameter_value().get<double>();
        
        this->declare_parameter("adaptive_threshold", config_.adaptive_threshold);
        config_.adaptive_threshold = this->get_parameter("adaptive_threshold").get_parameter_value().get<double>();
        
        this->declare_parameter("adaptive_decay", config_.adaptive_decay);
        config_.adaptive_decay = this->get_parameter("adaptive_decay").get_parameter_value().get<double>();
        
        this->declare_parameter("innovation_window_size", static_cast<int>(config_.innovation_window_size));
        config_.innovation_window_size = static_cast<unsigned int>(this->get_parameter("innovation_window_size").get_parameter_value().get<int>());
   }
   
   // TF listening
   this->declare_parameter("listen_tf", false);
   listen_tf_ = this->get_parameter("listen_tf").get_parameter_value().get<bool>();
   
   if (config_.debug) {
        RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
        RCLCPP_INFO(this->get_logger(), "  Model type: %s", ModelFactory::getModelName(config_.model_type));
        RCLCPP_INFO(this->get_logger(), "  dt_pred: %f", config_.dt_pred);
        RCLCPP_INFO(this->get_logger(), "  V_max: %f", config_.V_max);
        RCLCPP_INFO(this->get_logger(), "  V_certain: %f", config_.V_certain);
        RCLCPP_INFO(this->get_logger(), "  N_meas: %d", config_.N_meas);
        RCLCPP_INFO(this->get_logger(), "  sigma_a: %f", config_.sigma_a);
        RCLCPP_INFO(this->get_logger(), "  sigma_p: %f", config_.sigma_p);
        RCLCPP_INFO(this->get_logger(), "  sigma_v: %f", config_.sigma_v);
        RCLCPP_INFO(this->get_logger(), "  sigma_j: %f", config_.sigma_j);
        RCLCPP_INFO(this->get_logger(), "  track_measurement_timeout: %f", config_.track_measurement_timeout);

        // Add UKF parameter logging if using UKF model
        if (config_.model_type == ADAPTIVE_ACCEL_UKF) {
            RCLCPP_INFO(this->get_logger(), "  alpha: %f", config_.alpha);
            RCLCPP_INFO(this->get_logger(), "  beta: %f", config_.beta);
            RCLCPP_INFO(this->get_logger(), "  kappa: %f", config_.kappa);
            RCLCPP_INFO(this->get_logger(), "  jerk_std: %f", config_.jerk_std);
            RCLCPP_INFO(this->get_logger(), "  jerk_adaptive_max: %f", config_.jerk_adaptive_max);
            RCLCPP_INFO(this->get_logger(), "  adaptive_threshold: %f", config_.adaptive_threshold);
            RCLCPP_INFO(this->get_logger(), "  adaptive_decay: %f", config_.adaptive_decay);
            RCLCPP_INFO(this->get_logger(), "  innovation_window_size: %u", config_.innovation_window_size);
        }
   }
}

void TrackerROS::poseArrayCallback(const geometry_msgs::msg::PoseArray & msg)
{
   kf_tracker_->measurement_set_mtx_.lock();
   kf_tracker_->measurement_set_.clear();
   // Clear detection set since we're using basic measurements
   kf_tracker_->detection_set_.clear();
   kf_tracker_->measurement_set_mtx_.unlock();
   
   // Sanity check
   if(msg.poses.empty())
   {
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[poseArrayCallback]: No measurements received.");
      return;
   }

   // auto now = this->now().seconds();
   // Use measurement time, instead of ROS now(). This is a proper way to match the actual measurement time
   double msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
   latest_measurement_time_ = std::max(latest_measurement_time_, msg_time);
   kf_tracker_->measurement_set_mtx_.lock();
   for (auto it = msg.poses.begin(); it != msg.poses.end(); it++)
   {
      sensor_measurement z;
      
      // z.time_stamp = now;
      z.time_stamp = msg_time;  // Use message timestamp
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[poseArrayCallback]: Measurement time = %f.", z.time_stamp);
      
      z.id = 0;
      z.z = Eigen::MatrixXd::Zero(3,1); // 3, because it's always position only, for now!
      z.z(0) = (*it).position.x;
      z.z(1) = (*it).position.y;
      z.z(2) = (*it).position.z;
      
      auto nrm = z.z.norm();
      if(nrm > 10000.0 || isnan(nrm))
      {
         if(kf_tracker_->debug_)
            RCLCPP_WARN(this->get_logger(), "[poseArrayCallback] Measurement norm is very large %f", z.z.norm());
         continue;
      }
      
      kf_tracker_->measurement_set_.push_back(z);
   }

   if(kf_tracker_->debug_){
      RCLCPP_INFO(this->get_logger(), "[poseArrayCallback] Size of measurement_set_ : %lu", kf_tracker_->measurement_set_.size());
      RCLCPP_INFO(this->get_logger(), "[poseArrayCallback] Size of msg->poses : %lu", msg.poses.size());
   }

   kf_tracker_->measurement_set_mtx_.unlock();
}

void TrackerROS::detectionsCallback(const multi_target_kf::msg::Detections & msg)
{
   kf_tracker_->measurement_set_mtx_.lock();
   // Clear both measurement sets
   kf_tracker_->detection_set_.clear();
   kf_tracker_->measurement_set_.clear();
   kf_tracker_->measurement_set_mtx_.unlock();
   
   if(msg.detections.empty())
   {
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[detectionsCallback]: No detections received.");
      return;
   }
   
   // auto now = this->now().seconds();
   // Use the message timestamp instead of now()
   double msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
   latest_measurement_time_ = std::max(latest_measurement_time_, msg_time);
   
   kf_tracker_->measurement_set_mtx_.lock();
   for (const auto& detection : msg.detections) {
      enhanced_measurement det = convertFromDetectionMsg(detection);
      // det.time_stamp = now;
      det.time_stamp = msg_time;  // Use message timestamp
      
      kf_tracker_->detection_set_.push_back(det);
      
      // Also add to basic measurements for backward compatibility
      sensor_measurement basic_meas = convertToBasicMeasurement(det);
      kf_tracker_->measurement_set_.push_back(basic_meas);
   }
   kf_tracker_->measurement_set_mtx_.unlock();
   
   if(kf_tracker_->debug_){
      RCLCPP_INFO(this->get_logger(), "[detectionsCallback] Received %lu detections", msg.detections.size());
   }
}

enhanced_measurement TrackerROS::convertFromDetectionMsg(const multi_target_kf::msg::Detection& detection_msg)
{
   enhanced_measurement meas;
   
   meas.id = detection_msg.id;
   meas.class_name = detection_msg.class_name;
   meas.confidence = detection_msg.confidence;
   
   // Position
   meas.position = Eigen::VectorXd::Zero(3);
   meas.position(0) = detection_msg.position.x;
   meas.position(1) = detection_msg.position.y;
   meas.position(2) = detection_msg.position.z;
   
   // Position covariance (diagonal)
   meas.position_cov = Eigen::MatrixXd::Identity(3, 3);
   meas.position_cov(0, 0) = detection_msg.position_covariance.x;
   meas.position_cov(1, 1) = detection_msg.position_covariance.y;
   meas.position_cov(2, 2) = detection_msg.position_covariance.z;
   
   // Linear velocity
   meas.has_linear_velocity = (detection_msg.linear_velocity.x != 0.0 || 
                              detection_msg.linear_velocity.y != 0.0 || 
                              detection_msg.linear_velocity.z != 0.0);
   if (meas.has_linear_velocity) {
      meas.linear_velocity = Eigen::VectorXd::Zero(3);
      meas.linear_velocity(0) = detection_msg.linear_velocity.x;
      meas.linear_velocity(1) = detection_msg.linear_velocity.y;
      meas.linear_velocity(2) = detection_msg.linear_velocity.z;
   }
   
   // 2D Bounding box (YOLO format: already center + size)
   meas.has_2d_bbox = detection_msg.has_2d_bbox;
   if (meas.has_2d_bbox) {
      meas.bbox_2d_center_x = detection_msg.bbox_2d_center_x;
      meas.bbox_2d_center_y = detection_msg.bbox_2d_center_y;
      meas.bbox_2d_width = detection_msg.bbox_2d_width;
      meas.bbox_2d_height = detection_msg.bbox_2d_height;
   }
   
   // 3D Bounding box
   meas.has_3d_bbox = detection_msg.has_3d_bbox;
   if (meas.has_3d_bbox) {
      meas.bbox_3d_center = Eigen::Vector3d(
         detection_msg.bbox_3d_center.x,
         detection_msg.bbox_3d_center.y,
         detection_msg.bbox_3d_center.z
      );
      meas.bbox_3d_size = Eigen::Vector3d(
         detection_msg.bbox_3d_size.x,
         detection_msg.bbox_3d_size.y,
         detection_msg.bbox_3d_size.z
      );
      meas.bbox_3d_orientation = Eigen::Quaterniond(
         detection_msg.bbox_3d_orientation.w,
         detection_msg.bbox_3d_orientation.x,
         detection_msg.bbox_3d_orientation.y,
         detection_msg.bbox_3d_orientation.z
      );
   }
   
   meas.attributes = detection_msg.attributes;
   
   return meas;
}

sensor_measurement TrackerROS::convertToBasicMeasurement(const enhanced_measurement& enhanced)
{
   sensor_measurement basic;
   basic.time_stamp = enhanced.time_stamp;
   basic.id = enhanced.id;
   basic.z = enhanced.position;
   basic.R = enhanced.position_cov;
   return basic;
}

void TrackerROS::publishCertainTracks(void)
{
   if(kf_tracker_->certain_tracks_.empty()){
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[publishCertainTracks] certain_tracks_ is empty. No tracks to publish");
      return;
   }

   geometry_msgs::msg::Pose pose;
   geometry_msgs::msg::PoseArray pose_array;
   multi_target_kf::msg::KFTrack track_msg;
   multi_target_kf::msg::KFTracks tracks_msg;
   
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
      
      // Add unified track fields
      track_msg.class_name = (*it).class_name;
      track_msg.confidence = (*it).confidence;
      track_msg.track_score = (*it).track_score;

      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];
      track_msg.pose.pose.orientation.w = 1.0;

      /* The following are model-dependent ! */
      if (config_.model_type == CONSTANT_VELOCITY || 
          config_.model_type == CONSTANT_ACCELERATION || 
          config_.model_type == ADAPTIVE_ACCEL_UKF) {
         track_msg.twist.twist.linear.x = (*it).current_state.x[3];
         track_msg.twist.twist.linear.y = (*it).current_state.x[4];
         track_msg.twist.twist.linear.z = (*it).current_state.x[5];
      }
      
      if ((config_.model_type == CONSTANT_ACCELERATION || 
           config_.model_type == ADAPTIVE_ACCEL_UKF) &&
          (*it).current_state.x.size() >= 9) {
         track_msg.accel.accel.linear.x = (*it).current_state.x[6];
         track_msg.accel.accel.linear.y = (*it).current_state.x[7];
         track_msg.accel.accel.linear.z = (*it).current_state.x[8];
      }

      // Fill the PoseWithCovariance covariance (first 3x3 block for position)
      for (int i = 0; i < 3; ++i) {
         for (int j = 0; j < 3; ++j) {
            track_msg.pose.covariance[i*6 + j] = (*it).current_state.P(i, j);
         }
      }

      // Fill velocity covariance if available
      if (config_.model_type == CONSTANT_VELOCITY || 
         config_.model_type == CONSTANT_ACCELERATION || 
         config_.model_type == ADAPTIVE_ACCEL_UKF) {
         
         if ((*it).current_state.x.size() >= 6) {
            for (int i = 3; i < 6; ++i) {
               for (int j = 3; j < 6; ++j) {
                  track_msg.twist.covariance[(i-3)*6 + (j-3)] = (*it).current_state.P(i, j);
               }
            }
         }
      }

      // Fill acceleration covariance if available
      if ((config_.model_type == CONSTANT_ACCELERATION || 
         config_.model_type == ADAPTIVE_ACCEL_UKF) &&
         (*it).current_state.x.size() >= 9) {
         
         for (int i = 6; i < 9; ++i) {
            for (int j = 6; j < 9; ++j) {
               track_msg.accel.covariance[(i-6)*6 + (j-6)] = (*it).current_state.P(i, j);
            }
         }
      }
      
      // 2D Bounding box
      track_msg.has_2d_bbox = (*it).has_2d_bbox;
      if ((*it).has_2d_bbox) {
         track_msg.bbox_2d_center_x = (*it).bbox_2d_center_x;
         track_msg.bbox_2d_center_y = (*it).bbox_2d_center_y;
         track_msg.bbox_2d_width = (*it).bbox_2d_width;
         track_msg.bbox_2d_height = (*it).bbox_2d_height;
      }
      
      // 3D Bounding box
      track_msg.has_3d_bbox = (*it).has_3d_bbox;
      if ((*it).has_3d_bbox) {
         track_msg.bbox_3d_center.x = (*it).bbox_3d_center.x();
         track_msg.bbox_3d_center.y = (*it).bbox_3d_center.y();
         track_msg.bbox_3d_center.z = (*it).bbox_3d_center.z();
         track_msg.bbox_3d_size.x = (*it).bbox_3d_size.x();
         track_msg.bbox_3d_size.y = (*it).bbox_3d_size.y();
         track_msg.bbox_3d_size.z = (*it).bbox_3d_size.z();
         track_msg.bbox_3d_orientation.w = (*it).bbox_3d_orientation.w();
         track_msg.bbox_3d_orientation.x = (*it).bbox_3d_orientation.x();
         track_msg.bbox_3d_orientation.y = (*it).bbox_3d_orientation.y();
         track_msg.bbox_3d_orientation.z = (*it).bbox_3d_orientation.z();
      }
      
      // Track quality
      auto P_p = (*it).current_state.P.block(0,0,3,3);
      track_msg.position_uncertainty = sqrt(std::abs(P_p.determinant()));
      
      track_msg.attributes = (*it).attributes;

      tracks_msg.tracks.push_back(track_msg);
   }

   good_poses_pub_->publish(pose_array);
   good_tracks_pub_->publish(tracks_msg);
}

void TrackerROS::publishAllTracks(void)
{
   if(kf_tracker_->tracks_.empty()){
      if(kf_tracker_->debug_)
         RCLCPP_WARN(this->get_logger(), "[publishAllTracks] tracks_ is empty. No tracks to publish");
      return;
   }

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

     track_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>((*it).current_state.time_stamp*1e9));
     track_msg.header.frame_id = kf_tracker_->tracking_frame_;
     track_msg.id = (*it).id;
     track_msg.n = (*it).n;
     
     // Add unified track fields
     track_msg.class_name = (*it).class_name;
     track_msg.confidence = (*it).confidence;
     track_msg.track_score = (*it).track_score;

     track_msg.pose.pose.position.x = (*it).current_state.x[0];
     track_msg.pose.pose.position.y = (*it).current_state.x[1];
     track_msg.pose.pose.position.z = (*it).current_state.x[2];
     track_msg.pose.pose.orientation.w = 1.0;

     /* The following are model-dependent ! */
     if (config_.model_type == CONSTANT_VELOCITY || 
         config_.model_type == CONSTANT_ACCELERATION || 
         config_.model_type == ADAPTIVE_ACCEL_UKF) {
        track_msg.twist.twist.linear.x = (*it).current_state.x[3];
        track_msg.twist.twist.linear.y = (*it).current_state.x[4];
        track_msg.twist.twist.linear.z = (*it).current_state.x[5];
     }
     
     if ((config_.model_type == CONSTANT_ACCELERATION || 
          config_.model_type == ADAPTIVE_ACCEL_UKF) &&
         (*it).current_state.x.size() >= 9) {
        track_msg.accel.accel.linear.x = (*it).current_state.x[6];
        track_msg.accel.accel.linear.y = (*it).current_state.x[7];
        track_msg.accel.accel.linear.z = (*it).current_state.x[8];
     }

     // Fill the PoseWithCovariance covariance (first 3x3 block for position)
      for (int i = 0; i < 3; ++i) {
         for (int j = 0; j < 3; ++j) {
            track_msg.pose.covariance[i*6 + j] = (*it).current_state.P(i, j);
         }
      }

      // Fill velocity covariance if available
      if (config_.model_type == CONSTANT_VELOCITY || 
         config_.model_type == CONSTANT_ACCELERATION || 
         config_.model_type == ADAPTIVE_ACCEL_UKF) {
         
         if ((*it).current_state.x.size() >= 6) {
            for (int i = 3; i < 6; ++i) {
               for (int j = 3; j < 6; ++j) {
                  track_msg.twist.covariance[(i-3)*6 + (j-3)] = (*it).current_state.P(i, j);
               }
            }
         }
      }

      // Fill acceleration covariance if available
      if ((config_.model_type == CONSTANT_ACCELERATION || 
         config_.model_type == ADAPTIVE_ACCEL_UKF) &&
         (*it).current_state.x.size() >= 9) {
         
         for (int i = 6; i < 9; ++i) {
            for (int j = 6; j < 9; ++j) {
               track_msg.accel.covariance[(i-6)*6 + (j-6)] = (*it).current_state.P(i, j);
            }
         }
      }
     
     // 2D Bounding box
     track_msg.has_2d_bbox = (*it).has_2d_bbox;
     if ((*it).has_2d_bbox) {
        track_msg.bbox_2d_center_x = (*it).bbox_2d_center_x;
        track_msg.bbox_2d_center_y = (*it).bbox_2d_center_y;
        track_msg.bbox_2d_width = (*it).bbox_2d_width;
        track_msg.bbox_2d_height = (*it).bbox_2d_height;
     }
     
     // 3D Bounding box
     track_msg.has_3d_bbox = (*it).has_3d_bbox;
     if ((*it).has_3d_bbox) {
        track_msg.bbox_3d_center.x = (*it).bbox_3d_center.x();
        track_msg.bbox_3d_center.y = (*it).bbox_3d_center.y();
        track_msg.bbox_3d_center.z = (*it).bbox_3d_center.z();
        track_msg.bbox_3d_size.x = (*it).bbox_3d_size.x();
        track_msg.bbox_3d_size.y = (*it).bbox_3d_size.y();
        track_msg.bbox_3d_size.z = (*it).bbox_3d_size.z();
        track_msg.bbox_3d_orientation.w = (*it).bbox_3d_orientation.w();
        track_msg.bbox_3d_orientation.x = (*it).bbox_3d_orientation.x();
        track_msg.bbox_3d_orientation.y = (*it).bbox_3d_orientation.y();
        track_msg.bbox_3d_orientation.z = (*it).bbox_3d_orientation.z();
     }
     
     // Track quality
     auto P_p = (*it).current_state.P.block(0,0,3,3);
     track_msg.position_uncertainty = sqrt(std::abs(P_p.determinant()));
     
     track_msg.attributes = (*it).attributes;

     tracks_msg.tracks.push_back(track_msg);
  }

  all_poses_pub_->publish(pose_array);
  all_tracks_pub_->publish(tracks_msg);
}

void TrackerROS::filterLoop(void)
{
    if(kf_tracker_->debug_)
        RCLCPP_INFO(this->get_logger(), "[TrackerROS::filterLoop] inside filterLoop...");

    // Determine current time based on mode
    double current_time;

    current_time = this->now().seconds();
    
    // Safety check: don't propagate backwards in time
    if (latest_measurement_time_ > 0.0 && current_time < latest_measurement_time_) {
        if(kf_tracker_->debug_) {
            RCLCPP_WARN(this->get_logger(), 
                       "[filterLoop] Current time (%f) is behind latest measurement (%f). Using measurement time.", 
                       current_time, latest_measurement_time_);
        }
        current_time = latest_measurement_time_;
    }

    // Run the filter loop with proper current time
    kf_tracker_->filterLoop(current_time);

    // Publish tracks
    publishAllTracks();
    publishCertainTracks();
}


void TrackerROS::paramsTimerCallback()
{
  // Update model-specific parameters
  if (config_.model_type == CONSTANT_VELOCITY) {
     // Check if any key parameters have changed
     double sigma_a = this->get_parameter("sigma_a").as_double();
     if (sigma_a != config_.sigma_a) {
        config_.sigma_a = sigma_a;
        kf_tracker_->sigma_a_ = sigma_a;
        ConstantVelModel* model = static_cast<ConstantVelModel*>(kf_tracker_->kf_model_);
        model->setSigmaA(sigma_a);
     }
  }
  else if (config_.model_type == CONSTANT_ACCELERATION) {
     // Check if any key parameters have changed
     double sigma_j = this->get_parameter("sigma_j").as_double();
     if (sigma_j != config_.sigma_j) {
        config_.sigma_j = sigma_j;
        kf_tracker_->sigma_j_ = sigma_j;
        ConstantAccelModel* model = static_cast<ConstantAccelModel*>(kf_tracker_->kf_model_);
        model->setSigmaJ(sigma_j);
     }
   }
   // Update UKF model parameters if using that model
   else if (config_.model_type == ADAPTIVE_ACCEL_UKF) {
       AdaptiveAccelUKF* model = static_cast<AdaptiveAccelUKF*>(kf_tracker_->kf_model_);
       
       // Check if any key parameters have changed
       double jerk_std = this->get_parameter("jerk_std").as_double();
       if (jerk_std != config_.jerk_std) {
           config_.jerk_std = jerk_std;
           model->setJerkStd(jerk_std);
       }
       
       double adaptive_threshold = this->get_parameter("adaptive_threshold").as_double();
       if (adaptive_threshold != config_.adaptive_threshold) {
           config_.adaptive_threshold = adaptive_threshold;
           model->setAdaptiveThreshold(adaptive_threshold);
       }
       
       double adaptive_decay = this->get_parameter("adaptive_decay").as_double();
       if (adaptive_decay != config_.adaptive_decay) {
           config_.adaptive_decay = adaptive_decay;
           model->setAdaptiveDecay(adaptive_decay);
       }
   }
  
  // Update R matrix from individual components if they're set
  double r_diag_x = this->get_parameter("r_diag_x").as_double();
  double r_diag_y = this->get_parameter("r_diag_y").as_double();
  double r_diag_z = this->get_parameter("r_diag_z").as_double();
  std::vector<double> r_diag = {r_diag_x, r_diag_y, r_diag_z};
  
  if (r_diag != config_.r_diag) {
     config_.r_diag = r_diag;
     if (!kf_tracker_->kf_model_->R(r_diag))
        RCLCPP_ERROR(this->get_logger(), "[paramsTimerCallback] Could not set r_diag");
  }
  
  // Update other tracking parameters
  double V_max = this->get_parameter("V_max").as_double();
  if (V_max != config_.V_max) {
     config_.V_max = V_max;
     kf_tracker_->V_max_ = V_max;
  }
  
  double V_certain = this->get_parameter("V_certain").as_double();
  if (V_certain != config_.V_certain) {
     config_.V_certain = V_certain;
     kf_tracker_->V_certain_ = V_certain;
  }
  
  int N_meas = this->get_parameter("N_meas").as_int();
  if (N_meas > 0 && N_meas != config_.N_meas) {
     config_.N_meas = N_meas;
     kf_tracker_->N_meas_ = N_meas;
  }
  
  double l_threshold = this->get_parameter("l_threshold").as_double();
  if (l_threshold != config_.l_threshold) {
     config_.l_threshold = l_threshold;
     kf_tracker_->l_threshold_ = l_threshold;
  }
  bool do_update_step = this->get_parameter("do_kf_update_step").as_bool();
  if (do_update_step != config_.do_update_step) {
     config_.do_update_step = do_update_step;
     kf_tracker_->do_update_step_ = do_update_step;
  }
  
  double track_measurement_timeout = this->get_parameter("track_measurement_timeout").as_double();
  if (track_measurement_timeout != config_.track_measurement_timeout) {
     config_.track_measurement_timeout = track_measurement_timeout;
     kf_tracker_->track_measurement_timeout_ = track_measurement_timeout;
  }
  
  bool use_track_id = this->get_parameter("use_track_id").as_bool();
  if (use_track_id != config_.use_track_id) {
     config_.use_track_id = use_track_id;
     kf_tracker_->use_track_id_ = use_track_id;
  }
}