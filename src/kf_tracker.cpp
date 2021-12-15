/*
BSD 3-Clause License

Copyright (c) 2020, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* TODO:
5. Implement a simpler Q_ matrix and R_ as per Saska's paper

6. Use seperate standard deviation parameters for position & velocity in Q_ :  q_pos_std_ , q_vel_std_
*/

#include "multi_target_kf/kf_tracker.h"

KFTracker::KFTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
dt_pred_(0.05),// 20Hz
q_(0.1),
r_(0.01),
q_pos_std_(0.1),
q_vel_std_(0.01),
V_max_(20.0),
V_certain_(1.0),
N_meas_(20),
l_threshold_(-2.0),
is_state_initialzed_(false),
state_buffer_size_(40),
tracking_frame_("map"),
target_frameid_("tag"),
listen_tf_(true),
do_update_step_(true),
measurement_off_time_(2.0),
debug_(false)
{
   nh_private_.param<double>("dt_pred", dt_pred_, 0.05);
   nh_private_.param<double>("q_std", q_, 0.1);
   nh_private_.param<double>("q_pos_std", q_pos_std_, 0.1);
   nh_private_.param<double>("q_vel_std", q_vel_std_, 0.01);
   nh_private_.param<double>("r_std", r_, 0.01);
   nh_private_.param<double>("V_max", V_max_, 20.0);
   nh_private_.param<double>("V_certain", V_certain_, 1.0);
   nh_private_.param<int>("N_meas", N_meas_, 20);
   nh_private_.param<double>("l_threshold", l_threshold_, -2.0);
   int buff_size;
   nh_private_.param<int>("state_buffer_length", buff_size, 40);
   state_buffer_size_  = (unsigned int) buff_size;
   ROS_INFO("State buffer length corresponds to %f seconds", dt_pred_*(double)state_buffer_size_);
   nh_private.param<bool>("do_kf_update_step", do_update_step_, true);
   nh_private.param<double>("measurement_off_time", measurement_off_time_, 2.0);
   nh_private.param<bool>("print_debug_msg", debug_, false);
   nh_private.param<std::string>("tracking_frame", tracking_frame_, "map");
   nh_private.param<std::string>("apriltags_topic", apriltags_topic_, "tag_detections");
   nh_private.param<std::string>("target_frameid", target_frameid_, "tag");
   nh_private.param<bool>("listen_tf", listen_tf_, true);
   

   initKF();

   kf_loop_timer_ =  nh_.createTimer(ros::Duration(dt_pred_), &KFTracker::filterLoop, this); // Define timer for constant loop rate

   //pose_sub_ =  nh_.subscribe("measurement/pose", 1, &KFTracker::poseCallback, this);
   pose_array_sub_ =  nh_.subscribe("measurement/pose_array", 1, &KFTracker::poseArrayCallback, this);

   // apriltags_sub_ = nh_.subscribe(apriltags_topic_, 1, &KFTracker::apriltagsCallback, this);

   // state_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("kf/estimate", 1);
   poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("kf/tracks_pose_array", 1);
   tracks_pub_ = nh_.advertise<multi_target_kf::KFTracks>("kf/tracks", 1);
   
}

KFTracker::~KFTracker()
{
}

void KFTracker::setQ(void)
{
   // This is for constant velocity model \in R^3

   Q_.resize(6,6);
   Q_ = Eigen::MatrixXd::Zero(6,6);
   // Q_(0,0) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // x with x
   // Q_(0,3) = 0.5*dt_pred_*dt_pred_; // x with vx
   // Q_(1,1) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // y with y
   // Q_(1,4) = 0.5*dt_pred_*dt_pred_; // y with vy
   // Q_(2,2) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // z with z
   // Q_(2,5) = 0.5*dt_pred_*dt_pred_; // z with vz
   // Q_(3,0) = Q_(0,3); // vx with x. Symmetric
   // Q_(3, 3) = dt_pred_; // vx with vx
   // Q_(4,1) = Q_(1,4); // vy with y. Symmetric
   // Q_(4,4) = dt_pred_; // vy with vy
   // Q_(5,2) = Q_(2,5); // vz with z. Symmetric
   // Q_(5,5) = dt_pred_; // vz with vz

   // Q_ = q_*q_*Q_; // multiply by process noise variance

    /************* The following is based on Saska's paper ********/
   Eigen::MatrixXd Qp = Eigen::MatrixXd::Identity(3,3);
   Qp = dt_pred_*q_pos_std_*q_pos_std_*Qp;
   Eigen::MatrixXd Qv = Eigen::MatrixXd::Identity(3,3);
   Qv = dt_pred_*q_vel_std_*q_vel_std_*Qv;

   Q_.block(0,0,3,3) = Qp;
   Q_.block(3,3,3,3) = Qv;

}

Eigen::MatrixXd KFTracker::setQ(double dt)
{
   // This is for constant velocity model \in R^3

   Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6,6);
   // Q(0,0) = 1./3.*dt*dt*dt; // x with x
   // Q(0,3) = 0.5*dt*dt; // x with vx
   // Q(1,1) = 1./3.*dt*dt*dt; // y with y
   // Q(1,4) = 0.5*dt*dt; // y with vy
   // Q(2,2) = 1./3.*dt*dt*dt; // z with z
   // Q(2,5) = 0.5*dt*dt; // z with vz
   // Q(3,0) = Q_(0,3); // vx with x. Symmetric
   // Q(3, 3) = dt; // vx with vx
   // Q(4,1) = Q(1,4); // vy with y. Symmetric
   // Q(4,4) = dt; // vy with vy
   // Q(5,2) = Q(2,5); // vz with z. Symmetric
   // Q(5,5) = dt; // vz with vz

   // Q = q_*q_*Q; // multiply by process noise variance

   /************* The following is based on Saska's paper ********/
   Eigen::MatrixXd Qp = Eigen::MatrixXd::Identity(3,3);
   Qp = dt*q_pos_std_*q_pos_std_*Qp;
   Eigen::MatrixXd Qv = Eigen::MatrixXd::Identity(3,3);
   Qv = dt*q_vel_std_*q_vel_std_*Qv;

   Q.block(0,0,3,3) = Qp;
   Q.block(3,3,3,3) = Qv;


   return Q;
}

void KFTracker::setR(void)
{
   R_.resize(3,3);
   R_ = Eigen::MatrixXd::Identity(3,3);
   R_ = r_*r_*R_; // multiply by observation noise variance
}

void KFTracker::setF(void)
{
   // This is for constant velocity model \in R^3

   F_.resize(6,6);
   F_ = Eigen::MatrixXd::Identity(6,6);
   F_(0,3) = dt_pred_; // x - vx
   F_(1,4) = dt_pred_; // y - vy
   F_(2,5) = dt_pred_; // z - vz
}

void KFTracker::setH(void)
{
   H_.resize(3,6);
   H_ = Eigen::MatrixXd::Zero(3,6);
   H_(0,0) = 1.0; // observing x
   H_(1,1) = 1.0; // observing y
   H_(2,2) = 1.0; // observing z
}

void KFTracker::initP(void)
{
   kf_state_pred_.P = Q_;
}

void KFTracker::initKF(void)
{
   // initial state KF estimate
   kf_state_pred_.time_stamp = ros::Time::now();
   kf_state_pred_.x = Eigen::MatrixXd::Zero(6,1); // position and velocity \in R^3
   kf_state_pred_.x(3,0) = 0.0000001;
   kf_state_pred_.x(4,0) = 0.0000001;
   kf_state_pred_.x(5,0) = 0.0000001;

   setQ(); // Initialize process noise covariance
   initP();
   setR(); // Initialize observation noise covariance
   setF(); // Initialize transition matrix
   setH(); // Initialize observation matrix

   state_buffer_.clear(); // Clear state buffer

   z_meas_.time_stamp = ros::Time::now();
   z_meas_.z.resize(3,1); 
   z_meas_.z = Eigen::MatrixXd::Zero(3,1); //  measured position \in R^3
   z_last_meas_ = z_meas_;

   last_measurement_t_ = ros::Time::now();


   ROS_INFO("KF is initialized.");

   return;
}

void KFTracker::updateStateBuffer(void)
{
   kf_state kfstate;
   kfstate.time_stamp = kf_state_pred_.time_stamp;
   kfstate.x = kf_state_pred_.x;
   kfstate.P = kf_state_pred_.P;
   state_buffer_.push_back(kfstate);
   if(state_buffer_.size() > state_buffer_size_)
      state_buffer_.erase(state_buffer_.begin()); // remove first element in the buffer

   return;
}

void KFTracker::predictTracks(void)
{
   // for(int i=0; i < tracks_.size(); i++)
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();
      (*it).current_state.x = F_*(*it).current_state.x; // state
      (*it).current_state.P = F_*(*it).current_state.P*F_.transpose() + Q_; // covariance

      // Calculate uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matirx
      auto V = sqrt(std::fabs(P_p.determinant()));

      // Remove tracks with high uncertainty
      // if(V > V_max_)
      // {
      //    if(debug_)
      //       ROS_WARN_THROTTLE(1.0, "[Prediction step] Track %d uncertainty = %f is high. Removing it.", i, V);
      //    tracks_.erase(it--);
      //    continue;
      // }

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());
   }

   return;
}

//****************** To be removed
bool KFTracker::predict(void)
{
   kf_state_pred_.x = F_*kf_state_pred_.x;
   if(kf_state_pred_.x.norm() > 10000.0)
   {
      ROS_ERROR("state prediciotn exploded!!!");
      is_state_initialzed_ = false;
      initKF();
      return false;
   }
   kf_state_pred_.P = F_*kf_state_pred_.P*F_.transpose() + Q_;
   kf_state_pred_.time_stamp = ros::Time::now();
   
   // Add state to buffer
   updateStateBuffer();

   return true;
}

kf_state KFTracker::predict(kf_state x, double dt)
{
   if (dt < 0.0){
      ROS_ERROR("[KFTracker::predict] dt <0 !. Skipping state prediciton");
      return x;
   }
   Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
   F(0,3) = dt; // x - vx
   F(1,4) = dt; // y - vy
   F(2,5) = dt; // z - vz

   auto state = x;

   auto Q = setQ(dt);

   state.x = F*state.x;
   state.P = F*state.P*F.transpose() + Q;
   state.time_stamp = state.time_stamp + ros::Duration(dt);

   return state;
}

double KFTracker::logLikelihood(kf_state x, sensor_measurement z)
{
   auto y_hat = z.z - H_*x.x; // innovation
   auto S = R_ + H_*x.P*H_.transpose(); // innovation covariance
   double LL = -0.5 * (y_hat.transpose() * S.inverse() * y_hat + log( std::fabs(S.determinant()) ) + 3.0*log(2*M_PI)); // log-likelihood

   return LL;
}

kf_state KFTracker::correctState(kf_state state, sensor_measurement z)
{
   // compute innovation
   auto y = z.z - H_*state.x;

   // Innovation covariance
   auto S = H_*state.P*H_.transpose() + R_;

   // Kalman gain
   auto K = state.P*H_.transpose()*S.inverse();

   // Updated state estimate and its covariance
   auto new_state = state;
   new_state.x = new_state.x + K*y;
   new_state.P = new_state.P - K*H_*new_state.P;

   return new_state;
}

void KFTracker::updateTracks(ros::Time t)
{
   
   // Sanity check
   if(tracks_.empty())
      return;


   auto z = measurement_set_;
   if(z.empty())
   {
      if(debug_){
         ROS_WARN_THROTTLE(1.0, "[updateTracks] No available measurements. Skipping update step.");
      }
      return;
   }

    // check if we got new measurement
    auto z_t = z[0].time_stamp.toSec();
   if (z_t <= last_measurement_t_.toSec())
   {
      if(debug_)
         ROS_WARN_THROTTLE(1.0, "[updateTracks] No new measurment. Skipping KF update step.");
      return;
   }
   last_measurement_t_ = z[0].time_stamp;

   // Measurement association and state update, for each track
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   //for(int i = 0; i< tracks_.size(); i++)
   {
      // find closest predicted state in time
      //auto buf_size = (*it).buffer.size()
      // Get the time of the last buffered state
      bool found_closest_state = false;
      for (int j=(*it).buffer.size()-1 ; j >=0 ; j--)
      {
         auto x_t = (*it).buffer[j].time_stamp.toSec(); // time of the i-th state in the buffer
         if( z_t >= x_t)
         {
            (*it).current_state.time_stamp = (*it).buffer[j].time_stamp;
            (*it).current_state.x = (*it).buffer[j].x;
            (*it).current_state.P = (*it).buffer[j].P;
            (*it).buffer.erase((*it).buffer.begin(), (*it).buffer.begin()+j+1);
            found_closest_state = true;
            break;
         }
      } // end loop over tracks_[i].buffer

      if( found_closest_state ){
         // predict state up to the current meaasurement time
         auto dt = z_t - (*it).current_state.time_stamp.toSec();
         auto proj_state = predict((*it).current_state, dt);
         (*it).current_state.time_stamp = proj_state.time_stamp;
         (*it).current_state.x = proj_state.x;
         (*it).current_state.P = proj_state.P;

         // Measurement-state association. Find measurement with the highest log-likelihood
         sensor_measurement best_m;
         double max_LL = -99999.0; // Start with very low number for the log-likelihood
         int m_index = 0; // index of measurement with max LL
         for(int k=0; k < z.size(); k++)
         {
            auto LL = logLikelihood((*it).current_state, z[k]);
            if (LL > max_LL && z[k].id == (*it).id )
            {
               max_LL = LL;
               best_m = z[k];
               m_index = k;
            }
         }
         // Check if max_LL is acceptable
         if(max_LL > l_threshold_)
         {
            // Do KF update step for this track
            auto corrected_x = correctState((*it).current_state, z[m_index]);
            (*it).current_state.x = corrected_x.x;
            (*it).current_state.P = corrected_x.P;
            (*it).n = (*it).n + 1;
            // Remove measurement
            z.erase(z.begin()+m_index);
         }
         else{
            if(debug_)
               ROS_WARN("Could not do measurement association for this track. Log-likelihood = %f", max_LL);
         }

         // Update KF estimate to the current time t
         dt = t.toSec() - z_t;
         auto final_x = predict((*it).current_state, dt);
         (*it).current_state.time_stamp = final_x.time_stamp;
         (*it).current_state.x = final_x.x;
         (*it).current_state.P = final_x.P;
      }
      else{
         // Update KF estimate (of the ones with no association) to the current time t
         auto dt = t.toSec() - (*it).current_state.time_stamp.toSec();
         auto final_x = predict((*it).current_state, dt);
         (*it).current_state.time_stamp = final_x.time_stamp;
         (*it).current_state.x = final_x.x;
         (*it).current_state.P = final_x.P;
      }    

      

   } // done looping over tracks

   // Add remaining measurements as new tracks
   if(!z.empty())
   {
      for (auto it = z.begin(); it != z.end(); it++)
      {
         kf_state state;
         state.x = Eigen::MatrixXd::Zero(6,1);
         state.x.block(0,0,3,1) = (*it).z;
         state.x(3) = 0.000001; state.x(4) = 0.000001; state.x(5) = 0.000001;
         state.P = Q_;
         state.P.block(0,0,3,3) = R_;
         state.time_stamp = (*it).time_stamp;
         
         kf_track new_track;
         new_track.id = (*it).id;
         new_track.current_state = state;
         new_track.n = 1;
         new_track.buffer.push_back(state);

         tracks_.push_back(new_track);
         if(debug_){
            ROS_WARN_THROTTLE(1.0, "******* New track is added ******* ");
         }
      }

   }

}

void KFTracker::updateCertainTracks(void)
{
   certain_tracks_.clear();
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      // Calculate uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matirx
      auto V = sqrt(std::fabs(P_p.determinant()));
      // If certainty is acceptable, add it to certain_tracks_
      if (V < V_certain_ and (*it).n > N_meas_)
      {
         certain_tracks_.push_back((*it));
      }
      else
      {
         if(debug_){
            ROS_WARN("Track is not considered certain. V = %f, N = %d", V, (*it).n);
         }
      }
      

      // Remove tracks with high uncertainty
      if(V > V_max_)
      {
         if(debug_)
            ROS_WARN_THROTTLE(1.0, "[Update step] Track uncertainty = %f is high. Removing it.", V);
         tracks_.erase(it--);
      }
   }
}

void KFTracker::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
   measurement_set_.clear();
   
   // Sanity check
   if(msg->poses.empty())
   {
      if(debug_)
         ROS_WARN("[KF Tracker - PoseArray Callback]: No measurements received.");

      return;
   }

   for (auto it = msg->poses.begin(); it != msg->poses.end(); it++)
   {
      sensor_measurement z;
      z.time_stamp = msg->header.stamp;
      z.z = Eigen::MatrixXd::Zero(3,1);
      z.z(0) = (*it).position.x;
      z.z(1) = (*it).position.y;
      z.z(2) = (*it).position.z;
      if(z.z.norm()> 10000.0)
      {
         continue;
      }
      measurement_set_.push_back(z);
   }

}

// void KFTracker::apriltagsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
// {
//    measurement_set_.clear();
   
//    // Sanity check
//    if(msg->detections.empty())
//    {
//       if(debug_)
//          ROS_WARN_THROTTLE(1.0, "[KF Tracker - apriltagsCallback]: No apriltag detections received.");
//       return;
//    }
//    for (auto it = msg->detections.begin(); it != msg->detections.end(); it++)
//    {
//       sensor_measurement z;
//       z.time_stamp = msg->header.stamp;
//       z.id = (unsigned int)(*it).id[0];
//       z.z = Eigen::MatrixXd::Zero(3,1);

//       if(listen_tf_)
//       {
//          tf::StampedTransform transform;
//          auto id_str = std::to_string( (*it).id[0] );
//          auto target_frameid = target_frameid_+id_str;
//          try{
//             tf_listener_.lookupTransform(tracking_frame_, target_frameid,  
//                                        ros::Time(0), transform);

//             z.z(0) = transform.getOrigin().x();
//             z.z(1) = transform.getOrigin().y();
//             z.z(2) = transform.getOrigin().z();
//             if(z.z.norm()> 10000.0)
//             {
//                continue;
//             }
//             measurement_set_.push_back(z);
//          }
//          catch (tf::TransformException ex){
//             ROS_ERROR("%s",ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//          }
//       }
//       else{
//          z.z(0) = (*it).pose.pose.pose.position.x;
//          z.z(1) = (*it).pose.pose.pose.position.y;
//          z.z(2) = (*it).pose.pose.pose.position.z;
//          if(z.z.norm()> 10000.0)
//          {
//             continue;
//          }
//          measurement_set_.push_back(z);
//       }
      
//    }
// }

/****** To Be Removed  ******/
void KFTracker::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   Eigen::Vector3d pos;
   pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   if (pos.norm() > 10000.0)
   {
      ROS_ERROR("[KF poseCallback] Infinite measurement value. Ignoring measurement.");
      return;
   }
      
   z_meas_.time_stamp = msg->header.stamp;
   z_meas_.z(0) = msg->pose.position.x;
   z_meas_.z(1) = msg->pose.position.y;
   z_meas_.z(2) = msg->pose.position.z;

   if(!is_state_initialzed_)
   {
      kf_state_pred_.x = z_meas_.z;
      is_state_initialzed_ = true;
      ROS_INFO("KF state estimate is initialized.");
   }
}

void KFTracker::publishTracks(void)
{
   if(certain_tracks_.empty())
      return;

   geometry_msgs::Pose pose;
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
      track_msg.pose.pose.position.x = (*it).current_state.x[0];
      track_msg.pose.pose.position.y = (*it).current_state.x[1];
      track_msg.pose.pose.position.z = (*it).current_state.x[2];
      track_msg.n = (*it).n;

      tracks_msg.tracks.push_back(track_msg);
   }

   poses_pub_.publish(pose_array);
   tracks_pub_.publish(tracks_msg);
   
   return;
}

// ************************ To be adjusted to publish all tracks estimates
void KFTracker::publishState(void)
{
   geometry_msgs::PoseWithCovarianceStamped msg;
   msg.header.frame_id = tracking_frame_;
   msg.header.stamp = kf_state_pred_.time_stamp;
   
   msg.pose.pose.position.x = kf_state_pred_.x(0);
   msg.pose.pose.position.y = kf_state_pred_.x(1);
   msg.pose.pose.position.z = kf_state_pred_.x(2);
   msg.pose.pose.orientation.w = 1.0; // Idenetity orientation

   auto pxx = kf_state_pred_.P(0,0); auto pyy = kf_state_pred_.P(1,1); auto pzz = kf_state_pred_.P(2,2);
   msg.pose.covariance[0] = pxx; msg.pose.covariance[7] = pyy; msg.pose.covariance[14] = pzz;
   state_pub_.publish(msg);
}

void KFTracker::initTracks(void)
{
   auto z = measurement_set_;
   if(z.empty())
   {
      if(debug_){
         ROS_WARN_THROTTLE(1.0, "No available measurements. Track initialization is skipped.");
      }
      return;
   }

   if(z[0].time_stamp.toSec() <= last_measurement_t_.toSec())
   {
      if(debug_)
         ROS_WARN_THROTTLE(1.0, "No new measurements. Track initilization is skipped.");
      return;
   }

   last_measurement_t_ = z[0].time_stamp;

   for (int i=0; i < z.size(); i++)
   {
      kf_state state;
      state.time_stamp = z[i].time_stamp;
      state.x = Eigen::MatrixXd::Zero(6,1);
      state.x.block(0,0,3,1) = z[i].z; // 3D position
      state.x(3) = 0.000001; // vx
      state.x(4) = 0.000001; // vy
      state.x(5) = 0.000001; // vz
      state.P = Q_;
      state.P.block(0,0,3,3) = R_;

      kf_track track;
      track.n = 1; // Number of measurements = 1 since it's the 1st one
      track.current_state = state;
      track.buffer.push_back(state);

      tracks_.push_back(track);
   }
   if(debug_){
      ROS_INFO("Initialized %lu tracks", tracks_.size());
   }

   return;

}

void KFTracker::filterLoop(const ros::TimerEvent& event)
{
   if(tracks_.empty())
   {
      /* Initialize tracks with current measurements. Then, return */
      if(debug_){
         ROS_WARN_THROTTLE(1.0, "No tracks to update. Initializing tracks using current measurements.");
      }
      initTracks();

      return;
   }

   // Do prediction step for all tracks.
   predictTracks();

   // Do correction step for all tracks using current measurements.
   updateTracks(ros::Time::now());

   // Extrack good tracks, and remove bad ones
   updateCertainTracks();

   // Publish state estimates as PoseArray, and as custom msg of array of pose with covariance
   publishTracks();

   return;
}
