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
use_track_id_(false),
debug_(false)
{
   nh_private_.param<double>("dt_pred", dt_pred_, 0.05);
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
   nh_private.param<bool>("use_track_id", use_track_id_, true);

   if( !nh_private.getParam("q_diag", q_diag_) ){
      ROS_ERROR("Failed to get q_diag parameter");
      return;
   }


   if( !nh_private.getParam("r_diag", r_diag_) ){
      ROS_ERROR("Failed to get r_diag parameter");
      return;
   }
   

   if(!initKF()) return;

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



bool KFTracker::initKF(void)
{

   kf_model_.debug(debug_);
   if(!kf_model_.Q(q_diag_)) return false; // initialize Process covariance matrix
   if(!kf_model_.R(r_diag_)) return false; // initialize measurment covariance matrix
   if(!kf_model_.P(kf_model_.Q())) return false; // initialize state covariance matrix

   // Clear all buffers
   state_buffer_.clear();
   tracks_.clear();
   certain_tracks_.clear();

   z_meas_.time_stamp = ros::Time::now();
   z_meas_.z.resize(3,1); // number of measurements are always 3 (3D position)
   z_meas_.z = Eigen::MatrixXd::Zero(3,1); //  measured position \in R^3
   z_last_meas_ = z_meas_;

   last_measurement_t_ = ros::Time::now();


   ROS_INFO("KF is initialized. Waiting for measurements ...");

   return true;
}


void KFTracker::predictTracks(void)
{
   if(debug_)
      ROS_INFO("[KFTracker::predictTracks] Predicting tracks...");

   // for(int i=0; i < tracks_.size(); i++)
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      (*it).current_state = kf_model_.predictX((*it).current_state, dt_pred_);
      // (*it).current_state.time_stamp = (*it).current_state.time_stamp + ros::Duration(dt_pred_);
      // (*it).current_state.x = kf_model_.f((*it).current_state.x, dt_pred_); // state
      // (*it).current_state.P = kf_model_.F(dt_pred_)*(*it).current_state.P*kf_model_.F(dt_pred_).transpose() + kf_model_.Q(); // covariance

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());
   }

   if(debug_)
      ROS_INFO("[KFTracker::predictTracks] Done predicting tracks.");

   return;
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
      for (int j=(*it).buffer.size()-1 ; j >=0 ; j--) // start from the last state, should have the biggest/latest time stamp
      {
         auto x_t = (*it).buffer[j].time_stamp.toSec(); // time of the j-th state in the buffer
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

         if(debug_)
            ROS_INFO("[updateTracks] Closest state in track %d, time difference = %f", it-tracks_.begin(), z_t - (*it).current_state.time_stamp.toSec());

         double dt;
         // predict state up to the current meaasurement time (NOT needed?!)
         // dt = z_t - (*it).current_state.time_stamp.toSec();
         // auto proj_state = predict((*it).current_state, dt);
         // (*it).current_state.time_stamp = proj_state.time_stamp;
         // (*it).current_state.x = proj_state.x;
         // (*it).current_state.P = proj_state.P;

         // Measurement-state association. Find measurement with the highest log-likelihood
         sensor_measurement best_m;
         double max_LL = -99999.0; // Start with very low number for the log-likelihood
         int m_index = 0; // index of measurement with max LL
         for(int k=0; k < z.size(); k++)
         {
            auto LL = kf_model_.logLikelihood((*it).current_state, z[k]);
            if (LL > max_LL )
            {
               if( use_track_id_ && z[k].id != (*it).id ) continue;
               max_LL = LL;
               best_m = z[k];
               m_index = k;
            }
         }
         // Check if max_LL is acceptable
         if(max_LL > l_threshold_)
         {
            if(debug_){
               ROS_INFO("[updateTracks] Associated measurement %d with track %d", m_index, it-tracks_.begin());
            }
            // Do KF update step for this track
            auto corrected_x = kf_model_.updateX(z[m_index], (*it).current_state);
            (*it).current_state.time_stamp = corrected_x.time_stamp;
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
         dt = t.toSec() - (*it).current_state.time_stamp.toSec();
         auto final_x = kf_model_.predictX((*it).current_state, dt);
         (*it).current_state.time_stamp = final_x.time_stamp;
         (*it).current_state.x = final_x.x;
         (*it).current_state.P = final_x.P;
      }
      else{
         if(debug_)
            ROS_WARN("[updateTracks] Didn't find closest state of track %d in time w.r.t measurements", it-tracks_.begin());
         // Update KF estimate (of the ones with no association) to the current time t
         auto dt = t.toSec() - (*it).current_state.time_stamp.toSec();
         auto final_x = kf_model_.predictX((*it).current_state, dt);
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
         state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
         state.x.block(0,0,3,1) = (*it).z;

         state.P = kf_model_.Q();
         state.time_stamp = (*it).time_stamp;
         
         kf_track new_track;
         new_track.id = (*it).id;
         new_track.current_state = state;
         new_track.n = 1;
         new_track.buffer.push_back(state);

         tracks_.push_back(new_track);
         if(debug_){
            ROS_WARN( "******* New track is added ******* ");
         }
      }

   }

}

void KFTracker::updateCertainTracks(void)
{
   if(debug_){
      ROS_INFO("[KFTracker::updateCertainTracks] Number of available tracks = %d", tracks_.size());
   }

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
            ROS_WARN( "[Update step] Track uncertainty = %f is high (> %f). Removing it.", V, V_max_);
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
      z.id = 0;
      z.z = Eigen::MatrixXd::Zero(3,1); // 3, because it's always position only, for now!
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



void KFTracker::publishTracks(void)
{
   if(certain_tracks_.empty()){
      if(debug_)
         ROS_WARN("[KFTracker::publishTracks] certain_tracks_ is empty. No tracks to publish");
      return;
   }

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

   poses_pub_.publish(pose_array);
   tracks_pub_.publish(tracks_msg);
   
   return;
}

void KFTracker::initTracks(void)
{
   if(debug_)
      ROS_INFO("[KFTracker::initTracks] Initializing tracks...");

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
      state.x.resize(kf_model_.numStates(),1);
      state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
      state.x.block(0,0,3,1) = z[i].z; // 3D position
      state.x.block(3,0,kf_model_.numStates()-3,1) = 0.000001*Eigen::MatrixXd::Ones(kf_model_.numStates()-3,1);
      
      state.P = kf_model_.Q();
      // state.P.block(0,0,3,3) = R_;

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
   if(debug_)
      ROS_INFO_THROTTLE(1, "[KFTracker::filterLoop] inside filterLoop...");

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

   // Do correction step for all tracks using latest measurements.
   updateTracks(ros::Time::now());

   // Extrack good tracks, and remove bad ones
   updateCertainTracks();

   // Publish state estimates as PoseArray, and as custom msg of array of pose with covariance
   publishTracks();

   return;
}
