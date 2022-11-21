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
dist_threshold_(2.0),
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
   nh_private.param<double>("dist_threshold", dist_threshold_, 2.0);


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
   good_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("kf/good_tracks_pose_array", 1);
   all_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("kf/all_tracks_pose_array", 1);
   good_tracks_pub_ = nh_.advertise<multi_target_kf::KFTracks>("kf/good_tracks", 1);
   all_tracks_pub_ = nh_.advertise<multi_target_kf::KFTracks>("kf/all_tracks", 1);
   
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
   last_prediction_t_ = ros::Time::now();


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
      int i = it - tracks_.begin();

      if(debug_)
         ROS_INFO("[KFTracker::predictTracks] Predicting track %d", i);

      (*it).current_state = kf_model_.predictX((*it).current_state, dt_pred_);
      // (*it).current_state.time_stamp = (*it).current_state.time_stamp + ros::Duration(dt_pred_);
      // (*it).current_state.x = kf_model_.f((*it).current_state.x, dt_pred_); // state
      // (*it).current_state.P = kf_model_.F(dt_pred_)*(*it).current_state.P*kf_model_.F(dt_pred_).transpose() + kf_model_.Q(); // covariance

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());

      if(debug_)
         ROS_INFO("[KFTracker::predictTracks] Done predicting track %d", i);
   }

   if(debug_)
      ROS_INFO("[KFTracker::predictTracks] Done predicting all tracks.");
   
   return;
}

void KFTracker::predictTracks(double dt)
{
   if(debug_)
      ROS_INFO("[KFTracker::predictTracks] Predicting tracks...");

   // for(int i=0; i < tracks_.size(); i++)
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();

      if(debug_)
         ROS_INFO("[KFTracker::predictTracks] Predicting track %d", i);

      (*it).current_state = kf_model_.predictX((*it).current_state, dt);
      // (*it).current_state.time_stamp = (*it).current_state.time_stamp + ros::Duration(dt_pred_);
      // (*it).current_state.x = kf_model_.f((*it).current_state.x, dt_pred_); // state
      // (*it).current_state.P = kf_model_.F(dt_pred_)*(*it).current_state.P*kf_model_.F(dt_pred_).transpose() + kf_model_.Q(); // covariance

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());

      if(debug_)
         ROS_INFO("[KFTracker::predictTracks] Done predicting track %d", i);
   }

   if(debug_)
      ROS_INFO("[KFTracker::predictTracks] Done predicting all tracks.");
   
   return;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void KFTracker::updateTracks(ros::Time t)
{
   // if(debug_)
   //    ROS_INFO("[KFTracker::updateTracks] Thread id: %s", std::this_thread::get_id());

   // Sanity checks

   if(tracks_.empty())
      return;

   auto z = measurement_set_;

   if(z.empty())
   {
      if(debug_)
         ROS_WARN("[updateTracks] No available measurements. Skipping update step.");
      return;
   }

    // check if we got new measurement
    /** @warning Currently, it's assumed that all measurement in the buffer measurement_set_ have the same time stamp */
    auto z_t = z[0].time_stamp.toSec();
   if (z_t <= last_measurement_t_.toSec())
   {
      if(debug_)
         ROS_WARN("[updateTracks] No new measurment. Skipping KF update step.");
      return;
   }
   last_measurement_t_ = z[0].time_stamp;

   /** @todo compute loglikelihood matrix to be passed to the Hungarian algorithm */
   if(debug_)
      ROS_INFO("[KFTracker::updateTracks] Computing loglikelihood matrix...");
   // std::vector< std::vector<double> > cost_mat; cost_mat.resize(tracks_.size());
   Eigen::MatrixXd LL_mat(tracks_.size(), z.size());
   for(auto it_t=tracks_.begin(); it_t != tracks_.end(); it_t++){
      int tr_idx = it_t - tracks_.begin(); // track index
      /** @todo Should we predict the state to the current measurement time??? */

      for(auto it_z=z.begin(); it_z != z.end(); it_z++){
         int z_idx = it_z - z.begin(); // measurement index
         /** @todo check for INF values */ 
         LL_mat(tr_idx, z_idx) = kf_model_.logLikelihood((*it_t).current_state, (*it_z));

         // double dist = kf_model_.computeDistance((*it_t).current_state, (*it_z));
         // if(debug_)
         //    ROS_INFO("[KFTracker::updateTracks] Distance between track %d and measurement %d  = %f", tr_idx, z_idx, dist);
         // if ( (dist <= dist_threshold_) && found_closest_state){
         //    LL_mat(tr_idx, z_idx) = dist;
         // }
         // else  LL_mat(tr_idx, z_idx) = 9999.0;

         if(debug_)
            ROS_INFO("[KFTracker::updateTracks] LL_mat(%d,%d) = %f", tr_idx, z_idx, LL_mat(tr_idx, z_idx));
      }

   } // done looping over tracks & measurements, and computing the LL_mat

   if(debug_){
      ROS_INFO("[KFTracker::updateTracks] Done computing loglikelihood matrix");
      std::cout << "loglikelihood matrix: \n" << LL_mat << "\n";
   }

   /** @todo Post-process the cost matrix
    * 1. Hungarian algorithm minimizes cost, so first negate the logliklihood matrix
    * 2. Hungarian algorithm requires cost matrix with non negative elements only.
    *    So, subtract the minimum element from the matrix resulting from step 1
   */
   if (true){ //set to true if using loglikelihood instead of distance
      if(debug_)
         ROS_INFO("[KFTracker::updateTracks] Preparing cost matrix for the Hungarian algorithm...");
      // negate to convert to cost
      LL_mat = -1.0*LL_mat;
      // subtract minimum value to make sure all the elements are positive
      LL_mat = LL_mat - ( LL_mat.minCoeff()*Eigen::MatrixXd::Ones(LL_mat.rows(), LL_mat.cols()) );
   }
  
  std::vector< std::vector<double> > costMat;
  std::vector<double> row;
  for (int i=0; i< LL_mat.rows(); i++){
     row.clear();
     for(int j=0; j<LL_mat.cols(); j++){
        row.push_back(LL_mat(i,j));
     }
     costMat.push_back(row);
  }
  if(debug_){
     ROS_INFO("[KFTracker::updateTracks] Cost matrix is prepared");
     std::cout << "costMat: \n";
      for(int ii=0; ii<costMat.size(); ii++){
         for(int jj=0; jj<costMat[ii].size(); jj++)
            std::cout << costMat[ii][jj] << " "  ;
      }
      std::cout << "\n";
  }

   /** @todo  apply Hungarian algorithm on cost_mat, to get measurement-state assignment */
   if(debug_)
      ROS_INFO("[KFTracker::updateTracks] Executing Hungarian algorithm...");
   
   std::vector<int> assignment; // Assignment vector, has size of tracks_
   double cost = HungAlgo_.Solve(costMat, assignment);
   if(debug_){
      ROS_INFO("[KFTracker::updateTracks] Hungarian algorithm is executed");
      std::cout << "Assignment vector: \n";
      for(int i=0; i<assignment.size(); i++){
         std::cout << assignment[i] << " ";
      }
      std::cout << "\n";
   }

   // vector to mark the assigned measurements. 1 if assigned, 0 otherwise
   // This will be used to add the non-assigned measurement(s) as new track(s)
   // size of z
   Eigen::VectorXi assigned_z(z.size()); assigned_z = Eigen::VectorXi::Zero(z.size());

   /** @todo apply KF update step for each track, if it's assigned a measurement, and then predict to the current time step
    * If a track is not assigned a measurement, just predict it to the current time step
    * Remove measurements that are already assigned to tracks, after they are used to update their assigned tracks.
   */

   if(debug_)
      ROS_INFO("[KFTracker::updateTracks] Updating tracks using assigned measurements");
   for(auto it_t=tracks_.begin(); it_t!=tracks_.end(); it_t++){
      int tr_idx = it_t - tracks_.begin();

      
      // Apply state correction
      if (assignment[tr_idx] > -1){

         // we have to double check the assigned measurement is not bad!
         // because hungarian algorithm will just do matching without respecting any threshold
         double LL = kf_model_.logLikelihood((*it_t).current_state, z[assignment[tr_idx]]);
         // double dist = kf_model_.computeDistance((*it_t).current_state, z[assignment[tr_idx]]);
         // if(dist <= dist_threshold_){
         if(LL >= l_threshold_){
            assigned_z(assignment[tr_idx]) = 1;
            // correct/update track
            (*it_t).current_state = kf_model_.updateX(z[assignment[tr_idx]], (*it_t).current_state);
            (*it_t).n += 1;
            // (*it_t).buffer.push_back((*it_t).current_state);
            // if((*it_t).buffer.size() > state_buffer_size_)
            //    (*it_t).buffer.erase((*it_t).buffer.begin());
         }
      }

      // // predict track to the current time stamp, if possible
      // double dt = t.toSec() - (*it_t).current_state.time_stamp.toSec();
      // (*it_t).current_state = kf_model_.predictX((*it_t).current_state, dt);
      // (*it_t).buffer.push_back((*it_t).current_state);
      // if((*it_t).buffer.size() > state_buffer_size_)
      //    (*it_t).buffer.erase((*it_t).buffer.begin());


   }// Done updating tracks

   if(debug_)
      std::cout << "[updateTracks] assigned_z vector: \n" << assigned_z << "\n";

   /** @todo  If there are reamining measurements, use add them as new tracks. */
   if(debug_)
      ROS_INFO("[KFTracker::updateTracks] Adding new tracks using non-assigned measurements");
   for( int m=0; m<z.size(); m++){
      if(assigned_z(m) > 0) continue; // this measurement is assigned, so skip it
      kf_state state;
      state.x.resize(kf_model_.numStates(),1);
      state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
      state.x.block(0,0,3,1) = z[m].z;
      state.x.block(3,0,kf_model_.numStates()-3,1) = Eigen::MatrixXd::Zero(kf_model_.numStates()-3,1);

      state.P = kf_model_.Q(dt_pred_);
      // state.P.block(0,0,3,3) = kf_model_.R();
      state.time_stamp = z[m].time_stamp;
      
      kf_track new_track;
      new_track.id = z[m].id;
      new_track.current_state = state;
      new_track.n = 1;
      // new_track.buffer.push_back(state);

      tracks_.push_back(new_track);
      if(debug_){
         ROS_WARN( "******* New track is added using a non-assigned measurement: %d ******* ", m);
      }      
   }

   // DONE

}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// void KFTracker::updateTracks(ros::Time t)
// {
//    // if(debug_)
//    //    ROS_INFO("[KFTracker::updateTracks] Thread id: %s", std::this_thread::get_id());

//    // Sanity checks

//    if(tracks_.empty())
//       return;

//    auto z = measurement_set_;

//    if(z.empty())
//    {
//       if(debug_)
//          ROS_WARN("[updateTracks] No available measurements. Skipping update step.");
//       return;
//    }

//     // check if we got new measurement
//     /** @warning Currently, it's assumed that all measurement in the buffer measurement_set_ have the same time stamp */
//     auto z_t = z[0].time_stamp.toSec();
//    if (z_t <= last_measurement_t_.toSec())
//    {
//       if(debug_)
//          ROS_WARN("[updateTracks] No new measurment. Skipping KF update step.");
//       return;
//    }
//    last_measurement_t_ = z[0].time_stamp;

//    /** @todo compute loglikelihood matrix to be passed to the Hungarian algorithm */
//    if(debug_)
//       ROS_INFO("[KFTracker::updateTracks] Computing loglikelihood matrix...");
//    // std::vector< std::vector<double> > cost_mat; cost_mat.resize(tracks_.size());
//    Eigen::MatrixXd LL_mat(tracks_.size(), z.size());
//    for(auto it_t=tracks_.begin(); it_t != tracks_.end(); it_t++){
//       int tr_idx = it_t - tracks_.begin(); // track index      
//       // First, check if there is close state in time
//       bool found_closest_state = false;
//       for (int k=(*it_t).buffer.size()-1 ; k >=0 ; k--) // start from the last state in the buffer, should have the biggest/latest time stamp
//       {
//          auto x_t = (*it_t).buffer[k].time_stamp.toSec(); // time of the k-th state in the state-buffer of this track
//          if( z_t >= x_t)
//          {
//             // make the closest state the current_state, and remove all the future ones if any
//             (*it_t).current_state.time_stamp = (*it_t).buffer[k].time_stamp;
//             (*it_t).current_state.x = (*it_t).buffer[k].x;
//             (*it_t).current_state.P = (*it_t).buffer[k].P;
//             (*it_t).buffer.clear(); (*it_t).buffer.push_back((*it_t).current_state);
//             found_closest_state = true;

//             break;
//          }

//       } // end loop over bufferd state in this track

//       for(auto it_z=z.begin(); it_z != z.end(); it_z++){
//          int z_idx = it_z - z.begin(); // measurement index
//          double LL = kf_model_.logLikelihood((*it_t).current_state, (*it_z));
//          if ( (LL >= l_threshold_) && found_closest_state) LL_mat(tr_idx, z_idx) = LL;
//          else  LL_mat(tr_idx, z_idx) = -9999.0;

//          // double dist = kf_model_.computeDistance((*it_t).current_state, (*it_z));
//          // if(debug_)
//          //    ROS_INFO("[KFTracker::updateTracks] Distance between track %d and measurement %d  = %f", tr_idx, z_idx, dist);
//          // if ( (dist <= dist_threshold_) && found_closest_state){
//          //    LL_mat(tr_idx, z_idx) = dist;
//          // }
//          // else  LL_mat(tr_idx, z_idx) = 9999.0;

//          if(debug_)
//             ROS_INFO("[KFTracker::updateTracks] LL_mat(%d,%d) = %f", tr_idx, z_idx, LL_mat(tr_idx, z_idx));
//       }

//    } // done looping over tracks, and computing the LL_mat

//    if(debug_){
//       ROS_INFO("[KFTracker::updateTracks] Done computing loglikelihood matrix");
//       std::cout << "loglikelihood matrix: \n" << LL_mat << "\n";
//    }

//    /** @todo Post-process the cost matrix
//     * 1. Hungarian algorithm minimizes cost, so first negate the logliklihood matrix
//     * 2. Hungarian algorithm requires cost matrix with non negative elements only.
//     *    So, subtract the minimum element from the matrix resulting from step 1
//    */
//    if (true){ //set to true if using loglikelihood instead of distance
//       if(debug_)
//          ROS_INFO("[KFTracker::updateTracks] Preparing cost matrix for the Hungarian algorithm...");
//       // negate to convert to cost
//       LL_mat = -1.0*LL_mat;
//       // subtract minimum value to make sure all the elements are positive
//       LL_mat = LL_mat - ( LL_mat.minCoeff()*Eigen::MatrixXd::Ones(LL_mat.rows(), LL_mat.cols()) );
//    }
  
//   std::vector< std::vector<double> > costMat;
//   std::vector<double> row;
//   for (int i=0; i< LL_mat.rows(); i++){
//      row.clear();
//      for(int j=0; j<LL_mat.cols(); j++){
//         row.push_back(LL_mat(i,j));
//      }
//      costMat.push_back(row);
//   }
//   if(debug_){
//      ROS_INFO("[KFTracker::updateTracks] Cost matrix is prepared");
//      std::cout << "costMat: \n";
//       for(int ii=0; ii<costMat.size(); ii++){
//          for(int jj=0; jj<costMat[ii].size(); jj++)
//             std::cout << costMat[ii][jj] << " "  ;
//       }
//       std::cout << "\n";
//   }

//    /** @todo  apply Hungarian algorithm on cost_mat, to get measurement-state assignment */
//    if(debug_)
//       ROS_INFO("[KFTracker::updateTracks] Executing Hungarian algorithm...");
   
//    std::vector<int> assignment; // Assignment vector, has size of tracks_
//    double cost = HungAlgo_.Solve(costMat, assignment);
//    if(debug_){
//       ROS_INFO("[KFTracker::updateTracks] Hungarian algorithm is executed");
//       std::cout << "Assignment vector: \n";
//       for(int i=0; i<assignment.size(); i++){
//          std::cout << assignment[i] << " ";
//       }
//       std::cout << "\n";
//    }

//    // vector to mark the assigned measurements. 1 if assigned, 0 otherwise
//    // This will be used to add the non-assigned measurement(s) as new track(s)
//    // size of z
//    Eigen::VectorXi assigned_z(z.size()); assigned_z = Eigen::VectorXi::Zero(z.size());

//    /** @todo apply KF update step for each track, if it's assigned a measurement, and then predict to the current time step
//     * If a track is not assigned a measurement, just predict it to the current time step
//     * Remove measurements that are already assigned to tracks, after they are used to update their assigned tracks.
//    */

//    if(debug_)
//       ROS_INFO("[KFTracker::updateTracks] Updating tracks using assigned measurements");
//    for(auto it_t=tracks_.begin(); it_t!=tracks_.end(); it_t++){
//       int tr_idx = it_t - tracks_.begin();

      
//       // Apply state correction
//       if (assignment[tr_idx] > -1){

//          // we have to double check the assigned measurement is not bad!
//          // because hungarian algorithm will just do matching without respecting any threshold
//          double LL = kf_model_.logLikelihood((*it_t).current_state, z[assignment[tr_idx]]);
//          // double dist = kf_model_.computeDistance((*it_t).current_state, z[assignment[tr_idx]]);
//          // if(dist <= dist_threshold_){
//          if(LL >= l_threshold_){
//             assigned_z(assignment[tr_idx]) = 1;
//             // correct/update track
//             (*it_t).current_state = kf_model_.updateX(z[assignment[tr_idx]], (*it_t).current_state);
//             (*it_t).n += 1;
//             (*it_t).buffer.push_back((*it_t).current_state);
//             if((*it_t).buffer.size() > state_buffer_size_)
//                (*it_t).buffer.erase((*it_t).buffer.begin());
//          }
//       }

//       // predict track to the current time stamp, if possible
//       double dt = t.toSec() - (*it_t).current_state.time_stamp.toSec();
//       (*it_t).current_state = kf_model_.predictX((*it_t).current_state, dt);
//       (*it_t).buffer.push_back((*it_t).current_state);
//       if((*it_t).buffer.size() > state_buffer_size_)
//          (*it_t).buffer.erase((*it_t).buffer.begin());


//    }// Done updating tracks

//    if(debug_)
//       std::cout << "[updateTracks] assigned_z vector: \n" << assigned_z << "\n";

//    /** @todo  If there are reamining measurements, use add them as new tracks. */
//    if(debug_)
//       ROS_INFO("[KFTracker::updateTracks] Adding new tracks using non-assigned measurements");
//    for( int m=0; m<z.size(); m++){
//       if(assigned_z(m) > 0) continue; // this measurement is assigned, so skip it
//       kf_state state;
//       state.x.resize(kf_model_.numStates(),1);
//       state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
//       state.x.block(0,0,3,1) = z[m].z;
//       state.x.block(3,0,kf_model_.numStates()-3,1) = 0.0*Eigen::MatrixXd::Zero(kf_model_.numStates()-3,1);

//       state.P = kf_model_.Q(dt_pred_);
//       // state.P.block(0,0,3,3) = kf_model_.R();
//       state.time_stamp = z[m].time_stamp;
      
//       kf_track new_track;
//       new_track.id = z[m].id;
//       new_track.current_state = state;
//       new_track.n = 1;
//       new_track.buffer.push_back(state);

//       tracks_.push_back(new_track);
//       if(debug_){
//          ROS_WARN( "******* New track is added using a non-assigned measurement: %d ******* ", m);
//       }      
//    }

//    // DONE

// }


void KFTracker::removeUncertainTracks(){
   if(tracks_.empty())
      return;

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();
      // Calculate position uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matirx
      auto V = sqrt(std::fabs(P_p.determinant()));
      if(debug_)
         ROS_WARN( "[KFTracker::removeUncertainTracks] Track %d uncertainty = %f . number of measurements %d.", i, V, (*it).n);      

      // Remove tracks with high uncertainty
      if(V > V_max_ || isinf( (*it).current_state.x.norm() ))
      {
         if(debug_){
            ROS_WARN( "[KFTracker::removeUncertainTracks] Track %d uncertainty = %f is high (> %f). Removing it.", i, V, V_max_);
            ROS_WARN( "[KFTracker::removeUncertainTracks] Track %d norm(state) = %f", i, (*it).current_state.x.norm());
         }
         
         tracks_.erase(it--);
      }
   }  
}

void KFTracker::updateCertainTracks(void)
{
   if(debug_){
      ROS_INFO("[KFTracker::updateCertainTracks] Number of available tracks = %lu", tracks_.size());
   }

   if(tracks_.empty())
      return;

   certain_tracks_.clear();
   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();
      // Calculate uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matirx
      auto V = sqrt(std::fabs(P_p.determinant()));
      if(debug_)
         ROS_WARN( "[KFTracker::updateCertainTracks] Track %d uncertainty = %f . number of measurements %d.", i, V, (*it).n);
      // If certainty is acceptable, add it to certain_tracks_
      if (V <= V_certain_ && (*it).n >= N_meas_)
      {
         certain_tracks_.push_back((*it));
      }
      else
      {
         if(debug_){
            ROS_WARN("Track is not considered certain. V = %f, N = %d", V, (*it).n);
         }
      }
   }
}

void KFTracker::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
   // if(debug_)
   //    ROS_INFO("[KFTracker::poseArrayCallback] Thred id: %s", std::this_thread::get_id());

   // measurement_set_mtx_.lock();

   measurement_set_.clear();
   
   // Sanity check
   if(msg->poses.empty())
   {
      if(debug_)
         ROS_WARN("[KFTracker::poseArrayCallback]: No measurements received.");

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
            ROS_WARN("[poseArrayCallback] Measurement norm is very large %f", z.z.norm());
         continue;
      }
      measurement_set_.push_back(z);
   }
   if(debug_){
      ROS_INFO("[poseArrayCallback] Size of measurement_set_ : %lu", measurement_set_.size());
      ROS_INFO("[poseArrayCallback] Size of msg->poses : %lu", msg->poses.size());
   }

   // measurement_set_mtx_.unlock();

   return;

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



void KFTracker::publishCertainTracks(void)
{
   if(certain_tracks_.empty()){
      if(debug_)
         ROS_WARN("[KFTracker::publishCertainTracks] certain_tracks_ is empty. No tracks to publish");
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

   good_poses_pub_.publish(pose_array);
   good_tracks_pub_.publish(tracks_msg);
   
   return;
}

void KFTracker::publishAllTracks(void)
{
   if(tracks_.empty()){
      if(debug_)
         ROS_WARN("[KFTracker::publishAllTracks] tracks_ is empty. No tracks to publish");
      return;
   }

   // ROS_WARN("[KFTracker::publishAllTracks] Number of all tracks: %d", tracks_.size());

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

void KFTracker::initTracks(void)
{
   // if(debug_)
   //    ROS_INFO("[KFTracker::initTracks] Thred id: %s", std::this_thread::get_id());

   // measurement_set_mtx_.lock();
   auto z = measurement_set_;

   // measurement_set_mtx_.unlock();

   if(z.empty())
   {
      if(debug_){
         ROS_WARN("No available measurements. Track initialization is skipped.");
      }
      return;
   }

   if(z[0].time_stamp.toSec() <= last_measurement_t_.toSec())
   {
      if(debug_)
         ROS_WARN("No new measurements. Track initilization is skipped.");
      return;
   }

   if(debug_)
      ROS_INFO("[KFTracker::initTracks] Initializing tracks...");

   last_measurement_t_ = z[0].time_stamp;

   for (int i=0; i < z.size(); i++)
   {
      kf_state state;
      state.time_stamp = z[i].time_stamp;
      state.x.resize(kf_model_.numStates(),1);
      state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
      state.x.block(0,0,3,1) = z[i].z; // 3D position
      state.x.block(3,0,kf_model_.numStates()-3,1) = 0.0*Eigen::MatrixXd::Ones(kf_model_.numStates()-3,1);
      
      state.P = kf_model_.Q();//Q(dt_pred_);
      // state.P.block(0,0,3,3) = kf_model_.R();

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
      ROS_INFO("[KFTracker::filterLoop] inside filterLoop...");

   if(tracks_.empty())
   {
      /* Initialize tracks with current measurements. Then, return */
      if(debug_){
         ROS_WARN("No tracks to update. Initializing tracks using current measurements.");
      }
      initTracks();

      return;
   }

   // Do prediction step for all tracks.
   // predictTracks();

   double dt = ros::Time::now().toSec() - last_prediction_t_.toSec();
   predictTracks(dt);
   last_prediction_t_ = ros::Time::now();

   // Do correction step for all tracks using latest measurements.
   updateTracks(ros::Time::now());

   // Publish all available tracks
   publishAllTracks();

   // Extrack good tracks, and remove bad ones
   updateCertainTracks();

   // Publish state estimates pf ggod tracks as PoseArray, and as custom msg of array (KFTracks.msg) of pose with covariance
   publishCertainTracks();

   // Remove bad tracks
   removeUncertainTracks();

   return;
}
