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

KFTracker::KFTracker():
dt_pred_(0.05),// 20Hz
last_prediction_t_(0.0),
last_measurement_t_(0.0),
V_max_(20.0),
V_certain_(1.0),
N_meas_(20),
l_threshold_(-2.0),
dist_threshold_(2.0),
is_state_initialzed_(false),
state_buffer_size_(40),
tracking_frame_("map"),
do_update_step_(true),
measurement_off_time_(2.0),
use_track_id_(false),
sigma_a_(10),
sigma_p_(1),
sigma_v_(1),
track_mesurement_timeout_(3.0),
debug_(false)
{
   
   
}

KFTracker::~KFTracker()
{
}



bool KFTracker::initKF(void)
{

   kf_model_.debug(debug_);
   if(!kf_model_.Q(dt_pred_, sigma_a_)) return false; // initialize Process covariance matrix
   if(!kf_model_.R(r_diag_)) return false; // initialize measurment covariance matrix
   if(!kf_model_.P(sigma_p_, sigma_v_)) return false; // initialize state covariance matrix
   if(!kf_model_.setSigmaA(sigma_a_)) return false;
   if(!kf_model_.setSigmaP(sigma_p_)) return false;
   if(!kf_model_.setSigmaV(sigma_v_)) return false;

   // Clear all buffers
   tracks_.clear();
   certain_tracks_.clear();



   printf("KF is initialized. Waiting for measurements ...");

   return true;
}

void KFTracker::initTracks(void)
{
   // if(debug_)
   //    printf("[KFTracker::initTracks] Thred id: %s", std::this_thread::get_id());

   // Clear all tracks
   tracks_.clear();
   certain_tracks_.clear();

   measurement_set_mtx_.lock();
   auto z = measurement_set_;
   measurement_set_mtx_.unlock();

   if(z.empty())
   {
      if(debug_){
         printf("WARN [initTracks] No available measurements. Track initialization is skipped.\n");
      }
      return;
   }

   if(z[0].time_stamp <= last_measurement_t_)
   {
      if(debug_){
         printf("WARN [initTracks] No new measurements. Track initilization is skipped.\n");
         printf("[initTracks] z[0].time_stamp = %f. last_measurement_t_= %f \n", z[0].time_stamp,last_measurement_t_);
      }
      return;
   }

   if(debug_)
      printf("[KFTracker::initTracks] Initializing tracks...\n");

   last_measurement_t_ = z[0].time_stamp;

   for (long unsigned int i=0; i < z.size(); i++)
   {
      kf_state state;
      state.time_stamp = z[i].time_stamp;
      state.x.resize(kf_model_.numStates(),1);
      state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
      state.x.block(0,0,3,1) = z[i].z; // 3D position
      state.x.block(3,0,kf_model_.numStates()-3,1) = 0.001*Eigen::MatrixXd::Ones(kf_model_.numStates()-3,1);
      
      state.P = kf_model_.P();//Q(dt_pred_);
      // state.P.block(0,0,3,3) = dt_pred_*dt_pred_ * kf_model_.R();

      kf_track track;
      track.n = 1; // Number of measurements = 1 since it's the 1st one
      track.current_state = state;
      track.last_measurement_time = state.time_stamp;
      track.buffer.push_back(state);

      tracks_.push_back(track);
   }
   if(debug_){
      printf("[KFTracker::initTracks] Initialized %lu tracks\n", tracks_.size());
   }

   return;

}

void KFTracker::predictTracks(void)
{
   if(debug_)
      printf("[KFTracker::predictTracks] Predicting tracks...\n");

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();

      if(debug_)
         printf("[KFTracker::predictTracks] Predicting track %d \n", i);

      (*it).current_state = kf_model_.predictX((*it).current_state, dt_pred_);

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());

      if(debug_)
         printf("[KFTracker::predictTracks] Done predicting track %d \n", i);
   }

   if(debug_)
      printf("[KFTracker::predictTracks] Done predicting all tracks. \n");
   
   return;
}

void KFTracker::predictTracks(double dt)
{
   if(debug_)
      printf("[KFTracker::predictTracks] Predicting tracks... \n");

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();

      if(debug_)
         printf("[KFTracker::predictTracks] Predicting track %d \n", i);

      (*it).current_state = kf_model_.predictX((*it).current_state, dt);

      // update buffer
      (*it).buffer.push_back((*it).current_state);
      if((*it).buffer.size() > state_buffer_size_)
         (*it).buffer.erase((*it).buffer.begin());

      if(debug_)
         printf("[KFTracker::predictTracks] Done predicting track %d \n", i);
   }

   if(debug_)
      printf("[KFTracker::predictTracks] Done predicting all tracks. \n");
   
   return;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void KFTracker::updateTracks(double t)
{
   // if(debug_)
   //    printf("[KFTracker::updateTracks] Thread id: %s", std::this_thread::get_id());

   // Sanity checks

   if(tracks_.empty())
      return;

   measurement_set_mtx_.lock();
   auto z = measurement_set_;
   measurement_set_mtx_.unlock();

   if(z.empty())
   {
      if(debug_)
         printf("WARN [updateTracks] No available measurements. Skipping update step. \n");
      return;
   }

    // check if we got new measurement
    /** @warning Currently, it's assumed that all measurement in the buffer measurement_set_ have the same time stamp */
    auto z_t = z[0].time_stamp;
   if (z_t <= last_measurement_t_)
   {
      if(debug_)
         printf("WARN [updateTracks] No new measurement. Skipping KF update step. \n");
      return;
   }
   last_measurement_t_ = z[0].time_stamp;

   /** @brief compute loglikelihood matrix to be passed to the Hungarian algorithm */
   if(debug_)
      printf("[KFTracker::updateTracks] Computing loglikelihood matrix... \n");
   Eigen::MatrixXd LL_mat(tracks_.size(), z.size());
   for(auto it_t=tracks_.begin(); it_t != tracks_.end(); it_t++){
      int tr_idx = it_t - tracks_.begin(); // track index
      /** @todo Should we predict the state to the current measurement time??? */

      // First, check if there is close state in time
      bool found_closest_state = false;
      for (int k=(*it_t).buffer.size()-1 ; k >=0 ; k--) // start from the last state in the buffer, should have the biggest/latest time stamp
      {
         auto x_t = (*it_t).buffer[k].time_stamp; // time of the k-th state in the state-buffer of this track
         if( z_t >= x_t)
         {
            // make the closest state the current_state, and remove all the future ones if any
            (*it_t).current_state.time_stamp = (*it_t).buffer[k].time_stamp;
            (*it_t).current_state.x = (*it_t).buffer[k].x;
            (*it_t).current_state.P = (*it_t).buffer[k].P;
            (*it_t).buffer.clear(); (*it_t).buffer.push_back((*it_t).current_state);
            found_closest_state = true;

            break;
         }
      }

      for(auto it_z=z.begin(); it_z != z.end(); it_z++){
         int z_idx = it_z - z.begin(); // measurement index
         /** @todo check for INF values */ 
         double LL = kf_model_.logLikelihood((*it_t).current_state, (*it_z));
         if ( (LL >= l_threshold_) && found_closest_state) LL_mat(tr_idx, z_idx) = LL;
         else  LL_mat(tr_idx, z_idx) = -9999.0;

         if(debug_)
            printf("[KFTracker::updateTracks] LL_mat(%d,%d) = %f \n", tr_idx, z_idx, LL_mat(tr_idx, z_idx));
      }

   } // done looping over tracks & measurements, and computing the LL_mat

   if(debug_){
      printf("[KFTracker::updateTracks] Done computing loglikelihood matrix \n");
      std::cout << "loglikelihood matrix: \n" << LL_mat << "\n";
   }

   /** @brief Post-process the cost matrix
    * 1. Hungarian algorithm minimizes cost, so first negate the logliklihood matrix
    * 2. Hungarian algorithm requires cost matrix with non negative elements only.
    *    So, subtract the minimum element from the matrix resulting from step 1
   */
   if (true){ //set to true if using loglikelihood instead of distance
      if(debug_)
         printf("[KFTracker::updateTracks] Preparing cost matrix for the Hungarian algorithm... \n");
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
     printf("[KFTracker::updateTracks] Cost matrix is prepared.\n");
     std::cout << "costMat: \n";
      for(long unsigned int ii=0; ii<costMat.size(); ii++){
         for(long unsigned int jj=0; jj<costMat[ii].size(); jj++)
            std::cout << costMat[ii][jj] << " "  ;
      }
      std::cout << "\n";
  }

   /** @brief  apply Hungarian algorithm on cost_mat, to get measurement-state assignment */
   if(debug_)
      printf("[KFTracker::updateTracks] Executing Hungarian algorithm... \n");
   
   std::vector<int> assignment; // Assignment vector, has size of tracks_
   double cost = HungAlgo_.Solve(costMat, assignment);
   if(debug_){
      printf("[KFTracker::updateTracks] Hungarian algorithm is executed. cost = %f \n", cost);
      std::cout << "Assignment vector: \n";
      for(long unsigned int i=0; i<assignment.size(); i++){
         std::cout << assignment[i] << " ";
      }
      std::cout << "\n";
   }

   // vector to mark the assigned measurements. 1 if assigned, 0 otherwise
   // This will be used to add the non-assigned measurement(s) as new track(s)
   // size of z
   Eigen::VectorXi assigned_z(z.size()); assigned_z = Eigen::VectorXi::Zero(z.size());

   /** @brief apply KF update step for each track, if it's assigned a measurement, and then predict to the current time step
    * If a track is not assigned a measurement, just predict it to the current time step
    * Remove measurements that are already assigned to tracks, after they are used to update their assigned tracks.
   */

   if(debug_)
      printf("[KFTracker::updateTracks] Updating tracks using assigned measurements \n");
   for(auto it_t=tracks_.begin(); it_t!=tracks_.end(); it_t++){
      int tr_idx = it_t - tracks_.begin();

      
      // Apply state correction
      if (assignment[tr_idx] > -1){

         // we have to double check the assigned measurement is not bad!
         // because hungarian algorithm will just do matching without respecting any threshold
         double LL = kf_model_.logLikelihood((*it_t).current_state, z[assignment[tr_idx]]);
         if(LL >= l_threshold_){
            assigned_z(assignment[tr_idx]) = 1;
            // correct/update track
            double dt2 = z[assignment[tr_idx]].time_stamp -  (*it_t).last_measurement_time;
            dt2 *= dt2; // square it
            (*it_t).current_state = kf_model_.updateX(z[assignment[tr_idx]], (*it_t).current_state);
            (*it_t).n += 1;
            (*it_t).last_measurement_time = z[assignment[tr_idx]].time_stamp; 
         }
      }

      // predict track to the current time stamp, if possible
      double dt = t - (*it_t).current_state.time_stamp;
      (*it_t).current_state = kf_model_.predictX((*it_t).current_state, dt);
      (*it_t).buffer.push_back((*it_t).current_state);
      if((*it_t).buffer.size() > state_buffer_size_)
         (*it_t).buffer.erase((*it_t).buffer.begin());


   }// Done updating tracks

   if(debug_)
      std::cout << "[updateTracks] assigned_z vector: \n" << assigned_z << "\n";

   /** @todo  If there are reamining measurements, add them as new tracks. */
   if(debug_)
      printf("[KFTracker::updateTracks] Adding new tracks using non-assigned measurements \n");
   for( long unsigned int m=0; m<z.size(); m++){
      if(assigned_z(m) > 0) continue; // this measurement is assigned, so skip it
      kf_state state;
      state.x.resize(kf_model_.numStates(),1);
      state.x = Eigen::MatrixXd::Zero(kf_model_.numStates(),1);
      state.x.block(0,0,3,1) = z[m].z;
      state.x.block(3,0,kf_model_.numStates()-3,1) = Eigen::MatrixXd::Zero(kf_model_.numStates()-3,1);

      state.P = kf_model_.Q(dt_pred_);//kf_model_.P(); //kf_model_.Q(dt_pred_);
      // state.P.block(0,0,3,3) = kf_model_.R();
      state.time_stamp = z[m].time_stamp;
      
      kf_track new_track;
      new_track.id = z[m].id;
      new_track.current_state = state;
      new_track.n = 1;
      new_track.last_measurement_time = state.time_stamp;
      // new_track.buffer.push_back(state);

      tracks_.push_back(new_track);
      if(debug_){
         printf( "WARN [KFTracker::updateTracks] New track is added using a non-assigned measurement of ID: %lu ******* \n ", m);
      }      
   }

}// updateTracks DONE 

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
         printf( "WARN [KFTracker::removeUncertainTracks] Track %d uncertainty = %f . number of measurements %d. \n", i, V, (*it).n);      

      // Remove tracks with high uncertainty
      if(V > V_max_ || isinf( (*it).current_state.x.norm() ))
      {
         if(debug_){
            printf( "WARN [KFTracker::removeUncertainTracks] Track %d uncertainty = %f is high (> %f). Removing it.\n", i, V, V_max_);
            printf( "WARN [KFTracker::removeUncertainTracks] Track %d norm(state) = %f", i, (*it).current_state.x.norm());
         }
         
         tracks_.erase(it--);
         continue;
      }

      // Remove track if it has not received measurements for long time
      if(abs((*it).current_state.time_stamp - (*it).last_measurement_time) > track_mesurement_timeout_)
      {
         tracks_.erase(it--);
      }
   }  
}

void
KFTracker::updateCertainTracks(void)
{
   if(debug_){
      printf("[KFTracker::updateCertainTracks] Number of available tracks = %lu \n", tracks_.size());
   }

   certain_tracks_.clear();
   if(tracks_.empty())
      return;

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();
      // Calculate uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matirx
      auto V = sqrt(std::fabs(P_p.determinant()));
      if(debug_)
         printf( "WARN [KFTracker::updateCertainTracks] Track %d uncertainty = %f . number of measurements %d.\n", i, V, (*it).n);
      // If certainty is acceptable, add it to certain_tracks_
      if (V <= V_certain_ && (*it).n >= (unsigned int)N_meas_)
      {
         certain_tracks_.push_back((*it));
      }
      else
      {
         if(debug_){
            printf("WARN Track is not considered certain. V = %f, N = %d \n", V, (*it).n);
         }
      }
   }
}

/**
 * @param t current time
*/
void
KFTracker::filterLoop(double t)
{
   if(debug_)
      printf("[KFTracker::filterLoop] inside filterLoop... \n");

   if(tracks_.empty())
   {
      /* Initialize tracks with current measurements. Then, return */
      if(debug_){
         printf("WARN [KFTracker::filterLoop] No tracks to update. Initializing tracks using current measurements.\n");
      }
      initTracks();

      return;
   }

   // Do prediction step for all tracks.
   // predictTracks();

   double dt = t - last_prediction_t_;
   predictTracks(dt);
   last_prediction_t_ = t;

   // Do correction step for all tracks using latest measurements.
   updateTracks(t);

   // Extrack good tracks
   updateCertainTracks();

   // Remove bad tracks
   removeUncertainTracks();

   return;
}


