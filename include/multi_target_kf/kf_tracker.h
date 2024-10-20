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
#ifndef KF_TRACKER_H
#define KF_TRACKER_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <math.h>  // log

#include "multi_target_kf/models.h"

#include "multi_target_kf/hungarian.h" // For state measurement assignment

#include <mutex>          // std::mutex
#include <thread> // std::this_thread::get_id()

//using namespace std;
// using namespace Eigen;

enum MODEL : unsigned char
{
   CONSTANT_ACCEL = 0, COORDINATED_TURN = 1
};

/**
 * Structure to store the current stamped KF prediction and buffer of previous state
 */
struct kf_track
{
   unsigned int id; /**< Unique track ID, e.g. Apriltag ID. Also, useful for object association */
   std::string frame_id;
   kf_state current_state;
   std::vector<kf_state> buffer;
   unsigned int n; /**< Number of received measurements. */
   double last_measurement_time;
};

//* KFTracker class
/**
 * Implements a Kalman-filter-based object tracker based on constant velocity model
 * Reference 1: Constant velocity model (http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf)
 * Reference 2: Discrete KF equations (https://en.wikipedia.org/wiki/Kalman_filter)
 */
class KFTracker
{
private:
   // DubinsModel kf_model_; /* 3D Dubins EKF model */

   HungarianAlgorithm HungAlgo_; /** Hungarian algorithm object for state-measurement assignment */
public:
   // ConstantVelModel kf_model_; /* Constant velocity KF model */
   ConstantAccelerationModel  kf_model_; /* Constant acceleration KF model */

   double dt_pred_;
   double last_prediction_t_;

   std::vector<kf_track> tracks_; /**< Vector of current tracks. */
   std::vector<kf_track> certain_tracks_; /**< Vector of certain tracks only. */

   std::vector<sensor_measurement> measurement_set_; /**< set of all measurements. */
   double last_measurement_t_; /**<  Time of the last received measurement*/

   double V_max_; /**< minimum uncertainty before rejecting a track [m^3] */
   double V_certain_; /**< maximum uncertainty for a track to be considered certain [m^3] */
   int N_meas_; /**< minimum number of measurements to accept a track and add it to certain_tracks_. */
   double l_threshold_; /**< measurement association log-likelihood threshold. */

   double dist_threshold_; /** Maximum distance between  a state & measurement to consider them as a match */

   kf_state kf_state_pred_; /**< KF predicted state and covariance */
   
   bool is_state_initialzed_; /**< flag to start state prediction. Initialized by 1st measurement. */
   std::vector<kf_state> state_buffer_; /**< Bueffer to store last state_buffer_size_ predicted x and P */
   unsigned int state_buffer_size_; /**< lenght of state buffer state_buffer_*/
   std::string tracking_frame_; /**< Coordinate frame at which tracking is performed */
   bool do_update_step_; /**< Whether to perform the KF update step. WARNING this is just for debuggin. */
   double measurement_off_time_; /**< Maximum time (in seconds) with no measurement before filter is stopped. */
   bool use_track_id_; /**< False: does not consider track ID in measurement-state association */

   std::vector<double> q_diag_; /* diagonal elements of Q matrix */
   std::vector<double> r_diag_; /* diagonal elements of R matrix */

   double sigma_a_; /* Standard deviation of acceleration noise */
   double sigma_p_; /* Standard deviataion of the position. Used in the initial  state covariance matrix P*/
   double sigma_v_; /* Standard deviataion of the velocity. Used in the initial  state covariance matrix P*/
   double sigma_theta_; /* Standard deviation of heading angle in radians */
   double sigma_gamma_; /* Standard deviation of climb angle in radians*/
   double sigma_omega_; /* Standard deviation of turn rate in rad/sec */


   std::mutex measurement_set_mtx_; /* mutex to guard measurement_set_  from interferring calls */

   double track_mesurement_timeout_; /* maximum time (seconds) from last measurement before considering a track uncertain and removing it */

   

   bool debug_; /**< for printing debug message */

   /**
    * @brief Initializes KF F,H,Q,R, and initial state and covariance estimates
    */
   bool initKF(void);

   /**
    * @brief Initializes KF tracks using current measurements.
    */
   void initTracks(void);


   void predictTracks(void);

   void predictTracks(double dt);

   /**
    * @brief Performs KF update step for all tracks. 
    * @param t : current time in seconds
    */
   void updateTracks(double t);

   /**
    * @brief Extract tracks with high certainty from the current tracks.
    * Uses tracks_ and updates certain_tracks_ 
    */
   void updateCertainTracks(void);

   /**
    * @brief Removes tracks  (from tracks_ ) with position uncertainty > V_max__ 
    */
   void removeUncertainTracks();

   /**
    * @brief Executes one loop of the KF filter
    * @param t [double] current time
   */
   void filterLoop(double t);



   KFTracker();
   ~KFTracker();
};

#endif