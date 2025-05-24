/*
BSD 3-Clause License

Copyright (c) 2022, Mohamed Abdelkader Zahana
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
#include <mutex>   // std::mutex
#include <thread>  // std::this_thread::get_id()
#include <limits>

#include "multi_target_kf/motion_model.h"
#include "multi_target_kf/model_factory.h"
#include "multi_target_kf/hungarian.h" // For state measurement assignment
#include "multi_target_kf/tracker_config.h"

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
 * Implements a Kalman-filter-based object tracker based on various motion models
 */
class KFTracker
{
private:
   TrackerConfig config_;           /**< Configuration settings */
   HungarianAlgorithm HungAlgo_;    /**< Hungarian algorithm object for state-measurement assignment */
   unsigned int next_track_id_;     /**< Counter for assigning unique track IDs when use_track_id is false */
protected:
   MotionModel* kf_model_;          /**< Pointer to the motion model being used */
   std::mutex measurement_set_mtx_; /**< mutex to guard measurement_set_ from interfering calls */
public:
   // Add a friend declaration for TrackerROS
   friend class TrackerROS;

   // These public fields are maintained for backward compatibility.
   // New code should use the config_ object instead.
   double dt_pred_;
   double last_prediction_t_;
   double last_measurement_t_;
   double V_max_;
   double V_certain_;
   int N_meas_;
   double l_threshold_;
   double dist_threshold_;
   bool is_state_initialzed_;
   unsigned int state_buffer_size_;
   std::string tracking_frame_;
   bool do_update_step_;
   double measurement_off_time_;
   bool use_track_id_;
   std::vector<double> q_diag_;
   std::vector<double> r_diag_;
   double sigma_a_;
   double sigma_p_;
   double sigma_v_;
   double sigma_j_; /* Standard deviation of jerk noise (for constant acceleration) - NEW */
   double track_measurement_timeout_;
   bool debug_;

   std::vector<kf_track> tracks_;                     /**< Vector of current tracks. */
   std::vector<kf_track> certain_tracks_;             /**< Vector of certain tracks only. */
   std::vector<sensor_measurement> measurement_set_;  /**< set of all measurements. */
   kf_state kf_state_pred_;                           /**< KF predicted state and covariance */
   std::vector<kf_state> state_buffer_;               /**< Buffer to store last state_buffer_size_ predicted x and P */

   /**
    * @brief Initializes KF F,H,Q,R, and initial state and covariance estimates
    */
   bool initKF(void);

   /**
    * @brief Initializes KF tracks using current measurements.
    */
   void initTracks(void);

   /**
    * @brief Predicts the state of all tracks
    */
   void predictTracks(void);

   /**
    * @brief Predicts the state of all tracks using the specified time step
    * @param dt Time step in seconds
    */
   void predictTracks(double dt);

   /**
    * @brief Performs KF update step for all tracks. 
    * @param t Current time in seconds
    */
   void updateTracks(double t);

   /**
    * @brief Extract tracks with high certainty from the current tracks.
    * Uses tracks_ and updates certain_tracks_ 
    */
   void updateCertainTracks(void);

   /**
    * @brief Removes tracks (from tracks_) with position uncertainty > V_max_ 
    */
   void removeUncertainTracks();

   /**
    * @brief Executes one loop of the KF filter
    * @param t Current time in seconds
    */
   void filterLoop(double t);

   /**
    * @brief Constructor
    * @param config Configuration settings (default: use default TrackerConfig)
    */
   KFTracker(const TrackerConfig& config = TrackerConfig());
   
   /**
    * @brief Legacy constructor that accepts only model type
    * @param model_type Type of motion model to use
    */
   KFTracker(ModelType model_type);
   
   /**
    * @brief Destructor
    */
   ~KFTracker();

   /**
    * @brief Get the current configuration
    * @return Current configuration
    */
   const TrackerConfig& getConfig() const { return config_; }
   
   /**
    * @brief Set a new configuration
    * @param config New configuration
    * @return true if successful, false otherwise
    */
   bool setConfig(const TrackerConfig& config);
   
   /**
    * @brief Get the model type currently in use
    * @return ModelType enum value
    */
   ModelType getModelType() const { return config_.model_type; }
   
   /**
    * @brief Set a new motion model
    * @param model_type Type of model to use
    * @return true if successful, false otherwise
    */
   bool setModel(ModelType model_type);

   /**
    * @brief Synchronize the configuration with the current values of the public fields
    * This is useful for backward compatibility when the public fields are modified directly
    */
   void syncConfigFromFields();

   /**
    * @brief Update the public fields from the current configuration
    * This is useful for backward compatibility when the configuration is modified
    */
   void syncFieldsFromConfig();

   /**
    * @brief Get the next available track ID
    * @return Next unique track ID
    */
   unsigned int getNextTrackId();
};

#endif // KF_TRACKER_H