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

#include "multi_target_kf/kf_tracker.h"

KFTracker::KFTracker(const TrackerConfig& config) :
   config_(config),
   next_track_id_(1),
   kf_model_(nullptr),
   last_prediction_t_(0.0),
   last_measurement_t_(0.0),
   is_state_initialzed_(false)
{
   // Update public fields from the configuration for backward compatibility
   syncFieldsFromConfig();
   
   // Create the motion model
   kf_model_ = ModelFactory::createModel(config_.model_type);
   if (!kf_model_ && debug_) {
      printf("ERROR: Failed to create motion model of type %d\n", static_cast<int>(config_.model_type));
   }
}

KFTracker::KFTracker(ModelType model_type) :
   KFTracker(TrackerConfig())
{
   // Override the model type in the configuration
   config_.model_type = model_type;
   next_track_id_ = 1;  // Start from ID 1
   
   // Clean up the default model
   if (kf_model_) {
      delete kf_model_;
      kf_model_ = nullptr;
   }
   
   // Create the specified model
   kf_model_ = ModelFactory::createModel(model_type);
   if (!kf_model_ && debug_) {
      printf("ERROR: Failed to create motion model of type %d\n", static_cast<int>(model_type));
   }
}

KFTracker::~KFTracker()
{
   // Clean up the model
   if (kf_model_) {
      delete kf_model_;
      kf_model_ = nullptr;
   }
}

unsigned int KFTracker::getNextTrackId()
{
   return next_track_id_++;
}

void KFTracker::syncFieldsFromConfig()
{
   dt_pred_ = config_.dt_pred;
   V_max_ = config_.V_max;
   V_certain_ = config_.V_certain;
   N_meas_ = config_.N_meas;
   l_threshold_ = config_.l_threshold;
   dist_threshold_ = config_.dist_threshold;
   state_buffer_size_ = config_.state_buffer_size;
   tracking_frame_ = config_.tracking_frame;
   do_update_step_ = config_.do_update_step;
   measurement_off_time_ = config_.measurement_off_time;
   use_track_id_ = config_.use_track_id;
   track_measurement_timeout_ = config_.track_measurement_timeout;
   debug_ = config_.debug;
   sigma_a_ = config_.sigma_a;
   sigma_p_ = config_.sigma_p;
   sigma_v_ = config_.sigma_v;
   sigma_j_ = config_.sigma_j;  // NEW: Initialize sigma_j_ from config
   q_diag_ = config_.q_diag;
   r_diag_ = config_.r_diag;
}

void KFTracker::syncConfigFromFields()
{
   config_.dt_pred = dt_pred_;
   config_.V_max = V_max_;
   config_.V_certain = V_certain_;
   config_.N_meas = N_meas_;
   config_.l_threshold = l_threshold_;
   config_.dist_threshold = dist_threshold_;
   config_.state_buffer_size = state_buffer_size_;
   config_.tracking_frame = tracking_frame_;
   config_.do_update_step = do_update_step_;
   config_.measurement_off_time = measurement_off_time_;
   config_.use_track_id = use_track_id_;
   config_.track_measurement_timeout = track_measurement_timeout_;
   config_.debug = debug_;
   config_.sigma_a = sigma_a_;
   config_.sigma_p = sigma_p_;
   config_.sigma_v = sigma_v_;
   config_.sigma_j = sigma_j_;
   config_.q_diag = q_diag_;
   config_.r_diag = r_diag_;
}

bool KFTracker::setConfig(const TrackerConfig& config)
{
   // Make a copy of the old configuration in case we need to rollback
   TrackerConfig old_config = config_;
   
   // Update the configuration
   config_ = config;
   
   // Update the public fields for backward compatibility
   syncFieldsFromConfig();
   
   // If the model type has changed, we need to create a new model
   if (kf_model_ && config_.model_type != old_config.model_type) {
      // Try to set the new model
      if (!setModel(config_.model_type)) {
         // If setting the new model fails, rollback to the old configuration
         config_ = old_config;
         syncFieldsFromConfig();
         return false;
      }
   }
   
   // Reinitialize the KF with the new parameters
   if (!initKF()) {
      // If initialization fails, rollback to the old configuration
      config_ = old_config;
      syncFieldsFromConfig();
      return false;
   }
   
   return true;
}

bool KFTracker::setModel(ModelType model_type)
{
   // Update the configuration
   config_.model_type = model_type;
   
   // Clean up the old model
   if (kf_model_) {
      delete kf_model_;
      kf_model_ = nullptr;
   }
   
   // Create the new model
   kf_model_ = ModelFactory::createModel(model_type);
   if (!kf_model_) {
      if (debug_) {
         printf("ERROR: Failed to create motion model of type %d\n", static_cast<int>(model_type));
      }
      return false;
   }
   
   // Initialize the new model
   return initKF();
}

bool KFTracker::initKF(void)
{
   if (!kf_model_) {
      if (debug_) {
         printf("ERROR: No motion model available for initialization\n");
      }
      return false;
   }

   kf_model_->debug(debug_);
   
   // Model-specific initialization
   if (config_.model_type == CONSTANT_VELOCITY) {
      // Cast to ConstantVelModel for model-specific initialization
      ConstantVelModel* model = static_cast<ConstantVelModel*>(kf_model_);
      if (!model->Q(dt_pred_, sigma_a_)) return false;
      if (!model->setSigmaA(sigma_a_)) return false;
      if (!model->setSigmaP(sigma_p_)) return false;
      if (!model->setSigmaV(sigma_v_)) return false;
   }
   else if (config_.model_type == CONSTANT_ACCELERATION) {
      // Cast to ConstantAccelModel for model-specific initialization
      ConstantAccelModel* model = static_cast<ConstantAccelModel*>(kf_model_);
      if (!model->Q(dt_pred_, sigma_j_)) return false; // Now using sigma_j_ instead of sigma_a_
      if (!model->setSigmaJ(sigma_j_)) return false;   // Now using sigma_j_ instead of sigma_a_
      if (!model->setSigmaP(sigma_p_)) return false;
      if (!model->setSigmaV(sigma_v_)) return false;
      if (!model->setSigmaA(sigma_a_)) return false;       // Default acceleration std
      
      // Setup initial P matrix
      if (!model->P(sigma_p_, sigma_v_, sigma_a_)) return false;
   }
   else if (config_.model_type == ADAPTIVE_ACCEL_UKF) {
      // Cast to AdaptiveAccelUKF for model-specific initialization
      AdaptiveAccelUKF* model = static_cast<AdaptiveAccelUKF*>(kf_model_);
      
      // Set UKF parameters
      if (!model->setAlpha(config_.alpha)) return false;
      if (!model->setBeta(config_.beta)) return false;
      if (!model->setKappa(config_.kappa)) return false;
      
      // Set adaptive noise parameters
      if (!model->setJerkStd(config_.jerk_std)) return false;
      if (!model->setJerkAdaptiveMax(config_.jerk_adaptive_max)) return false;
      if (!model->setAdaptiveThreshold(config_.adaptive_threshold)) return false;
      if (!model->setAdaptiveDecay(config_.adaptive_decay)) return false;
      if (!model->setInnovationWindowSize(config_.innovation_window_size)) return false;
      
      // Set initial uncertainties
      if (!model->setSigmaP(config_.sigma_p)) return false;
      if (!model->setSigmaV(config_.sigma_v)) return false;
      if (!model->setSigmaA(config_.sigma_a)) return false;
      
      // Setup initial P matrix
      if (!model->P(config_.sigma_p, config_.sigma_v, config_.sigma_a)) return false;
    }
   // Add more model-specific initializations here for future models
   
   // Common initialization for all models
   if (!kf_model_->R(r_diag_)) return false;
   
   // Clear all buffers
   tracks_.clear();
   certain_tracks_.clear();
   
   if (debug_) {
      printf("KF is initialized with model type %s. Waiting for measurements...\n", 
             ModelFactory::getModelName(config_.model_type));
   } else {
      printf("KF is initialized. Waiting for measurements...");
   }
   
   return true;
}

void KFTracker::initTracks(void)
{
    tracks_.clear();
    certain_tracks_.clear();

    // Check if we have detection measurements first
    measurement_set_mtx_.lock();
    auto detections = detection_set_;
    auto basic_measurements = measurement_set_;
    measurement_set_mtx_.unlock();

    if (!detections.empty()) {
        initTracksFromDetections();
    } else if (!basic_measurements.empty()) {
        initTracksFromBasicMeasurements();
    } else {
        if(debug_) {
            printf("WARN [initTracks] No available measurements. Track initialization is skipped.\n");
        }
        return;
    }
}

void KFTracker::initTracksFromDetections(void)
{
    measurement_set_mtx_.lock();
    auto detections = detection_set_;
    measurement_set_mtx_.unlock();

    if(detections.empty()) return;

    if(debug_) printf("[KFTracker::initTracksFromDetections] Initializing tracks...\n");

    last_measurement_t_ = detections[0].time_stamp;

    for (size_t i = 0; i < detections.size(); i++) {
        // Convert to basic measurement for KF initialization
        sensor_measurement basic_meas = toSensorMeasurement(detections[i]);
        kf_state state = kf_model_->initStateFromMeasurements(basic_meas);
        
        kf_track track;
        track.n = 1;
        track.current_state = state;
        track.last_measurement_time = state.time_stamp;
        track.buffer.push_back(state);
        
        // Copy detection information
        track.class_name = detections[i].class_name;
        track.confidence = detections[i].confidence;
        track.track_score = detections[i].confidence;
        
        // Copy bounding box information
        track.has_2d_bbox = detections[i].has_2d_bbox;
        if (track.has_2d_bbox) {
            track.bbox_2d_center_x = detections[i].bbox_2d_center_x;
            track.bbox_2d_center_y = detections[i].bbox_2d_center_y;
            track.bbox_2d_width = detections[i].bbox_2d_width;
            track.bbox_2d_height = detections[i].bbox_2d_height;
        }
        
        track.has_3d_bbox = detections[i].has_3d_bbox;
        if (track.has_3d_bbox) {
            track.bbox_3d_center = detections[i].bbox_3d_center;
            track.bbox_3d_size = detections[i].bbox_3d_size;
            track.bbox_3d_orientation = detections[i].bbox_3d_orientation;
        }
        
        track.attributes = detections[i].attributes;
        
        // Assign track ID
        if (use_track_id_ && detections[i].id != 0) {
            track.id = detections[i].id;
        } else {
            track.id = getNextTrackId();
        }

        tracks_.push_back(track);
        
        if(debug_) {
            printf("[KFTracker::initTracksFromDetections] Initialized track with ID: %u, class: %s\n", 
                   track.id, track.class_name.c_str());
        }
    }
}

void KFTracker::initTracksFromBasicMeasurements(void)
{
    measurement_set_mtx_.lock();
    auto measurements = measurement_set_;
    measurement_set_mtx_.unlock();

    if(measurements.empty()) return;

    if(debug_) printf("[KFTracker::initTracksFromBasicMeasurements] Initializing tracks...\n");

    last_measurement_t_ = measurements[0].time_stamp;

    for (size_t i = 0; i < measurements.size(); i++) {
        kf_state state = kf_model_->initStateFromMeasurements(measurements[i]);
        
        kf_track track;  // Constructor sets default values
        track.n = 1;
        track.current_state = state;
        track.last_measurement_time = state.time_stamp;
        track.buffer.push_back(state);
        
        // Default values already set by constructor:
        // class_name = "unknown", confidence = 1.0, etc.
        
        // Assign track ID
        if (use_track_id_ && measurements[i].id != 0) {
            track.id = measurements[i].id;
        } else {
            track.id = getNextTrackId();
        }

        tracks_.push_back(track);
        
        if(debug_) {
            printf("[KFTracker::initTracksFromBasicMeasurements] Initialized track with ID: %u\n", track.id);
        }
    }
}

void KFTracker::updateTrackFromDetection(kf_track& track, const enhanced_measurement& detection)
{
    // Update class and confidence (with smoothing)
    if (track.class_name == "unknown" || track.class_name.empty()) {
        track.class_name = detection.class_name;
        track.confidence = detection.confidence;
    } else if (track.class_name == detection.class_name) {
        // Smooth confidence over time
        track.confidence = 0.7 * track.confidence + 0.3 * detection.confidence;
    }
    
    // Update bounding boxes (take the latest detection)
    track.has_2d_bbox = detection.has_2d_bbox;
    if (detection.has_2d_bbox) {
        track.bbox_2d_center_x = detection.bbox_2d_center_x;
        track.bbox_2d_center_y = detection.bbox_2d_center_y;
        track.bbox_2d_width = detection.bbox_2d_width;
        track.bbox_2d_height = detection.bbox_2d_height;
    }

    track.has_3d_bbox = detection.has_3d_bbox;
    if (detection.has_3d_bbox) {
        track.bbox_3d_center = detection.bbox_3d_center;
        track.bbox_3d_size = detection.bbox_3d_size;
        track.bbox_3d_orientation = detection.bbox_3d_orientation;
    }

    // Update attributes
    track.attributes = detection.attributes;
    
    // Update track score based on detection confidence
    track.track_score = 0.8 * track.track_score + 0.2 * detection.confidence;
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

      // Use the motion model interface for prediction
      (*it).current_state = kf_model_->predictX((*it).current_state, dt_pred_);

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

      // Use the motion model interface for prediction
      (*it).current_state = kf_model_->predictX((*it).current_state, dt);

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

void KFTracker::updateTracks(double t)
{
    // Sanity checks
    if(tracks_.empty()) return;
    if (!do_update_step_) {
        printf("WARN [updateTracks] Update step is disabled via configuration. Skipping update step.\n");
        return;
    }

    // Get both types of measurements
    measurement_set_mtx_.lock();
    auto detections = detection_set_;
    auto basic_measurements = measurement_set_;
    measurement_set_mtx_.unlock();

    if (!detections.empty()) {
        if(debug_) printf("[updateTracks] Using detection measurements with additional information\n");
        updateTracksFromDetections(t);
    } else if (!basic_measurements.empty()) {
        if(debug_) printf("[updateTracks] Using basic pose measurements\n");
        updateTracksFromBasicMeasurements(t);
    } else {
        if(debug_) printf("WARN [updateTracks] No available measurements. Skipping update step.\n");
        return;
    }
}

void KFTracker::updateTracksFromDetections(double t)
{
    // Get detection measurements
    measurement_set_mtx_.lock();
    auto detections = detection_set_;
    measurement_set_mtx_.unlock();

    if(detections.empty()) {
        if(debug_) printf("WARN [updateTracksFromDetections] No detection measurements available\n");
        return;
    }

    // Check if we got new measurements
    auto z_t = detections[0].time_stamp;
    if (z_t <= last_measurement_t_) {
        if(debug_) printf("WARN [updateTracksFromDetections] No new measurements. Skipping update step.\n");
        return;
    }
    last_measurement_t_ = detections[0].time_stamp;

    // Convert detections to basic measurements for Hungarian algorithm
    std::vector<sensor_measurement> basic_measurements;
    for (const auto& detection : detections) {
        basic_measurements.push_back(toSensorMeasurement(detection));
    }

    if(debug_) printf("[updateTracksFromDetections] Processing %lu detections\n", detections.size());

    // Compute loglikelihood matrix for Hungarian algorithm
    if(debug_) printf("[updateTracksFromDetections] Computing loglikelihood matrix...\n");
    
    Eigen::MatrixXd LL_mat(tracks_.size(), basic_measurements.size());
    for(auto it_t = tracks_.begin(); it_t != tracks_.end(); it_t++){
        int tr_idx = it_t - tracks_.begin();

        // Find closest state in time
        bool found_closest_state = false;
        for (int k = (*it_t).buffer.size()-1; k >= 0; k--) {
            auto x_t = (*it_t).buffer[k].time_stamp;
            if(z_t >= x_t) {
                (*it_t).current_state.time_stamp = (*it_t).buffer[k].time_stamp;
                (*it_t).current_state.x = (*it_t).buffer[k].x;
                (*it_t).current_state.P = (*it_t).buffer[k].P;
                (*it_t).buffer.clear(); 
                (*it_t).buffer.push_back((*it_t).current_state);
                found_closest_state = true;
                break;
            }
        }

        for(auto it_z = basic_measurements.begin(); it_z != basic_measurements.end(); it_z++){
            int z_idx = it_z - basic_measurements.begin();

            // Check track ID matching if enabled
            if (use_track_id_ && (*it_t).id != 0 && (*it_z).id != 0 && (*it_t).id != (*it_z).id) {
                LL_mat(tr_idx, z_idx) = -9999.0;
                if(debug_) printf("[updateTracksFromDetections] Track ID %u != Detection ID %u. Setting low likelihood.\n", 
                                 (*it_t).id, (*it_z).id);
                continue;
            }

            double LL = kf_model_->logLikelihood((*it_t).current_state, (*it_z));
            if ((LL >= l_threshold_) && found_closest_state) {
                LL_mat(tr_idx, z_idx) = LL;
            } else {
                LL_mat(tr_idx, z_idx) = -9999.0;
            }

            if(debug_) printf("[updateTracksFromDetections] LL_mat(%d,%d) = %f\n", tr_idx, z_idx, LL_mat(tr_idx, z_idx));
        }
    }

    if(debug_){
        printf("[updateTracksFromDetections] Done computing loglikelihood matrix\n");
        std::cout << "loglikelihood matrix:\n" << LL_mat << "\n";
    }

    // Prepare cost matrix for Hungarian algorithm (negate and make positive)
    if(debug_) printf("[updateTracksFromDetections] Preparing cost matrix for Hungarian algorithm...\n");
    
    LL_mat = -1.0 * LL_mat;  // Negate to convert to cost
    LL_mat = LL_mat - (LL_mat.minCoeff() * Eigen::MatrixXd::Ones(LL_mat.rows(), LL_mat.cols()));

    std::vector<std::vector<double>> costMat;
    std::vector<double> row;
    for (int i = 0; i < LL_mat.rows(); i++){
        row.clear();
        for(int j = 0; j < LL_mat.cols(); j++){
            row.push_back(LL_mat(i,j));
        }
        costMat.push_back(row);
    }

    if(debug_){
        printf("[updateTracksFromDetections] Cost matrix prepared\n");
        std::cout << "costMat:\n";
        for(size_t ii = 0; ii < costMat.size(); ii++){
            for(size_t jj = 0; jj < costMat[ii].size(); jj++)
                std::cout << costMat[ii][jj] << " ";
        }
        std::cout << "\n";
    }

    // Apply Hungarian algorithm
    if(debug_) printf("[updateTracksFromDetections] Executing Hungarian algorithm...\n");
    
    std::vector<int> assignment;
    double cost = HungAlgo_.Solve(costMat, assignment);
    
    if(debug_){
        printf("[updateTracksFromDetections] Hungarian algorithm executed. cost = %f\n", cost);
        std::cout << "Assignment vector:\n";
        for(size_t i = 0; i < assignment.size(); i++){
            std::cout << assignment[i] << " ";
        }
        std::cout << "\n";
    }

    // Vector to mark assigned measurements
    Eigen::VectorXi assigned_z(basic_measurements.size()); 
    assigned_z = Eigen::VectorXi::Zero(basic_measurements.size());

    // Update tracks with assigned measurements
    if(debug_) printf("[updateTracksFromDetections] Updating tracks using assigned measurements\n");
    
    for(auto it_t = tracks_.begin(); it_t != tracks_.end(); it_t++){
        int tr_idx = it_t - tracks_.begin();
        
        // Apply state correction if measurement is assigned
        if (assignment[tr_idx] > -1){
            // Double check the assigned measurement is good
            double LL = kf_model_->logLikelihood((*it_t).current_state, basic_measurements[assignment[tr_idx]]);
            if(LL >= l_threshold_){
                assigned_z(assignment[tr_idx]) = 1;
                
                // Update kinematic state using Kalman filter
                (*it_t).current_state = kf_model_->updateX(basic_measurements[assignment[tr_idx]], (*it_t).current_state);
                (*it_t).n += 1;
                (*it_t).last_measurement_time = basic_measurements[assignment[tr_idx]].time_stamp;
                
                // Update detection fields (class, confidence, bounding boxes, etc.)
                updateTrackFromDetection((*it_t), detections[assignment[tr_idx]]);
                
                if(debug_) printf("[updateTracksFromDetections] Updated track %d with detection %d\n", 
                                 tr_idx, assignment[tr_idx]);
            }
        }

        // Predict track to current time stamp
        // double dt = t - (*it_t).current_state.time_stamp;
        // if(debug_) printf("[updateTracksFromDetections] Time difference between track %d time and current tim dt=%f \n", tr_idx, dt);
        // (*it_t).current_state = kf_model_->predictX((*it_t).current_state, dt);
        (*it_t).buffer.push_back((*it_t).current_state);
        if((*it_t).buffer.size() > state_buffer_size_)
            (*it_t).buffer.erase((*it_t).buffer.begin());
    }

    if(debug_) std::cout << "[updateTracksFromDetections] assigned_z vector:\n" << assigned_z << "\n";

    // Add unassigned detections as new tracks
    if(debug_) printf("[updateTracksFromDetections] Adding new tracks using unassigned detections\n");
    
    for(size_t m = 0; m < detections.size(); m++){
        if(assigned_z(m) > 0) continue;  // Skip assigned detections
        
        // Create new track from unassigned detection
        sensor_measurement basic_meas = toSensorMeasurement(detections[m]);
        kf_state state = kf_model_->initStateFromMeasurements(basic_meas);
        state.P = kf_model_->Q(dt_pred_);  // Use process noise for initial uncertainty
        
        kf_track new_track;
        new_track.current_state = state;
        new_track.n = 1;
        new_track.last_measurement_time = state.time_stamp;
        new_track.buffer.push_back(state);
        
        // Copy detection information
        new_track.class_name = detections[m].class_name;
        new_track.confidence = detections[m].confidence;
        new_track.track_score = detections[m].confidence;
        
        // Copy bounding box information
        new_track.has_2d_bbox = detections[m].has_2d_bbox;
        if (new_track.has_2d_bbox) {
            new_track.bbox_2d_center_x = detections[m].bbox_2d_center_x;
            new_track.bbox_2d_center_y = detections[m].bbox_2d_center_y;
            new_track.bbox_2d_width = detections[m].bbox_2d_width;
            new_track.bbox_2d_height = detections[m].bbox_2d_height;
        }
        
        new_track.has_3d_bbox = detections[m].has_3d_bbox;
        if (new_track.has_3d_bbox) {
            new_track.bbox_3d_center = detections[m].bbox_3d_center;
            new_track.bbox_3d_size = detections[m].bbox_3d_size;
            new_track.bbox_3d_orientation = detections[m].bbox_3d_orientation;
        }
        
        new_track.attributes = detections[m].attributes;
        
        // Assign track ID
        if (use_track_id_ && detections[m].id != 0) {
            new_track.id = detections[m].id;
        } else {
            new_track.id = getNextTrackId();
        }

        tracks_.push_back(new_track);
        
        if(debug_) {
            printf("[updateTracksFromDetections] New track added with ID: %u, class: %s using unassigned detection\n", 
                   new_track.id, new_track.class_name.c_str());
        }
    }
    
    if(debug_) printf("[updateTracksFromDetections] Done updating tracks from detections\n");
}

void KFTracker::updateTracksFromBasicMeasurements(double t)
{
    // This is the existing updateTracks logic, just renamed and using basic measurements only
    measurement_set_mtx_.lock();
    auto z = measurement_set_;
    measurement_set_mtx_.unlock();

    if(z.empty()) {
        if(debug_) printf("WARN [updateTracksFromBasicMeasurements] No basic measurements available\n");
        return;
    }

    // Check if we got new measurements
    auto z_t = z[0].time_stamp;
    if (z_t <= last_measurement_t_) {
        if(debug_) printf("WARN [updateTracksFromBasicMeasurements] No new measurements. Skipping update step.\n");
        return;
    }
    last_measurement_t_ = z[0].time_stamp;

    // Rest of the logic is the same as the original updateTracks method
    // (Copy the existing Hungarian algorithm logic from the original updateTracks method)
    
    if(debug_) printf("[updateTracksFromBasicMeasurements] Computing loglikelihood matrix...\n");
    
    Eigen::MatrixXd LL_mat(tracks_.size(), z.size());
    for(auto it_t = tracks_.begin(); it_t != tracks_.end(); it_t++){
        int tr_idx = it_t - tracks_.begin();

        // Find closest state in time
        bool found_closest_state = false;
        for (int k = (*it_t).buffer.size()-1; k >= 0; k--) {
            auto x_t = (*it_t).buffer[k].time_stamp;
            if(z_t >= x_t) {
                (*it_t).current_state.time_stamp = (*it_t).buffer[k].time_stamp;
                (*it_t).current_state.x = (*it_t).buffer[k].x;
                (*it_t).current_state.P = (*it_t).buffer[k].P;
                (*it_t).buffer.clear(); 
                (*it_t).buffer.push_back((*it_t).current_state);
                found_closest_state = true;
                break;
            }
        }

        for(auto it_z = z.begin(); it_z != z.end(); it_z++){
            int z_idx = it_z - z.begin();

            if (use_track_id_ && (*it_t).id != 0 && (*it_z).id != 0 && (*it_t).id != (*it_z).id) {
                LL_mat(tr_idx, z_idx) = -9999.0;
                continue;
            }

            double LL = kf_model_->logLikelihood((*it_t).current_state, (*it_z));
            if ((LL >= l_threshold_) && found_closest_state) {
                LL_mat(tr_idx, z_idx) = LL;
            } else {
                LL_mat(tr_idx, z_idx) = -9999.0;
            }
        }
    }

    // Hungarian algorithm (same as before)
    LL_mat = -1.0 * LL_mat;
    LL_mat = LL_mat - (LL_mat.minCoeff() * Eigen::MatrixXd::Ones(LL_mat.rows(), LL_mat.cols()));

    std::vector<std::vector<double>> costMat;
    std::vector<double> row;
    for (int i = 0; i < LL_mat.rows(); i++){
        row.clear();
        for(int j = 0; j < LL_mat.cols(); j++){
            row.push_back(LL_mat(i,j));
        }
        costMat.push_back(row);
    }

    std::vector<int> assignment;
    double cost = HungAlgo_.Solve(costMat, assignment);

    Eigen::VectorXi assigned_z(z.size()); 
    assigned_z = Eigen::VectorXi::Zero(z.size());

    // Update tracks (no detection field updates, just kinematic)
    for(auto it_t = tracks_.begin(); it_t != tracks_.end(); it_t++){
        int tr_idx = it_t - tracks_.begin();
        
        if (assignment[tr_idx] > -1){
            double LL = kf_model_->logLikelihood((*it_t).current_state, z[assignment[tr_idx]]);
            if(LL >= l_threshold_){
                assigned_z(assignment[tr_idx]) = 1;
                (*it_t).current_state = kf_model_->updateX(z[assignment[tr_idx]], (*it_t).current_state);
                (*it_t).n += 1;
                (*it_t).last_measurement_time = z[assignment[tr_idx]].time_stamp;
            }
        }

        // Predict to current time
        double dt = t - (*it_t).current_state.time_stamp;
        (*it_t).current_state = kf_model_->predictX((*it_t).current_state, dt);
        (*it_t).buffer.push_back((*it_t).current_state);
        if((*it_t).buffer.size() > state_buffer_size_)
            (*it_t).buffer.erase((*it_t).buffer.begin());
    }

    // Add new tracks from unassigned measurements (with default detection fields)
    for(size_t m = 0; m < z.size(); m++){
        if(assigned_z(m) > 0) continue;
        
        kf_state state = kf_model_->initStateFromMeasurements(z[m]);
        state.P = kf_model_->Q(dt_pred_);
        
        kf_track new_track;  // Constructor sets default detection field values
        new_track.current_state = state;
        new_track.n = 1;
        new_track.last_measurement_time = state.time_stamp;
        new_track.buffer.push_back(state);
        
        if (use_track_id_ && z[m].id != 0) {
            new_track.id = z[m].id;
        } else {
            new_track.id = getNextTrackId();
        }

        tracks_.push_back(new_track);
    }
} 

void KFTracker::removeUncertainTracks(){
   if(tracks_.empty())
      return;

   for (auto it = tracks_.begin(); it != tracks_.end(); it++)
   {
      int i = it - tracks_.begin();
      // Calculate position uncertainty of this track
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matrix
      auto V = sqrt(std::fabs(P_p.determinant()));
      if(debug_)
         printf("WARN [KFTracker::removeUncertainTracks] Track %d uncertainty = %f. number of measurements %d. \n", i, V, (*it).n);      

      // Remove tracks with high uncertainty
      if(V > V_max_ || isinf((*it).current_state.x.norm()))
      {
         if(debug_){
            printf("WARN [KFTracker::removeUncertainTracks] Track %d uncertainty = %f is high (> %f). Removing it.\n", i, V, V_max_);
            printf("WARN [KFTracker::removeUncertainTracks] Track %d norm(state) = %f\n", i, (*it).current_state.x.norm());
         }
         
         tracks_.erase(it--);
         continue;
      }

      // Remove track if it has not received measurements for long time
      if(abs((*it).current_state.time_stamp - (*it).last_measurement_time) > track_measurement_timeout_)
      {
         if(debug_) {
            printf("WARN [KFTracker::removeUncertainTracks] Track %d has not been updated for %f seconds. Removing it.\n", 
                  i, abs((*it).current_state.time_stamp - (*it).last_measurement_time));
         }
         tracks_.erase(it--);
      }
   }  
}

void KFTracker::updateCertainTracks(void)
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
      auto P_p = (*it).current_state.P.block(0,0,3,3); // extract position covariance sub-matrix
      auto V = sqrt(std::fabs(P_p.determinant()));
      if(debug_)
         printf("WARN [KFTracker::updateCertainTracks] Track %d uncertainty = %f. number of measurements %d.\n", i, V, (*it).n);
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
void KFTracker::filterLoop(double t)
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

   double dt = t - last_prediction_t_;
   predictTracks(dt);
   last_prediction_t_ = t;

   // Do correction step for all tracks using latest measurements.
   updateTracks(t);

   // Extract good tracks
   updateCertainTracks();

   // Remove bad tracks
   removeUncertainTracks();

   return;
}