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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <apriltag_ros/AprilTagDetectionArray.h>
#include "multi_target_kf/KFTrack.h"
#include "multi_target_kf/KFTracks.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <math.h>  // log

#include "multi_target_kf/models.h"

#include <mutex>          // std::mutex
#include <thread> // std::this_thread::get_id()

//using namespace std;
// using namespace Eigen;

enum MODEL : unsigned char
{
   CONSTANT_ACCEL = 0, DUBINS = 1
};

/**
 * Structure to store the current stamped KF prediction and buffer of previous state
 */
struct kf_track
{
   unsigned int id; /**< Unique track ID, e.g. Apriltag ID. Also, useful for object association */
   kf_state current_state;
   std::vector<kf_state> buffer;
   unsigned int n; /**< Number of received measurements. */
};

const unsigned char kMODEL = MODEL::DUBINS;

//* KFTracker class
/**
 * Implements a Kalman-filter-based object tracker based on constant velocity model
 * Reference 1: Constant velocity model (http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf)
 * Reference 2: Discrete KF equations (https://en.wikipedia.org/wiki/Kalman_filter)
 */
class KFTracker
{
private:
   ros::NodeHandle nh_;
   ros::NodeHandle nh_private_;

   double dt_pred_;

   std::vector<kf_track> tracks_; /**< Vector of current tracks. */
   std::vector<kf_track> certain_tracks_; /**< Vector of certain tracks only. */

   std::vector<sensor_measurement> measurement_set_; /**< set of all measurements. */
   ros::Time last_measurement_t_; 

   double V_max_; /**< minimum uncertainty before rejecting a track [m^3] */
   double V_certain_; /**< maximum uncertainty for a track to be considered certain [m^3] */
   int N_meas_; /**< minimum number of measurements to accept a track and add it to certain_tracks_. */
   double l_threshold_; /**< measurement association log-likelihood threshold. */

   kf_state kf_state_pred_; /**< KF predicted state and covariance */
   sensor_measurement z_meas_; /**< current sensor measurement. 3x1 vector. */
   sensor_measurement z_last_meas_; /**< previous sensor measurement. 3x1 vector. */
   
   bool is_state_initialzed_; /**< flag to start state prediction. Initialized by 1st measurement. */
   std::vector<kf_state> state_buffer_; /**< Bueffer to store last state_buffer_size_ predicted x and P */
   unsigned int state_buffer_size_; /**< lenght of state buffer state_buffer_*/
   std::string tracking_frame_; /**< Coordinate frame at which tracking is performed */
   bool do_update_step_; /**< Whether to perform the KF update step. WARNING this is just for debuggin. */
   ros::Time last_measurement_time_; /**<  Time of the last received measurement*/
   double measurement_off_time_; /**< Maximum time (in seconds) with no measurement before filter is stopped. */
   std::string apriltags_topic_; /**< ROS topic name of apriltag detections */
   std::string target_frameid_; /**< Target frame name which will be post-fixed by the target unique number e.g. "tag", post-fixed will be like "tag1" */
   tf::TransformListener tf_listener_; /**< TF listener for measurements */
   bool listen_tf_; /**< listens to TF to find transofrms of received measurements w.r.t. tracking_frame_ */
   bool use_track_id_; /**< False: does not consider track ID in measurement-state association */

   std::vector<double> q_diag_; /* diagonal elements of Q matrix */
   std::vector<double> r_diag_; /* diagonal elements of R matrix */

   ConstantAccelModel kf_model_; /* Constant acceleration KF model */
   // DubinsModel kf_model_; /* 3D Dubins EKF model */

   std::mutex measurement_set_mtx_; /* mutex to guard measurement_set_  from interferring calls */

   

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

   /**
    * @brief Performs KF update step for all tracks. 
    * @param t : current time in seconds
    */
   void updateTracks(ros::Time t);

   /**
    * @brief Extract tracks with high certainty from the current tracks.
    * Uses tracks_ and updates certain_tracks_ 
    */
   void updateCertainTracks(void);


   /**
    * @brief Executes the KF predict and update steps.
    */
   void filterLoop(const ros::TimerEvent& event);


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

   ros::Publisher state_pub_;
   ros::Publisher poses_pub_; /**< ROS publisher for KF estimated (certain) tracks positions */
   ros::Publisher all_poses_pub_; /**< ROS publisher for KF estimated  (ALL) tracks positions */
   ros::Publisher certain_tracks_pub_;/**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
   ros::Publisher all_tracks_pub_;/**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
   ros::Subscriber pose_sub_; /**< Subscriber to measurments. */
   ros::Subscriber pose_array_sub_; /**< Subscriber to measurments. */
   ros::Subscriber apriltags_sub_; /**< Subscriber to apriltags detections (requires apriltag_ros pkg). */

   ros::Timer kf_loop_timer_;

public:
   KFTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
   ~KFTracker();
};

#endif