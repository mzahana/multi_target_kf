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

#ifndef BASE_MODEL_H
#define BASE_MODEL_H

#include <ros/ros.h>
#include <math.h>  // log

#include <Eigen/Dense>



/**
 * Structure to store the current stamped KF prediction
 */
struct kf_state
{
   ros::Time time_stamp;
   Eigen::VectorXd x; // State estimate
   Eigen::MatrixXd P; // State estimate covariance
};

/**
 * Structure to store current stamped sensor measurement.
 */
struct sensor_measurement
{
   ros::Time time_stamp;
   unsigned int id; /**< OPtional. Associated measurement ID, e.g. Apriltag ID */
   Eigen::VectorXd z; /**< Measurements, e.g. 3D position, velocity, ... etc */
   Eigen::MatrixXd R; /* Measurement covariance matrix */
};


/**
 * @brief Base class that contains all common members and functions for all models.
 * @todo Needs implementation
 */
class BaseModel
{
protected:

Eigen::MatrixXd F_; /* State transition jacobian matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd P_; /* State covariance estimate */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed] */
const unsigned int NUM_STATES=8; /* State dimension */
const unsigned int NUM_MEASUREMENTS=8; /* Measurements dimension */
double dt_; /* Prediction sampling time */
ros::Time current_t_; /* Current time stamp */

public:
BaseModel(){}
~BaseModel(){}

};

#endif //BASE_MODEL_H
