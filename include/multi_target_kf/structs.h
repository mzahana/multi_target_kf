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

#ifndef STRUCTS_H
#define STRUCTS_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <math.h>  // log
#include <vector>  // ADDED THIS
#include <Eigen/Dense>

/**
 * Structure to store the current stamped KF prediction
 */
struct kf_state
{
   double time_stamp;
   Eigen::VectorXd x; // State estimate
   Eigen::MatrixXd P; // State estimate covariance
};

/**
 * Structure to store current stamped sensor measurement.
 */
struct sensor_measurement
{
   double time_stamp; /**< time in seconds */
   unsigned int id; /**< OPtional. Associated measurement ID, e.g. Apriltag ID */
   Eigen::VectorXd z; /**< Measurements, e.g. 3D position, velocity, ... etc */
   Eigen::MatrixXd R; /* Measurement covariance matrix */
};

/**
 * Enhanced measurement structure for object detection and tracking
 */
struct enhanced_measurement
{
   double time_stamp;           /**< time in seconds */
   unsigned int id;             /**< Object ID */
   std::string class_name;      /**< Object class name */
   double confidence;           /**< Detection confidence [0.0, 1.0] */
   
   // Core measurements
   Eigen::VectorXd position;           /**< 3D position [x, y, z] */
   Eigen::MatrixXd position_cov;       /**< Position covariance matrix */
   
   // Optional velocity measurements
   bool has_linear_velocity;
   Eigen::VectorXd linear_velocity;    /**< Linear velocity [vx, vy, vz] */
   Eigen::MatrixXd linear_velocity_cov;
   
   bool has_angular_velocity;
   Eigen::VectorXd angular_velocity;   /**< Angular velocity [wx, wy, wz] */
   Eigen::MatrixXd angular_velocity_cov;
   
   bool has_linear_acceleration;
   Eigen::VectorXd linear_acceleration; /**< Linear acceleration [ax, ay, az] */
   Eigen::MatrixXd linear_acceleration_cov;
   
   // 2D Bounding box (YOLO format: center + size)
   bool has_2d_bbox;
   double bbox_2d_center_x, bbox_2d_center_y, bbox_2d_width, bbox_2d_height;
   
   // 3D Bounding box (center + size + orientation)
   bool has_3d_bbox;
   Eigen::Vector3d bbox_3d_center;
   Eigen::Vector3d bbox_3d_size;
   Eigen::Quaterniond bbox_3d_orientation;
   
   // Additional attributes
   std::vector<std::string> attributes;
   
   // Constructor - FIXED INITIALIZATION ORDER
   enhanced_measurement() :
      time_stamp(0.0), id(0), confidence(0.0),
      has_linear_velocity(false), has_angular_velocity(false), 
      has_linear_acceleration(false), 
      has_2d_bbox(false), bbox_2d_center_x(0.0), bbox_2d_center_y(0.0), bbox_2d_width(0.0), bbox_2d_height(0.0),
      has_3d_bbox(false)
   {
      position = Eigen::VectorXd::Zero(3);
      position_cov = Eigen::MatrixXd::Identity(3, 3);
   }
   
   // Helper function to convert 2D bbox from center+size to top-left+size (if needed)
   void get2DBboxTopLeft(double& top_left_x, double& top_left_y) const {
      top_left_x = bbox_2d_center_x - bbox_2d_width / 2.0;
      top_left_y = bbox_2d_center_y - bbox_2d_height / 2.0;
   }
   
   // Helper function to set 2D bbox from top-left+size format
   void set2DBboxFromTopLeft(double top_left_x, double top_left_y, double width, double height) {
      bbox_2d_center_x = top_left_x + width / 2.0;
      bbox_2d_center_y = top_left_y + height / 2.0;
      bbox_2d_width = width;
      bbox_2d_height = height;
   }
};

// Conversion function from enhanced_measurement to basic sensor_measurement
sensor_measurement toBasicMeasurement(const enhanced_measurement& enhanced);

#endif //STRUCTS_H