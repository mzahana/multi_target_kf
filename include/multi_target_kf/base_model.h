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

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <math.h>  // log
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
 * @brief Base class that contains all common members and functions for all models.
 * @todo Needs implementation
 */
class BaseModel
{
public:

   Eigen::MatrixXd F_; /* State transition jacobian matrix */
   Eigen::MatrixXd H_; /* Observation jacobian matrix */
   Eigen::MatrixXd Q_; /** Process covariance matrix */
   Eigen::MatrixXd P_; /* State covariance estimate */
   Eigen::MatrixXd R_; /** Measurements covariance matrix */
   Eigen::VectorXd x_; /* Current state vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed] */
   unsigned int NUM_STATES=8; /* State dimension */
   unsigned int NUM_MEASUREMENTS=8; /* Measurements dimension */
   double dt_; /* Prediction sampling time */
   double current_t_; /* Current time stamp */

   BaseModel(){ init();}
   ~BaseModel(){}


   bool
   init(void)
   {
      F_.resize(NUM_STATES,NUM_STATES);
      H_.resize(NUM_MEASUREMENTS,NUM_STATES);
      Q_.resize(NUM_STATES,NUM_STATES);
      P_.resize(NUM_STATES,NUM_STATES);
      R_.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
      x_ = Eigen::MatrixXd::Zero(NUM_STATES,1);

      return(true);

   }
   unsigned int
   numStates(){
      return NUM_STATES;
   }

   unsigned int
   numMeasurements(){
      return NUM_MEASUREMENTS;
   }

   void
   setCurrentTimeStamp(double t){
      current_t_ = t;
   }

   bool
   setDt(double dt){
      if (dt < 0){
         printf("WARN [DubinModel::setDt] dt < 0");
         return false;
      }
      dt_ = dt;
      return true;
   }


   /**
    * @brief Computes the state prediction using model f(x)
    * @param state vector
    * @param dt sampling time in seconds
    * @return predicted state vector
    */
   virtual Eigen::VectorXd
   f(Eigen::VectorXd x, double dt)
   {
      if(debug_)
         printf("[BaseModel::f] Calculating f");

      if (dt <= 0){
         printf("ERROR [BaseModel::f] dt is <= 0. Returning same x");
         return x;
      }

      Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

      /** @todo Needs implementation in the specific model */
      
      return (A*x);
   }


   /**
    * @brief Computes the state prediction using internal state x_, and dt_
    * @param state vector
    * @return predicted state vector
    */
   Eigen::VectorXd
   f()
   {
      auto new_x = f(x_, dt_);
      return new_x;
   }

   /**
    * @brief Computes the state prediction using model given state, and internal dt_
    * @param state vector
    * @return predicted state vector
    */
   Eigen::VectorXd
   f(Eigen::VectorXd x)
   {
      Eigen::VectorXd new_x = f(x, dt_); // Updates x_
      return new_x;
   }

   /**
    * @brief Computes the state transition jacobian matrix F
    * @param x state vector
    * @param dt sampling time in seconds
    * @return jacobain F matrix
    */
   virtual Eigen::MatrixXd
   F(Eigen::VectorXd x, double dt)
   {
      // x = [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed]
      //      0    1   2   3      4      5           6          7
      // F_.resize(NUM_STATES,NUM_STATES);
      Eigen::MatrixXd FF = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

      /** @todo Needs specific implementation */

      return FF;
   }

   /**
    * @brief State transition matrix of a linear system
    * @param double Sampling time in seconds
    * @return MatrixXd State transition matrix
    */
   virtual Eigen::MatrixXd
   F(double dt){
      if(debug_)
         printf("[BaseModel::F] Calculating F");

      Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

      /** @todo Needs implementation in the specific model */

      return A;
   }

   /**
    * @brief Computes the state transition jacobian matrix F
    * @param x state vector
    * @return jacobain F matrix
    */
   Eigen::MatrixXd
   F(Eigen::VectorXd x)
   {
      Eigen::MatrixXd FF = F(x,dt_);
      return FF;
   }

   /**
    * @brief Sets transition jacobian matrix F_
    * @param M Matrix of size (NUM_STATES, NUM_STATES)
    */
   bool
   F(Eigen::MatrixXd M){
      if (M.cols()==NUM_STATES && M.rows()==NUM_STATES){
         F_.resize(NUM_STATES,NUM_STATES);
         F_=M;
         return true;
      }
      else    return false;
      
   }

   /**
    * @brief Returns transition jacobian matrix F_
    */
   Eigen::MatrixXd
   F()
   {
      return F_;
   }

   /**
    * @brief Computes the observation model h(x)
    * @param x state vector
    * @return observation z vector
    */
   virtual Eigen::VectorXd
   h(Eigen::VectorXd x)
   {
      if(debug_)
         printf("[BaseModel::h] Calculating h");

      H_.resize(NUM_MEASUREMENTS, NUM_STATES);
      H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
      
      /** @todo Needs implementation in the specific model */

      Eigen::VectorXd z = H_*x;
      if(debug_)
         std::cout << "h(x) = \n" << z << "\n";
      return z;
   }

   /**
    * @brief Returns jacobian observation matrix H_
    */
   virtual Eigen::MatrixXd
   H(void){
      if(debug_)
         printf("[BaseModel::H] Returning H_");

      H_.resize(NUM_MEASUREMENTS, NUM_STATES);
      H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
      
      /** @todo Needs implementation in the specific model */

      return H_;
   }

   /**
    * @brief Sets Jacobain observation matrix H_
    * @param M MatrixXd Input observation matrix
    * @return Bool True if dimensions are OK.
    */
   bool
   H(Eigen::MatrixXd M){
      if(debug_)
         ROS_INFO("[BaseModel::H] Setting H_");

      if(M.cols() == NUM_STATES && M.rows() == NUM_MEASUREMENTS){
         H_.resize(NUM_MEASUREMENTS, NUM_STATES);
         H_ = M;
         return true;
      }
      else
         return false;
   }


   /**
    * @brief Returns Q_
    */
   Eigen::MatrixXd
   Q(void)
   {
      return Q_;
   }


   virtual Eigen::MatrixXd
   Q(double dt){
      if (dt <= 0) return Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);;


      // Initialize
      Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

      /** @todo Needs implementation in the specific model */

      return Q_;
   }

   /**
    * @brief Sets Q_ from Eigen::MatrixXd
    * @param M Eigen::MatrixXd
    */
   bool
   Q(Eigen::MatrixXd M)
   {
      if (M.cols() == NUM_STATES && M.rows()==NUM_STATES){
         Q_.resize(NUM_STATES,NUM_STATES);
         Q_=M;
         return true;
      }
      else
         return false;
   }

   /**
    * @brief Initialize Q_ as a diagonal matrix using a std::vector representing diagonal elements
    * @param v std::vector<double> Vector of diagonla elements.  Its size must be equal to NUM_STATES
    * @return Bool True if size of input vector = NUM_STATES
    */
   bool
   Q(std::vector<double> v){
      if(v.size()!=NUM_STATES){
         ROS_ERROR("[DubinsModel::Q] Input vector size != NUM_STATES");
         return false;
      }
      Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_STATES,1);
      for (int i=0; i< NUM_STATES; i++) temp(i) = v[i];

      Q_ = temp.asDiagonal();
      return true;
   }


   /**
    * @brief Returns R_
    * @return MatrixXd Measurement covariance matrix R_
    */
   Eigen::MatrixXd
   R(void)
   {
      return R_;
   }

   /**
    * @brief Sets R_ using MatrixXd
    * @param MatrixXd Input R matrix
    * @return Bool. True if dimensions are OK. 
    */
   bool
   R(Eigen::MatrixXd M)
   {
      if(debug_)
         printf("[BaseModel::R] Setting R_ from a matrix");

      if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS){
         R_.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
         R_ = M;
         return true;
      }
      else
         return false;
   }

   /**
    * @brief Initialize R_ as a diagonal matrix using a std::vector representing diagonal elements
    * @param v std::vector<double> Vector of diagonla elements.  Its size must be equal to NUM_MEASUREMENTS
    * @return Bool True if size of input vector = NUM_MEASUREMENTS
    */
   bool
   R(std::vector<double> v){
      if(debug_)
         printf("[BaseModel::R] Setting diagonal R_ from a vector");

      if((unsigned int)v.size()!=NUM_MEASUREMENTS){
         printf("ERROR [BaseModel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu", v.size());
         return false;
      }
      Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
      for (int i=0; i< NUM_MEASUREMENTS; i++) temp(i) = v[i];

      R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
      R_ = temp.asDiagonal();
      if(debug_)
         std::cout << "R_=" << std::endl << R_ << std::endl;
      return true;
   }

   /**
    * @brief Returns P_
    * @return MatrixXd State covariance matrix
    */
   Eigen::MatrixXd
   P(void)
   {
      return P_;
   }

   /**
    * @brief Sets P_
    * @param MatrixXd input state covariance matrix
    * @return Bool. True if dimensions are OK.
    */
   bool
   P(Eigen::MatrixXd M)
   {
      if(debug_){
         ROS_INFO("[BaseModel::P] Setting P from a matrix");
      }

      if (M.rows() == NUM_STATES && M.cols() == NUM_STATES){
         P_.resize(NUM_STATES,NUM_STATES);
         P_ = M;
         if(debug_)
               std::cout << "P: " << std::endl << P_<< std::endl;
         return true;
      }
      else
         return false;
   }

   /**
    * @brief Returns current state vectror 
    * @return VectorXd State vector x_
    */
   Eigen::VectorXd
   getx(void){
      return x_;
   }

   /**
    * @brief Sets current state vectror x_
    * @param VectorXd Input state vector
    * @return Bool. True if dimensions are OK.
    */
   bool
   setx(Eigen::VectorXd v){
      if(v.cols() == 1 and v.rows()==NUM_STATES){
         x_ = v;
         return true;
      }
      else
         return false;
   }


   /**
    * @brief Prediciton step of a discrete KF using given state s and dt
    * @return kf_state current state (includes time, state, covariance)
    * @return kf_state Predicted state (includes time, state, covariance)
    */
   virtual kf_state
   predictX(kf_state s, double dt){
      if(debug_)
         printf("[BaseModel::predictX] Predicting x");

      if (dt <= 0){
         printf("WARN [BaseModel::predictX] dt = %f <= 0. Returning same state", dt);
         return s;
      }

      if(debug_)
         printf("[BaseModel::predictX] det(P) of current state: %f", s.P.determinant());

      kf_state xx;
      xx.time_stamp = s.time_stamp + dt;

      auto FF = F(dt);
      xx.P = FF*s.P*FF.transpose()+Q(dt);
      xx.x = f(s.x, dt);

      if(debug_){
         printf("[BaseModel::predictX] ||x_new - x_old|| = %f", (xx.x - s.x).norm());
         printf("[BaseModel::predictX] det(P) of predicted state: %f", xx.P.determinant());
      }

      if(debug_){
         std::cout << "[predictX] old P = \n" << s.P << "\n";
         std::cout << "[predictX] old x = \n" << s.x << "\n";

         std::cout << "[predictX] new P = \n" << xx.P << "\n";
         std::cout << "[predictX] new x = \n" << xx.x << "\n";
      }
      
      return xx;
   }

   /**
    * @brief Update step of a discrete KF using given state x and dt, and measurements z
    * @param kf_state current state (includes time, state, covariance)
    * @return kf_state Predicted state (includes time, state, covariance)
    */
   virtual kf_state
   updateX(sensor_measurement z, kf_state s){
      if(debug_)
         printf("[BaseModel::updateX] Updating x");

      kf_state xx;
      xx.time_stamp = z.time_stamp; //z.time_stamp;

      Eigen::VectorXd y = z.z - h(s.x); // innovation
      Eigen::MatrixXd S = H()*s.P*H().transpose() + R_; // innovation covariance
      Eigen::MatrixXd K = s.P*H().transpose()*S.inverse(); // optimal Kalman gain
      xx.x = s.x + K*y; 
      xx.P = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - K*H())*s.P;

      if(debug_){
         printf("[BaseModel::updateX] Done updating state");
         printf("[BaseModel::updateX] Norm of innovation y = %f ", y.norm());
         std::cout << "[BaseModel::updateX] predicted state P: \n" << s.P << "\n";
         // std::cout << "[BaseModel::updateX] innovation covariance S: \n" << S << "\n";
         std::cout << "[BaseModel::updateX] Kalman gain: \n" << K << "\n";
         std::cout << "[BaseModel::updateX] uncorrected state: \n" << s.x << "\n";
         std::cout << "[BaseModel::updateX] measurement: \n" << z.z << "\n";
         std::cout << "[BaseModel::updateX] corrected state: \n" << xx.x << "\n";
      }

      return xx;
   }

   /**
    * @brief Computes log Likelihood between predicted state and a measurement
    * @param xx kf_state Predicted state (time, state, covariance)
    * @param z sensor_measurement measurement (time, state, covariance)
    * @return double logLikelihood
    */
   virtual double
   logLikelihood(kf_state xx, sensor_measurement z){
      // if(debug_){
      //     ROS_INFO("[ConstantAccelModel::logLikelihood] Calculating logLikelihood");
      //     std::cout << "x: \n" << xx.x << "\n";
      //     std::cout << "z: \n" << z.z << "\n";
      // }

      Eigen::VectorXd y_hat; //z.z - h(xx.x); // innovation
      y_hat = z.z - h(xx.x); // innovation
      // if(debug_) std::cout << "y_hat: \n" << y_hat << "\n";

      Eigen::MatrixXd S;
      S = R_ + H()*xx.P*H().transpose(); // innovation covariance
      // if(debug_) std::cout << "S: \n" << S << "\n";

      auto S_inv = S.inverse();
      // if (debug_) std::cout << "S_inv \n" << S_inv << "\n";
      auto tmp = y_hat.transpose()*S_inv*y_hat;

      double LL = -0.5 * (tmp + log( std::fabs(S.determinant()) ) + 3.0*log(2*M_PI)); // log-likelihood

      if(debug_)
         printf("[BaseModel::logLikelihood] Done computing logLikelihood. L= %f", LL);

      return LL;
   }

   virtual kf_state
   initStateFromMeasurements(sensor_measurement z){

      if(debug_)
         printf("[BaseModel::initStateFromMeasurements] Initializing state from measurements");

      kf_state state;
      state.time_stamp = z.time_stamp;
      state.x = Eigen::MatrixXd::Zero(NUM_STATES,1);
      state.x(0) = z.z(0);
      state.x(1) = z.z(1);
      state.x(2) = z.z(2);

      state.P = Q_;

      return state;
   }

   /**
    * @brief Computes ecluian distance between two states
   */
   double
   computeDistance(kf_state xx, sensor_measurement zz){
      auto y = zz.z - h(xx.x); // assumes length of xx.x = length of zz.z
      auto nrm = y.norm();
      if(debug_)
         printf("[BaseModel::computeDistance] Distance between state and measurement = %f", nrm);
      return nrm;
   }

};

#endif //BASE_MODEL_H
