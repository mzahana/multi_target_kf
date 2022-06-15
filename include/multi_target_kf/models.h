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

#ifndef MODELS_H
#define MODELS_H

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

struct DubinState
{
   ros::Time stamp;
   double px, py, pz, vx, vy, vz, theta, gamma, theta_dot, gamma_dot, speed;
};

class DubinsModel
{
/* Can read, Can't write*/
protected:

Eigen::MatrixXd F_; /* State transition jacobian matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd P_; /* State covariance estimate */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed] */
const unsigned int NUM_STATES=8; /* State dimension */
const unsigned int NUM_MEASUREMENTS=8; /* Measurements dimension */
int dt_; /* Prediction sampling time */
ros::Time current_t_; /* Current time stamp */


public:

DubinsModel():
dt_(0.01),
current_t_(ros::Time::now())
{}
~DubinsModel(){}

unsigned int
numStates(){
    return NUM_STATES;
}

unsigned int
numMeasurements(){
    return NUM_MEASUREMENTS;
}

bool init(void)
{
    F_.resize(NUM_STATES,NUM_STATES);
    H_.resize(NUM_MEASUREMENTS,NUM_STATES);
    x_ = Eigen::MatrixXd::Zero(NUM_STATES,1);

}

bool
setDt(int dt){
    if (dt < 0){
        ROS_WARN("[DubinModel::setDt] dt < 0");
        return false;
    }
    dt_ = dt;
    return true;
}

void
setCurrentTimeStamp(ros::Time t){
    current_t_ = t;
}

/**
 * @brief converts the vector state x to readable dubins state struct
 * @param x VectorXd statae vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed]
 */
DubinState
vector2DubinState(Eigen::VectorXd x){
    DubinState d;
    d.px = x(0); d.py = x(1); d.pz = x(2);
    d.theta = x(3); d.gamma = x(4);
    d.theta_dot = x(5); d.gamma_dot = x(6);
    d.speed = x(7);

    return d;
}

Eigen::VectorXd
dubinState2Vector(DubinState d){
    Eigen::VectorXd x;

    x(0)=d.px; x(1)=d.py; x(2)=d.pz;
    x(3)=d.theta; x(4)=d.gamma;
    x(5)=d.theta_dot; x(6)=d.gamma_dot;
    x(7)=d.speed;
    return x;
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
    auto new_x = f(x, dt_); // Updates x_
    return new_x;
}

/**
 * @brief Computes the state prediction using model f(x)
 * @param state vector
 * @param dt sampling time in seconds
 * @return predicted state vector
 */
Eigen::VectorXd
f(Eigen::VectorXd x, int dt)
{
    if (dt < 0){
        ROS_WARN("[DubinModel::f] dt is < 0. Returning same x");
        return x;
    }
    DubinState d = vector2DubinState(x);

    d.px = d.px + dt*d.speed*cos(d.theta)*sin(d.gamma_dot);
    d.py = d.py + dt*d.speed*(d.theta)*cos(d.gamma);
    d.pz = d.pz + dt*d.speed*sin(d.gamma);
    d.theta = d.theta + dt*d.theta_dot;
    d.gamma = d.gamma + dt*d.gamma_dot;

    auto new_x = dubinState2Vector(d);
    
    return new_x;
}

/**
 * @brief Computes the observation model h(x)
 * @param x state vector
 * @return observation z vector
 */
Eigen::VectorXd
h(Eigen::VectorXd x)
{
    return Eigen::MatrixXd::Identity(NUM_MEASUREMENTS, NUM_STATES) * x;
}

/**
 * @brief Computes the state transition jacobian matrix F
 * @param x state vector
 * @param dt sampling time in seconds
 * @return jacobain F matrix
 */
Eigen::MatrixXd
F(Eigen::VectorXd x, int dt)
{
    // x = [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed]
    //      0    1   2   3      4      5           6          7
    // F_.resize(NUM_STATES,NUM_STATES);
    Eigen::MatrixXd FF = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

    auto s_theta = sin(x(3)); auto c_theta = cos(x(3)); auto s_gamma = sin(x(4)); auto c_gamma = cos(x(4));
    FF(0,3) = -dt*s_theta*c_gamma;// px-theta
    FF(0,4) = -dt*c_theta*s_gamma; // px-gamma
    FF(0,7) = dt*c_theta*c_gamma; // px-speed
    FF(1,3) = dt*c_theta*c_gamma; // py-theta
    FF(1,4) = -dt*s_theta*s_gamma; // py-gamma
    FF(1,7) = dt*s_theta*c_gamma; // py-speed
    FF(2,4) = dt*c_gamma; // pz-gamma
    FF(2,7) = dt*s_gamma; // pz-speed
    FF(3,5) = dt; // theta-theta_dot
    FF(4,6) = dt; // gamma-gamma_dot


    return FF;
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
 * @brief Computes the observation jacobian matrix H
 * @param x state vector
 * @return jacobain H_ matrix
 */
Eigen::MatrixXd
H(Eigen::VectorXd x)
{
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Identity(NUM_MEASUREMENTS, NUM_STATES);
    return H_;
}

/**
 * @brief Computes and Returns H_
 */
Eigen::MatrixXd
H()
{
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Identity(NUM_MEASUREMENTS, NUM_STATES);
    return H_;
}

/**
 * @brief Returns Q_
 */
Eigen::MatrixXd
Q(void)
{
    return Q_;
}

/**
 * @brief Sets Q_
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
 */
Eigen::MatrixXd
R(void)
{
    return R_;
}

/**
 * @brief Sets R_
 */
bool
R(Eigen::MatrixXd M)
{
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
    if(v.size()!=NUM_MEASUREMENTS){
        ROS_ERROR("[DubinsModel::R] Input vector size != NUM_MEASUREMENTS");
        return false;
    }
    Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
    for (int i=0; i< NUM_STATES; i++) temp(i) = v[i];

    R_ = temp.asDiagonal();
    return true;
}

/**
 * @brief Returns P_
 */
Eigen::MatrixXd
P(void)
{
    return P_;
}

/**
 * @brief Sets P_
 */
bool
P(Eigen::MatrixXd M)
{
    if (M.rows() == NUM_STATES && M.cols() == NUM_STATES){
        P_.resize(NUM_STATES,NUM_STATES);
        P_ = M;
        return true;
    }
    else
        return false;
}

/**
 * @brief Returns current state vectror x_
 */
Eigen::VectorXd
x(void){
    return x_;
}

/**
 * @brief Sets current state vectror x_
 */
bool
x(Eigen::VectorXd v){
    if(v.cols() == 1 and v.rows()==NUM_STATES){
        x_ = v;
        return true;
    }
    else
        return false;
}

/**
 * @brief Estimates full state observation from two position vectors
 * @param p_m Vector3d Measured position vector
 * @param s VectorXd last predicted state
 * @param dt double  difference between last state time and current measurement time, in seconds
 * @return VectorXd Estimated observation
 * 
 */
Eigen::VectorXd 
estimateObservationFromPosition(Eigen::Vector3d p_m, Eigen::VectorXd s, double dt){
    // z = [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed]
    //       0   1   2    3      4      5            6        7
    
    // initial full observation estimate
    Eigen::VectorXd z = Eigen::MatrixXd::Zero(NUM_STATES,1);

    // observed position
    z(0) = p_m(0); z(1) = p_m(1); z(2) = p_m(2);

    // current position vector from the predicted state
    Eigen::Vector3d p; p << s(0), s(1), s(2);


    if (dt <=0) return s; // not a new/valid measurement

    // estimate velocity
    Eigen::Vector3d vel = (p_m-p)/dt;
    z(7) = vel.norm(); // speed
    
    //theta
    z(3) = atan2f64(vel(1), vel(0));
    
    // gamma
    auto gamma = asinf64(vel(2)/z(7));
    if (isnan(gamma)) z(4) = s(4); else z(4) = gamma;

    // theta_dot
    z(5) = (z(3) - s(3))/dt;

    // gamma_dot
    z(6) = (z(4) - s(4))/dt;

    // Done!

    return z;


}


/**
 * @brief Prediciton step of a discrete EKF using given state x and dt
 * @return kf_state current state (includes time, state, covariance)
 * @return kf_state Predicted state (includes time, state, covariance)
 */
kf_state
predictX(kf_state s, int dt){
    kf_state xx;
    xx.time_stamp = s.time_stamp + ros::Duration(dt);

    auto FF = F(s.x, dt);
    xx.P = FF*s.P*FF.transpose()+Q_;
    xx.x = f(s.x, dt);
    
    return xx;
}

/**
 * @brief Update step of a discrete EKF using given state x and dt, and measurements z
 * @param kf_state current state (includes time, state, covariance)
 * @return kf_state Predicted state (includes time, state, covariance)
 */
kf_state
updateX(sensor_measurement z, kf_state s){
    kf_state xx;
    xx.time_stamp = z.time_stamp;

    // estimate full observation
    Eigen::Vector3d p; p << z.z(0), z.z(1), z.z(2); // position measurement
    double dt = (z.time_stamp - s.time_stamp).toSec();
    auto z_m = estimateObservationFromPosition(p, s.x, dt);
    auto y = z_m - h(s.x); // innovation
    auto S = H()*s.P*H().transpose() + R_; // innovation covariance
    auto K = s.P*H().transpose()*S.inverse(); // near-optimal Kalman gain
    xx.x = s.x + K*y;
    xx.P = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - K*H())*s.P;

    return xx;
}

/**
 * @brief Computes log Likelihood between predicted state and a measurement
 * @param xx kf_state Predicted state (time, state, covariance)
 * @param z sensor_measurement measurement (time, state, covariance)
 * @return double logLikelihood
 */
double
logLikelihood(kf_state xx, sensor_measurement z){
    /* Just associate based on position, because other states are not obseravble */
    
    Eigen::MatrixXd HH = Eigen::MatrixXd::Zero(3, NUM_STATES);
    HH(0,0) = 1; //px
    HH(1,1) = 1; //py
    HH(2,2) = 1; //pz

    auto y_hat = z.z - h(xx.x).block(0,0,3,1); // innovation, position only
    auto S = R_ + HH*xx.P*HH.transpose(); // innovation covariance
    double LL = -0.5 * (y_hat.transpose() * S.inverse() * y_hat + log( std::fabs(S.determinant()) ) + 3.0*log(2*M_PI)); // log-likelihood

    return LL;
}

/**
 * Needs to be revisited!!! we can't push multiple measurements
 * in the context of multi-target, we can't gurantee that measurements are for the same target!!!!!
 * Just use one measurement
 */
kf_state
initStateFromMeasurements(sensor_measurement z){

    kf_state state;
    state.time_stamp = z.time_stamp;
    state.x = Eigen::MatrixXd::Zero(NUM_STATES,1);
    state.x(0) = z.z(0);
    state.x(1) = z.z(1);
    state.x(2) = z.z(2);

    state.P = Q_;

    return state;
}

};

// -----------------------------------------------------------------------//
//------------------------- ConstantAccelModel --------------------------//
//-----------------------------------------------------------------------//

/**
 * @brief Constant acceleration model
 */
class ConstantAccelModel
{
protected:

Eigen::MatrixXd F_; /* State transition matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd P_; /* State covariance estimate */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state (mean) vector [x, y, z, vx, vy, vz, ax, ay, az] */
const unsigned int NUM_STATES=9;
const unsigned int NUM_MEASUREMENTS=3; // position \in R^3
int dt_; /* Prediction sampling time */
ros::Time current_t_; /* Current time stamp */

public:
ConstantAccelModel():
dt_(0.01)
{};
~ConstantAccelModel(){};

unsigned int
numStates(){
    return NUM_STATES;
}

unsigned int
numMeasurements(){
    return NUM_MEASUREMENTS;
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
    auto new_x = f(x, dt_); // Updates x_
    return new_x;
}

/**
 * @brief Computes the state prediction using model f(x)
 * @param state vector
 * @param dt sampling time in seconds
 * @return predicted state vector
 */
Eigen::VectorXd
f(Eigen::VectorXd x, int dt)
{
    if (dt < 0){
        ROS_WARN("[ConstantAccelModel::f] dt is < 0. Returning same x");
        return x;
    }
    // The following is based on this thesis:
   // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
   Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);
   A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);
   A.block(0,6,3,3)= (dt*dt/2.0)*Eigen::MatrixXd::Identity(3,3);
   A.block(3,6,3,3)= dt*Eigen::MatrixXd::Identity(3,3);
    
    return A*x;
}

/**
 * @brief State transition matrix of a linear system
 * @param double Sampling time in seconds
 * @return MatrixXd State transition matrix
 */
Eigen::MatrixXd
F(int dt){
    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);
    A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);
    A.block(0,6,3,3)= (dt*dt/2.0)*Eigen::MatrixXd::Identity(3,3);
    A.block(3,6,3,3)= dt*Eigen::MatrixXd::Identity(3,3);

    return A;
}

/**
 * @brief Computes the observation model h(x)
 * @param x state vector
 * @return observation z vector
 */
Eigen::VectorXd
h(Eigen::VectorXd x)
{
    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
    H_(0,0) = 1.0; // observing x
    H_(1,1) = 1.0; // observing y
    H_(2,2) = 1.0; // observing z
    return H_ * x;
}

/**
 * @brief Returns observation matrix H_
 */
Eigen::MatrixXd
H(void){
    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
    H_(0,0) = 1.0; // observing x
    H_(1,1) = 1.0; // observing y
    H_(2,2) = 1.0; // observing z
    return H_;
}

/**
 * @brief Sets observation matrix H_
 * @param M MatrixXd Input observation matrix
 * @return Bool True of dimenstions are OK.
 */
bool
H(Eigen::MatrixXd M){
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

/**
 * @brief Sets Q_
 * @param M MatrixXd Input Q matrix
 * @return Bool. True if dimenstions are OK. 
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
    if((unsigned int)v.size()!=NUM_STATES){
        ROS_ERROR("[ConstantAccelModel::Q] Input vector size != NUM_STATES, v.size = %d", v.size());
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
 * @brief Sets R_
 * @param MatrixXd Input R matrix
 * @return Bool. True if dimensions are OK. 
 */
bool
R(Eigen::MatrixXd M)
{
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
    if((unsigned int)v.size()!=NUM_MEASUREMENTS){
        ROS_ERROR("[ConstantAccelModel::R] Input vector size != NUM_MEASUREMENTS, v.size = %d", v.size());
        return false;
    }
    Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
    for (int i=0; i< NUM_STATES; i++) temp(i) = v[i];

    R_ = temp.asDiagonal();
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
    if (M.rows() == NUM_STATES && M.cols() == NUM_STATES){
        P_.resize(NUM_STATES,NUM_STATES);
        P_ = M;
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
x(void){
    return x_;
}

/**
 * @brief Sets current state vectror x_
 * @param VectorXd Input state vector
 * @return Bool. True if dimensions are OK.
 */
bool
x(Eigen::VectorXd v){
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
kf_state
predictX(kf_state s, int dt){
    kf_state xx;
    xx.time_stamp = current_t_ + ros::Duration(dt);

    auto FF = F(dt);
    xx.P = FF*s.P*FF.transpose()+Q_;
    xx.x = f(s.x, dt);
    
    return xx;
}

/**
 * @brief Update step of a discrete KF using given state x and dt, and measurements z
 * @param kf_state current state (includes time, state, covariance)
 * @return kf_state Predicted state (includes time, state, covariance)
 */
kf_state
updateX(sensor_measurement z, kf_state s){
    kf_state xx;
    xx.time_stamp = z.time_stamp;

    auto y = z.z - h(s.x); // innovation
    auto S = H()*s.P*H().transpose() + R_; // innovation covariance
    auto K = s.P*H().transpose()*S.inverse(); // optimal Kalman gain
    xx.x = s.x + K*y;
    xx.P = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - K*H())*s.P;

    return xx;
}

/**
 * @brief Computes log Likelihood between predicted state and a measurement
 * @param xx kf_state Predicted state (time, state, covariance)
 * @param z sensor_measurement measurement (time, state, covariance)
 * @return double logLikelihood
 */
double
logLikelihood(kf_state xx, sensor_measurement z){
   auto y_hat = z.z - h(xx.x); // innovation
   auto S = R_ + H()*xx.P*H().transpose(); // innovation covariance
   double LL = -0.5 * (y_hat.transpose() * S.inverse() * y_hat + log( std::fabs(S.determinant()) ) + 3.0*log(2*M_PI)); // log-likelihood

   return LL;
}

kf_state
initStateFromMeasurements(sensor_measurement z){

    kf_state state;
    state.time_stamp = z.time_stamp;
    state.x = Eigen::MatrixXd::Zero(NUM_STATES,1);
    state.x(0) = z.z(0);
    state.x(1) = z.z(1);
    state.x(2) = z.z(2);

    state.P = Q_;

    return state;
}

};

#endif //MODELS_H