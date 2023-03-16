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

#ifndef DUBINS_MODEL_H
#define DUBINS_MODEL_H

#include "multi_target_kf/base_model.h"


struct DubinState
{
   double stamp;
   double px, py, pz, vx, vy, vz, theta, gamma, theta_dot, gamma_dot, speed;
};


class DubinsModel
{
/* Can read, Can't write*/
private:

    unsigned int NUM_STATES=8; /* State dimension */
    unsigned int NUM_MEASUREMENTS=8; /* Measurements dimension */


public:

    DubinsModel():
    dt_(0.01),
    current_t_(0)
    {}
    ~DubinsModel(){}


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
     * @brief Computes the state prediction using model f(x)
     * @param state vector
     * @param dt sampling time in seconds
     * @return predicted state vector
     */
    Eigen::VectorXd
    f(Eigen::VectorXd x, double dt) override
    {
        if (dt < 0){
            printf("WARN [DubinModel::f] dt is < 0. Returning same x");
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
    h(Eigen::VectorXd x) override
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
    F(Eigen::VectorXd x, double dt) override
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
     * @brief Computes the observation jacobian matrix H
     * @param x state vector
     * @return jacobain H_ matrix
     */
    Eigen::MatrixXd
    H(Eigen::VectorXd x)
    {
        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        /** @warning Not implemented*/
        return H_;
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
    predictX(kf_state s, double dt) override
    {
        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;

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
    updateX(sensor_measurement z, kf_state s) override
    {
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
    logLikelihood(kf_state xx, sensor_measurement z) override
    {
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
    initStateFromMeasurements(sensor_measurement z) override
    {

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

#endif //DUBINS_MODEL_H