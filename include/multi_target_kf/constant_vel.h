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

#ifndef CONSTANT_VEL_H
#define CONSTANT_VEL_H

#include "multi_target_kf/base_model.h"


// -----------------------------------------------------------------------//
//------------------------- ConstantAccelModel --------------------------//
//-----------------------------------------------------------------------//

/**
 * @brief Constant velocity model
 */
class ConstantVelModel: public BaseModel
{
public:

    unsigned int NUM_STATES=6;// constant velocity model
    unsigned int NUM_MEASUREMENTS=3; // position \in R^3

    Eigen::VectorXd acc_variance_; /* 3D vector for accelration variances (sigma^2) in x,y z */

    double sigma_a_, sigma_p_, sigma_v_;

    /* Constructor */
    ConstantVelModel():
    dt_(0.01),
    F_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
    H_(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES)),
    Q_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
    P_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
    R_(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS)),
    x_(Eigen::MatrixXd::Zero(NUM_STATES,1)),
    acc_variance_(Eigen::VectorXd::Zero(3))
    {}
    ~ConstantVelModel(){}

    /**
     * @brief Sets the acceleration variance vector acc_variance_
     * @param a Eigen::VectorXd 3D vector contains [ax,ay,az]
     * @return bool True if size of a is 3. Otheriwse, returns false
    */
    bool
    setAccVar(Eigen::VectorXd a)
    {
        if (a.size() != 3) return false;

        acc_variance_.resize(3); acc_variance_= a;
        return true;
    }

    bool
    setSigmaA(double sigma_a)
    {
        if (sigma_a<=0) return false;
        sigma_a_ = sigma_a;

        if (debug_) std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }

    bool
    setSigmaP(double sigma_p)
    {
        if (sigma_p<=0) return false;
        sigma_p_ = sigma_p;
        if (debug_) std::cout << " sigma_p: " << sigma_p_ << "\n";
        return true;
    }

    bool
    setSigmaV(double sigma_v)
    {
        if (sigma_v<=0) return false;
        sigma_v_ = sigma_v;
        if (debug_) std::cout << " sigma_v: " << sigma_v_ << "\n";
        return true;
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
        if(debug_)
            printf("[ConstantVelModel::f] Calculating f");

        if (dt <= 0){
            ROS_WARN("[ConstantVelModel::f] dt is <= 0. Returning same x");
            return x;
        }

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

        // constant velocity model
        // A(0,3) = dt; // x - vx
        // A(1,4) = dt; // y - vy
        // A(2,5) = dt; // z - vz

        // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);
        
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
     * @brief State transition matrix of a linear system
     * @param double Sampling time in seconds
     * @return MatrixXd State transition matrix
     */
    Eigen::MatrixXd
    F(double dt) override
    {
        if(debug_)
            printf("[ConstantVelModel::F] Calculating F");

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

        // constant velocity model
        // A(0,3) = dt; // x - vx
        // A(1,4) = dt; // y - vy
        // A(2,5) = dt; // z - vz

        // The following is based on this thesis:
        // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
        A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);

        return A;
    }

    /**
     * @brief Computes the observation model h(x)
     * @param x state vector
     * @return observation z vector
     */
    Eigen::VectorXd
    h(Eigen::VectorXd x) override
    {
        if(debug_)
            printf("[ConstantVelModel::h] Calculating h");

        // The following is based on this thesis:
        // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
        H_(0,0) = 1.0; // observing x
        H_(1,1) = 1.0; // observing y
        H_(2,2) = 1.0; // observing z

        Eigen::VectorXd z = H_*x;
        if(debug_)
            std::cout << "h(x) = \n" << z << "\n";
        return z;
    }

    /**
     * @brief Returns observation matrix H_
     */
    Eigen::MatrixXd
    H(void) override
    {
        if(debug_)
            printf("[ConstantVelModel::H] Returning H_");

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
     * @brief Sets the process covariance matrix Q. Used in the prediction step of the Kalman filter
     * @param dt [double] time step in seconds
     * @param sigma_a [double] Standard deviation of the process accelration noise
    */
    bool
    Q(double dt, double sigma_a)
    {
        if (dt <= 0) return false;
        if (sigma_a <= 0) return false;


        // Initialize
        Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

        // Construct the upper left block, Qpp
        Q_.block(0,0,3,3) = dt*dt*dt*dt/4*Eigen::MatrixXd::Identity(3,3);
        // Construct the upper right block, Qpv
        Q_.block(0,3,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
        // Construct the lower left block, Qvp
        Q_.block(3,0,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
        // Construct the lower right block, Qvv
        Q_.block(3,3,3,3) = dt*dt*Eigen::MatrixXd::Identity(3,3);

        Q_ = sigma_a*sigma_a*Q_;

        if(debug_) std::cout << "Q: \n" << Q_ << "\n";


        return true;
    }

    Eigen::MatrixXd
    Q(double dt) override
    {
        if (dt <= 0) return Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);;


        // Initialize
        Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

        // Construct the upper left block, Qpp
        Q_.block(0,0,3,3) = dt*dt*dt*dt/4*Eigen::MatrixXd::Identity(3,3);
        // Construct the upper right block, Qpv
        Q_.block(0,3,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
        // Construct the lower left block, Qvp
        Q_.block(3,0,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
        // Construct the lower right block, Qvv
        Q_.block(3,3,3,3) = dt*dt*Eigen::MatrixXd::Identity(3,3);

        Q_ = sigma_a_*sigma_a_*Q_;


        return Q_;
    }

    /**
     * @brief Sets P_
     * @param sigma_p [double] Standard deviation of position
     * @param sigma_v [double] Standard deviation of velocity
     * @return Bool. True if sigma_p && and sigma_va are non-zero
     */
    bool
    P(double sigma_p, double sigma_v)
    {
        if (sigma_p <=0 || sigma_v<=0) return false;
        
        if(debug_){
            printf("[ConstantVelModel::P] Setting P from standard deviations");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0,0,3,3) = sigma_p*sigma_p*Eigen::MatrixXd::Identity(3,3);
        P_.block(3,3,3,3) = sigma_v*sigma_v*Eigen::MatrixXd::Identity(3,3);

        if (debug_) std::cout << "P: \n" << P_ << "\n";

        return true;
    }

};

#endif //CONSTANT_VEL_H