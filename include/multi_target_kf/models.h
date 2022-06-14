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

struct dubin_state
{
   ros::Time stamp;
   double px, py, pz, vx, vy, vz, theta, gamma, theta_dot, gamma_dot, speed;
};

class DubinModel
{
/* Can read, Can't write*/
protected:

Eigen::MatrixXd F_; /* State transition jacobian matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed] */
const unsigned int NUM_STATES=8;
const unsigned int NUM_MEASUREMENTS=8;

public:

DubinModel(){}
~DubinModel(){}

bool init(void)
{
    F_.resize(NUM_STATES,NUM_STATES);
    H_.resize(NUM_MEASUREMENTS,NUM_STATES);
}

/**
 * @brief Computes the state prediction using model f(x)
 * @param state vector
 * @return predicted state vector
 */
Eigen::VectorXd
f(Eigen::VectorXd x)
{

}

/**
 * @brief Computes the observation model h(x)
 * @param x state vector
 * @return observation z vector
 */
Eigen::VectorXd
h(Eigen::VectorXd x)
{

}

/**
 * @brief Computes the state transition jacobian matrix F
 * @param x state vector
 * @return jacobain F matrix
 */
Eigen::MatrixXd
F(Eigen::VectorXd x)
{
    F_.resize(NUM_STATES,NUM_STATES);
    F_ = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

    return F_;
}

Eigen::MatrixXd
F()
{
    return F_;
}

/**
 * @brief Computes the observation jacobian matrix H
 * @param x state vector
 * @return jacobain H matrix
 */
Eigen::MatrixXd
H(Eigen::VectorXd x)
{
    H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
    return H_;
}

Eigen::MatrixXd
H()
{
    return H_;
}

Eigen::MatrixXd
Q()
{
    return Q_;
}

Eigen::MatrixXd
R(void)
{
    return R_;
}

};

#endif //MODELS_H