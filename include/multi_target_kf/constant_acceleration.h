/*
BSD 3-Clause License

Copyright (c) 2024, Mohamed Abdelkader Zahana
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

#ifndef CONSTANT_ACCELERATION_H
#define CONSTANT_ACCELERATION_H

#include "multi_target_kf/structs.h"
#include <vector>

class ConstantAccelerationModel
{
private:
    Eigen::MatrixXd F_; /* State transition Jacobian matrix */
    Eigen::MatrixXd H_; /* Observation Jacobian matrix */
    Eigen::MatrixXd Q_; /* Process covariance matrix */
    Eigen::MatrixXd P_; /* State covariance estimate */
    Eigen::MatrixXd R_; /* Measurements covariance matrix */
    Eigen::VectorXd x_; /* Current state vector */

    unsigned int NUM_STATES = 9; /* State dimension */
    unsigned int NUM_MEASUREMENTS = 3; /* Measurements dimension */

    double dt_; /* Prediction sampling time */
    double current_t_; /* Current time stamp */

    bool debug_;

    double sigma_a_; /* Standard deviation of acceleration noise */
    double sigma_p_; /* Standard deviation of position noise */
    double sigma_v_; /* Standard deviation of velocity noise */

public:
    /* Constructor */
    ConstantAccelerationModel()
    {
        init();
    }
    ~ConstantAccelerationModel(){}

    bool init(void)
    {
        F_.resize(NUM_STATES, NUM_STATES);
        F_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);

        Q_.resize(NUM_STATES, NUM_STATES);
        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        P_.resize(NUM_STATES, NUM_STATES);
        P_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        R_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_MEASUREMENTS);

        x_.resize(NUM_STATES, 1);
        x_ = Eigen::MatrixXd::Zero(NUM_STATES, 1);

        debug_ = false;
        dt_ = 0.01;
        current_t_ = 0.0;

        sigma_a_ = 1.0;
        sigma_p_ = 1.0;
        sigma_v_ = 1.0;

        return true;
    }

    unsigned int numStates()
    {
        return NUM_STATES;
    }

    bool debug(void)
    {
        return debug_;
    }

    void debug(bool d)
    {
        debug_ = d;
    }

    bool setNumStates(unsigned int n)
    {
        if (n < 1)
            return false;
        NUM_STATES = n;
        return true;
    }

    unsigned int numMeasurements()
    {
        return NUM_MEASUREMENTS;
    }

    bool setNumMeasurements(unsigned int n)
    {
        if (n < 1)
            return false;
        NUM_MEASUREMENTS = n;
        return true;
    }

    void setCurrentTimeStamp(double t)
    {
        current_t_ = t;
    }

    bool setDt(double dt)
    {
        if (dt < 0)
        {
            printf("WARN [ConstantAccelerationModel::setDt] dt < 0 \n");
            return false;
        }
        dt_ = dt;
        return true;
    }

    bool setSigmaA(double sigma_a)
    {
        if (sigma_a <= 0)
        {
            printf("ERROR [ConstantAccelerationModel::setSigmaA] sigma_a <=0 \n");
            return false;
        }
        sigma_a_ = sigma_a;

        if (debug())
            std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }

    bool setSigmaP(double sigma_p)
    {
        if (sigma_p <= 0)
        {
            printf("ERROR [ConstantAccelerationModel::setSigmaP] sigma_p <= 0 \n");
            return false;
        }
        sigma_p_ = sigma_p;
        if (debug())
            std::cout << " sigma_p: " << sigma_p_ << "\n";
        return true;
    }

    bool setSigmaV(double sigma_v)
    {
        if (sigma_v <= 0)
        {
            printf("ERROR [setSigmaV] sigma_v <= 0 \n");
            return false;
        }
        sigma_v_ = sigma_v;
        if (debug())
            std::cout << " sigma_v: " << sigma_v_ << "\n";
        return true;
    }

    /**
     * @brief Computes the state prediction using model f(x)
     * @param x state vector
     * @param dt sampling time in seconds
     * @return predicted state vector
     */
    Eigen::VectorXd f(Eigen::VectorXd x, double dt)
    {
        if (debug())
            printf("[ConstantAccelerationModel::f] Calculating f \n");

        if (dt <= 0)
        {
            printf("[ConstantAccelerationModel::f] dt is <= 0. Returning same x \n");
            return x;
        }

        // State vector: [px, py, pz, vx, vy, vz, ax, ay, az]
        // Indices:        0    1    2    3    4    5    6    7    8

        Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(NUM_STATES);

        // Position update
        x_pred(0) = x(0) + x(3) * dt + 0.5 * x(6) * dt * dt;
        x_pred(1) = x(1) + x(4) * dt + 0.5 * x(7) * dt * dt;
        x_pred(2) = x(2) + x(5) * dt + 0.5 * x(8) * dt * dt;

        // Velocity update
        x_pred(3) = x(3) + x(6) * dt;
        x_pred(4) = x(4) + x(7) * dt;
        x_pred(5) = x(5) + x(8) * dt;

        // Acceleration update (assuming constant acceleration)
        x_pred(6) = x(6);
        x_pred(7) = x(7);
        x_pred(8) = x(8);

        return x_pred;
    }

    Eigen::VectorXd f()
    {
        return f(x_, dt_);
    }

    Eigen::VectorXd f(Eigen::VectorXd x)
    {
        return f(x, dt_);
    }

    /**
     * @brief Computes the state transition Jacobian matrix F
     * @param x state vector
     * @param dt sampling time in seconds
     * @return Jacobian F matrix
     */
    Eigen::MatrixXd F(Eigen::VectorXd x, double dt)
    {
        if (debug())
            printf("[ConstantAccelerationModel::F] Calculating F \n");

        F_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

        // Partial derivatives for position w.r.t velocity and acceleration
        F_(0, 3) = dt;
        F_(1, 4) = dt;
        F_(2, 5) = dt;

        F_(0, 6) = 0.5 * dt * dt;
        F_(1, 7) = 0.5 * dt * dt;
        F_(2, 8) = 0.5 * dt * dt;

        // Partial derivatives for velocity w.r.t acceleration
        F_(3, 6) = dt;
        F_(4, 7) = dt;
        F_(5, 8) = dt;

        // Acceleration remains constant
        // F_(6, 6) = 1.0;
        // F_(7, 7) = 1.0;
        // F_(8, 8) = 1.0;

        return F_;
    }

    Eigen::MatrixXd F(double dt)
    {
        return F(x_, dt);
    }

    Eigen::MatrixXd F(Eigen::VectorXd x)
    {
        return F(x, dt_);
    }

    bool F(Eigen::MatrixXd M)
    {
        if (M.cols() == NUM_STATES && M.rows() == NUM_STATES)
        {
            F_.resize(NUM_STATES, NUM_STATES);
            F_ = M;
            return true;
        }
        else
            return false;
    }

    Eigen::MatrixXd F()
    {
        return F_;
    }

    /**
     * @brief Computes the observation model h(x)
     * @param x state vector
     * @return observation z vector
     */
    Eigen::VectorXd h(Eigen::VectorXd x)
    {
        if (debug())
            printf("[ConstantAccelerationModel::h] Calculating h \n");

        // Observation model: we observe positions directly
        Eigen::VectorXd z = Eigen::VectorXd::Zero(NUM_MEASUREMENTS);
        z(0) = x(0); // x position
        z(1) = x(1); // y position
        z(2) = x(2); // z position

        return z;
    }

    /**
     * @brief Returns observation Jacobian matrix H_
     */
    Eigen::MatrixXd H(void)
    {
        if (debug())
            printf("[ConstantAccelerationModel::H] Returning H_ \n");

        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // Partial derivative of position x w.r.t state x(0)
        H_(1, 1) = 1.0; // Partial derivative of position y w.r.t state x(1)
        H_(2, 2) = 1.0; // Partial derivative of position z w.r.t state x(2)

        return H_;
    }

    bool H(Eigen::MatrixXd M)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::H] Setting H_ \n");

        if (M.cols() == NUM_STATES && M.rows() == NUM_MEASUREMENTS)
        {
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
    Eigen::MatrixXd Q(void)
    {
        return Q_;
    }

    bool Q(double dt)
    {
        if (dt <= 0)
        {
            printf("ERROR [ConstantAccelerationModel::Q] dt<=0 \n");
            return false;
        }

        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        // Process noise covariance matrix
        // Assuming acceleration noise in ax, ay, az
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        double q11 = 0.25 * dt4 * sigma_a_ * sigma_a_; // position-position covariance
        double q13 = 0.5 * dt3 * sigma_a_ * sigma_a_;  // position-velocity covariance
        double q22 = dt2 * sigma_a_ * sigma_a_;        // velocity-velocity covariance
        double q33 = dt * sigma_a_ * sigma_a_;         // acceleration-acceleration covariance

        // Position covariance
        Q_(0, 0) = q11;  // cov(x, x)
        Q_(0, 3) = q13;  // cov(x, v_x)
        Q_(0, 6) = q13;  // cov(x, a_x)

        Q_(1, 1) = q11;  // cov(y, y)
        Q_(1, 4) = q13;  // cov(y, v_y)
        Q_(1, 7) = q13;  // cov(y, a_y)

        Q_(2, 2) = q11;  // cov(z, z)
        Q_(2, 5) = q13;  // cov(z, v_z)
        Q_(2, 8) = q13;  // cov(z, a_z)

        // Velocity covariance
        Q_(3, 0) = Q_(0, 3);  // cov(v_x, x)
        Q_(3, 3) = q22;       // cov(v_x, v_x)
        Q_(3, 6) = 0.5 * dt2 * sigma_a_ * sigma_a_;  // cov(v_x, a_x)

        Q_(4, 1) = Q_(1, 4);  // cov(v_y, y)
        Q_(4, 4) = q22;       // cov(v_y, v_y)
        Q_(4, 7) = 0.5 * dt2 * sigma_a_ * sigma_a_;  // cov(v_y, a_y)

        Q_(5, 2) = Q_(2, 5);  // cov(v_z, z)
        Q_(5, 5) = q22;       // cov(v_z, v_z)
        Q_(5, 8) = 0.5 * dt2 * sigma_a_ * sigma_a_;  // cov(v_z, a_z)

        // Acceleration covariance
        Q_(6, 0) = Q_(0, 6);  // cov(a_x, x)
        Q_(6, 3) = Q_(3, 6);  // cov(a_x, v_x)
        Q_(6, 6) = q33;       // cov(a_x, a_x)

        Q_(7, 1) = Q_(1, 7);  // cov(a_y, y)
        Q_(7, 4) = Q_(4, 7);  // cov(a_y, v_y)
        Q_(7, 7) = q33;       // cov(a_y, a_y)

        Q_(8, 2) = Q_(2, 8);  // cov(a_z, z)
        Q_(8, 5) = Q_(5, 8);  // cov(a_z, v_z)
        Q_(8, 8) = q33;       // cov(a_z, a_z)

        return true;
    }


    bool Q(Eigen::MatrixXd M)
    {
        if (M.cols() == NUM_STATES && M.rows() == NUM_STATES)
        {
            Q_.resize(NUM_STATES, NUM_STATES);
            Q_ = M;
            return true;
        }
        else
            return false;
    }

    bool Q(std::vector<double> v)
    {
        if (v.size() != NUM_STATES)
        {
            printf("ERROR [ConstantAccelerationModel::Q] Input vector size != NUM_STATES. %lu != %u  \n", v.size(), NUM_STATES);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        for (unsigned int i = 0; i < NUM_STATES; i++)
            temp(i) = v[i];

        Q_ = temp.asDiagonal();
        return true;
    }

    Eigen::MatrixXd R(void)
    {
        return R_;
    }

    bool R(Eigen::MatrixXd M)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::R] Setting R_ from a matrix \n");

        if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS)
        {
            R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
            R_ = M;
            return true;
        }
        else
            return false;
    }

    bool R(std::vector<double> v)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::R] Setting diagonal R_ from a vector \n");

        if (v.size() != NUM_MEASUREMENTS)
        {
            printf("ERROR [ConstantAccelerationModel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u \n", v.size(), NUM_MEASUREMENTS);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, 1);
        for (unsigned int i = 0; i < NUM_MEASUREMENTS; i++)
            temp(i) = v[i];

        R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        R_ = temp.asDiagonal();
        if (debug_)
            std::cout << "R_=" << std::endl << R_ << std::endl;
        return true;
    }

    Eigen::MatrixXd P(void)
    {
        return P_;
    }

    bool P(double sigma_p, double sigma_v, double sigma_a)
    {
        if (sigma_p <= 0 || sigma_v <= 0 || sigma_a <= 0)
        {
            printf("ERROR [ConstantAccelerationModel::P] One or more sigma values are <= 0 \n");
            return false;
        }

        if (debug())
        {
            printf("[ConstantAccelerationModel::P] Setting P from standard deviations \n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0, 0, 3, 3) = sigma_p * sigma_p * Eigen::MatrixXd::Identity(3, 3); // Position covariance
        P_.block(3, 3, 3, 3) = sigma_v * sigma_v * Eigen::MatrixXd::Identity(3, 3); // Velocity covariance
        P_.block(6, 6, 3, 3) = sigma_a * sigma_a * Eigen::MatrixXd::Identity(3, 3); // Acceleration covariance

        if (debug())
            std::cout << "P: \n" << P_ << "\n";

        return true;
    }

    bool P(Eigen::MatrixXd M)
    {
        if (debug_)
        {
            printf("[ConstantAccelerationModel::P] Setting P from a matrix \n");
        }

        if (M.rows() == NUM_STATES && M.cols() == NUM_STATES)
        {
            P_.resize(NUM_STATES, NUM_STATES);
            P_ = M;
            if (debug_)
                std::cout << "P: " << std::endl << P_ << std::endl;
            return true;
        }
        else
            return false;
    }

    Eigen::VectorXd getx(void)
    {
        return x_;
    }

    bool setx(Eigen::VectorXd v)
    {
        if (v.cols() == 1 && v.rows() == NUM_STATES)
        {
            x_ = v;
            return true;
        }
        else
            return false;
    }

    /**
     * @brief Prediction step of a discrete EKF using given state s and dt
     * @param s Current state (includes time, state, covariance)
     * @param dt Time step
     * @return Predicted state (includes time, state, covariance)
     */
    kf_state predictX(kf_state s, double dt)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::predictX] Predicting x \n");

        if (dt <= 0)
        {
            printf("WARN [ConstantAccelerationModel::predictX] dt = %f <= 0. Returning same state \n", dt);
            return s;
        }

        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;

        // Predict state
        xx.x = f(s.x, dt);

        // Compute Jacobian
        Eigen::MatrixXd FF = F(s.x, dt);

        // Predict covariance
        Q(dt); // Update Q_
        xx.P = FF * s.P * FF.transpose() + Q_;

        if (debug_)
        {
            printf("[ConstantAccelerationModel::predictX] Done predicting state \n");
            std::cout << "[predictX] new P = \n" << xx.P << "\n";
            std::cout << "[predictX] new x = \n" << xx.x << "\n";
        }

        return xx;
    }

    /**
     * @brief Update step of a discrete EKF using given state x and measurement z
     * @param z Sensor measurement
     * @param s Current state (includes time, state, covariance)
     * @return Updated state (includes time, state, covariance)
     */
    kf_state updateX(sensor_measurement z, kf_state s)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::updateX] Updating x \n");

        kf_state xx;
        xx.time_stamp = z.time_stamp;

        // Compute innovation
        Eigen::VectorXd y = z.z - h(s.x);

        // Compute Jacobian of h (in this case, H_)
        Eigen::MatrixXd HH = H();

        // Compute innovation covariance
        Eigen::MatrixXd S = HH * s.P * HH.transpose() + R_;

        // Compute Kalman gain
        Eigen::MatrixXd K = s.P * HH.transpose() * S.inverse();

        // Update state estimate
        xx.x = s.x + K * y;

        // Update covariance estimate
        xx.P = (Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) - K * HH) * s.P;

        if (debug_)
        {
            printf("[ConstantAccelerationModel::updateX] Done updating state \n");
            printf("[ConstantAccelerationModel::updateX] Norm of innovation y = %f \n", y.norm());
            std::cout << "[ConstantAccelerationModel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstantAccelerationModel::updateX] corrected state: \n" << xx.x << "\n";
        }

        return xx;
    }

    double logLikelihood(kf_state xx, sensor_measurement z)
    {
        Eigen::VectorXd y_hat = z.z - h(xx.x); // Innovation

        Eigen::MatrixXd S = R_ + H() * xx.P * H().transpose(); // Innovation covariance

        double det_S = S.determinant();
        if (det_S <= 0)
        {
            printf("ERROR [ConstantAccelerationModel::logLikelihood] Non-positive definite S matrix. det(S) = %f \n", det_S);
            return -std::numeric_limits<double>::infinity();
        }

        double tmp = y_hat.transpose() * S.inverse() * y_hat;

        double LL = -0.5 * (tmp + log(det_S) + NUM_MEASUREMENTS * log(2 * M_PI)); // Log-likelihood

        if (debug_)
            printf("[ConstantAccelerationModel::logLikelihood] Done computing logLikelihood. L= %f \n", LL);

        return LL;
    }

    kf_state initStateFromMeasurements(sensor_measurement z)
    {
        if (debug_)
            printf("[ConstantAccelerationModel::initStateFromMeasurements] Initializing state from measurements \n");

        kf_state state;
        state.time_stamp = z.time_stamp;
        state.x = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        state.x(0) = z.z(0); // x position
        state.x(1) = z.z(1); // y position
        state.x(2) = z.z(2); // z position

        // Initialize other states as needed
        state.x(3) = 0.0; // Initial velocity estimate vx
        state.x(4) = 0.0; // Initial velocity estimate vy
        state.x(5) = 0.0; // Initial velocity estimate vz

        state.x(6) = 0.0; // Initial acceleration estimate ax
        state.x(7) = 0.0; // Initial acceleration estimate ay
        state.x(8) = 0.0; // Initial acceleration estimate az

        state.P = P_; // Use initial covariance

        return state;
    }

    double computeDistance(kf_state xx, sensor_measurement zz)
    {
        auto y = zz.z - h(xx.x); // Assumes length of xx.x = length of zz.z
        auto nrm = y.norm();
        if (debug_)
            printf("[ConstantAccelerationModel::computeDistance] Distance between state and measurement = %f \n", nrm);
        return nrm;
    }
};

#endif // CONSTANT_ACCELERATION_H
