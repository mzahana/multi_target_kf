// coordinated_turn.h

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

#ifndef COORDINATED_TURN_H
#define COORDINATED_TURN_H

#include "multi_target_kf/structs.h"
#include <vector>
#include <cmath> // For sin and cos functions

class CoordinatedTurnModel
{
private:
    Eigen::MatrixXd F_; /* State transition Jacobian matrix */
    Eigen::MatrixXd H_; /* Observation Jacobian matrix */
    Eigen::MatrixXd Q_; /* Process covariance matrix */
    Eigen::MatrixXd P_; /* State covariance estimate */
    Eigen::MatrixXd R_; /* Measurements covariance matrix */
    Eigen::VectorXd x_; /* Current state vector */

    unsigned int NUM_STATES = 7; /* State dimension */
    unsigned int NUM_MEASUREMENTS = 3; /* Measurements dimension */

    double dt_; /* Prediction sampling time */
    double current_t_; /* Current time stamp */

    bool debug_;

    double sigma_a_; /* Standard deviation of acceleration noise */
    double sigma_omega_; /* Standard deviation of turn rate noise */
    double sigma_v_; /* Standard deviation of velocity noise */
    double sigma_p_; /* Standard deviation of position noise */

public:
    /* Constructor */
    CoordinatedTurnModel()
    {
        init();
    }
    ~CoordinatedTurnModel(){}

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
            printf("WARN [CoordinatedTurnModel::setDt] dt < 0 \n");
            return false;
        }
        dt_ = dt;
        return true;
    }

    bool setSigmaA(double sigma_a)
    {
        if (sigma_a <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::setSigmaA] sigma_a <=0 \n");
            return false;
        }
        sigma_a_ = sigma_a;

        if (debug())
            std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }

    bool setSigmaOmega(double sigma_omega)
    {
        if (sigma_omega <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::setSigmaOmega] sigma_omega <=0 \n");
            return false;
        }
        sigma_omega_ = sigma_omega;

        if (debug())
            std::cout << " sigma_omega: " << sigma_omega_ << "\n";
        return true;
    }

    bool setSigmaP(double sigma_p)
    {
        if (sigma_p <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::setSigmaP] sigma_p <= 0 \n");
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
            printf("[CoordinatedTurnModel::f] Calculating f \n");

        if (dt <= 0)
        {
            printf("[CoordinatedTurnModel::f] dt is <= 0. Returning same x \n");
            return x;
        }

        // State vector: [px, py, pz, v, heading (theta), climb angle (gamma), turn rate (omega)]
        // Indices:        0    1    2    3         4                5                6

        double px = x(0);
        double py = x(1);
        double pz = x(2);
        double v = x(3);
        double theta = x(4);
        double gamma = x(5);
        double omega = x(6);

        Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(NUM_STATES);

        if (fabs(omega) > 1e-5)
        {
            // With turn rate
            double theta_new = theta + omega * dt;
            x_pred(0) = px + (v / omega) * (sin(theta_new) - sin(theta)) * cos(gamma);
            x_pred(1) = py + (v / omega) * (-cos(theta_new) + cos(theta)) * cos(gamma);
            x_pred(2) = pz + v * dt * sin(gamma);
        }
        else
        {
            // Straight line motion (omega ~ 0)
            x_pred(0) = px + v * dt * cos(theta) * cos(gamma);
            x_pred(1) = py + v * dt * sin(theta) * cos(gamma);
            x_pred(2) = pz + v * dt * sin(gamma);
        }

        x_pred(3) = v; // Assuming constant speed
        x_pred(4) = theta + omega * dt; // Update heading
        x_pred(5) = gamma; // Assuming constant climb angle
        x_pred(6) = omega; // Assuming constant turn rate

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
            printf("[CoordinatedTurnModel::F] Calculating F \n");

        double v = x(3);
        double theta = x(4);
        double gamma = x(5);
        double omega = x(6);

        F_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        if (fabs(omega) > 1e-5)
        {
            // With turn rate
            double omega_dt = omega * dt;
            double sin_theta = sin(theta);
            double cos_theta = cos(theta);
            double sin_theta_omega_dt = sin(theta + omega_dt);
            double cos_theta_omega_dt = cos(theta + omega_dt);
            double omega2 = omega * omega;

            // Partial derivatives calculated for non-zero omega
            F_(0, 0) = 1.0;
            F_(0, 3) = (1.0 / omega) * (sin_theta_omega_dt - sin_theta) * cos(gamma);
            F_(0, 4) = (v / omega) * (cos_theta_omega_dt - cos_theta) * cos(gamma);
            F_(0, 5) = -(v / omega) * (sin_theta_omega_dt - sin_theta) * sin(gamma);
            F_(0, 6) = (v / omega2) * (sin_theta_omega_dt - sin_theta - omega_dt * cos_theta_omega_dt) * cos(gamma);

            F_(1, 1) = 1.0;
            F_(1, 3) = (1.0 / omega) * (-cos_theta_omega_dt + cos_theta) * cos(gamma);
            F_(1, 4) = (v / omega) * (sin_theta_omega_dt - sin_theta) * cos(gamma);
            F_(1, 5) = -(v / omega) * (-cos_theta_omega_dt + cos_theta) * sin(gamma);
            F_(1, 6) = (v / omega2) * (-cos_theta_omega_dt + cos_theta - omega_dt * sin_theta_omega_dt) * cos(gamma);

            F_(2, 2) = 1.0;
            F_(2, 3) = dt * sin(gamma);
            F_(2, 5) = v * dt * cos(gamma);
            // F_(2, 6) = 0; // Zero because pz does not depend on omega directly

            F_(3, 3) = 1.0; // Assuming constant speed
            F_(4, 4) = 1.0;
            F_(4, 6) = dt;
            F_(5, 5) = 1.0; // Assuming constant climb angle
            F_(6, 6) = 1.0; // Assuming constant turn rate
        }
        else
        {
            // Straight line motion (omega ~ 0)
            F_(0, 0) = 1.0;
            F_(0, 3) = dt * cos(theta) * cos(gamma);
            F_(0, 4) = -v * dt * sin(theta) * cos(gamma);
            F_(0, 5) = -v * dt * cos(theta) * sin(gamma);

            F_(1, 1) = 1.0;
            F_(1, 3) = dt * sin(theta) * cos(gamma);
            F_(1, 4) = v * dt * cos(theta) * cos(gamma);
            F_(1, 5) = -v * dt * sin(theta) * sin(gamma);

            F_(2, 2) = 1.0;
            F_(2, 3) = dt * sin(gamma);
            F_(2, 5) = v * dt * cos(gamma);

            F_(3, 3) = 1.0; // Assuming constant speed
            F_(4, 4) = 1.0;
            F_(4, 6) = dt;
            F_(5, 5) = 1.0; // Assuming constant climb angle
            F_(6, 6) = 1.0; // Assuming constant turn rate
        }

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
            printf("[CoordinatedTurnModel::h] Calculating h \n");

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
            printf("[CoordinatedTurnModel::H] Returning H_ \n");

        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // Partial derivative of position x w.r.t state x(0)
        H_(1, 1) = 1.0; // Partial derivative of position y w.r.t state x(1)
        H_(2, 2) = 1.0; // Partial derivative of position z w.r.t state x(2)

        return H_;
    }

    bool H(Eigen::MatrixXd M)
    {
        if (debug_)
            printf("[CoordinatedTurnModel::H] Setting H_ \n");

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
            printf("ERROR [CoordinatedTurnModel::Q] dt<=0 \n");
            return false;
        }

        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);

        // Process noise covariance matrix
        // Assuming process noise in speed (v), heading (theta), and turn rate (omega)

        Q_(3, 3) = sigma_a_ * sigma_a_ * dt; // Speed noise
        Q_(4, 4) = sigma_omega_ * sigma_omega_ * dt; // Heading noise
        Q_(6, 6) = sigma_omega_ * sigma_omega_ * dt; // Turn rate noise

        // Optionally, you can add small noise to other states if necessary

        return true;
    }

    bool Q(double dt, double sigma_a, double sigma_omega)
    {
        if (dt <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::Q] dt<=0 \n");
            return false;
        }
        if (sigma_a <= 0 || sigma_omega <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::Q] sigma_a or sigma_omega <=0 \n");
            return false;
        }

        sigma_a_ = sigma_a;
        sigma_omega_ = sigma_omega;

        return Q(dt);
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
            printf("ERROR [CoordinatedTurnModel::Q] Input vector size != NUM_STATES. %lu != %u  \n", v.size(), NUM_STATES);
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
            printf("[CoordinatedTurnModel::R] Setting R_ from a matrix \n");

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
            printf("[CoordinatedTurnModel::R] Setting diagonal R_ from a vector \n");

        if (v.size() != NUM_MEASUREMENTS)
        {
            printf("ERROR [CoordinatedTurnModel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u \n", v.size(), NUM_MEASUREMENTS);
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

    bool P(double sigma_p, double sigma_v)
    {
        if (sigma_p <= 0 || sigma_v <= 0)
        {
            printf("ERROR [CoordinatedTurnModel::P] sigma_p or sigma_v <=0 \n");
            return false;
        }

        if (debug())
        {
            printf("[CoordinatedTurnModel::P] Setting P from standard deviations \n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0, 0, 3, 3) = sigma_p * sigma_p * Eigen::MatrixXd::Identity(3, 3);
        P_(3, 3) = sigma_v * sigma_v; // For speed
        P_(4, 4) = 1.0; // For heading angle (theta)
        P_(5, 5) = 1.0; // For climb angle (gamma)
        P_(6, 6) = 0.1; // For turn rate (omega)

        if (debug())
            std::cout << "P: \n" << P_ << "\n";

        return true;
    }

    bool P(Eigen::MatrixXd M)
    {
        if (debug_)
        {
            printf("[CoordinatedTurnModel::P] Setting P from a matrix \n");
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
            printf("[CoordinatedTurnModel::predictX] Predicting x \n");

        if (dt <= 0)
        {
            printf("WARN [CoordinatedTurnModel::predictX] dt = %f <= 0. Returning same state \n", dt);
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
            printf("[CoordinatedTurnModel::predictX] Done predicting state \n");
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
            printf("[CoordinatedTurnModel::updateX] Updating x \n");

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
            printf("[CoordinatedTurnModel::updateX] Done updating state \n");
            printf("[CoordinatedTurnModel::updateX] Norm of innovation y = %f \n", y.norm());
            std::cout << "[CoordinatedTurnModel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[CoordinatedTurnModel::updateX] corrected state: \n" << xx.x << "\n";
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
            printf("ERROR [CoordinatedTurnModel::logLikelihood] Non-positive definite S matrix \n");
            return -std::numeric_limits<double>::infinity();
        }

        double tmp = y_hat.transpose() * S.inverse() * y_hat;

        double LL = -0.5 * (tmp + log(det_S) + NUM_MEASUREMENTS * log(2 * M_PI)); // Log-likelihood

        if (debug_)
            printf("[CoordinatedTurnModel::logLikelihood] Done computing logLikelihood. L= %f \n", LL);

        return LL;
    }

    kf_state initStateFromMeasurements(sensor_measurement z)
    {
        if (debug_)
            printf("[CoordinatedTurnModel::initStateFromMeasurements] Initializing state from measurements \n");

        kf_state state;
        state.time_stamp = z.time_stamp;
        state.x = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        state.x(0) = z.z(0); // x position
        state.x(1) = z.z(1); // y position
        state.x(2) = z.z(2); // z position

        // Initialize other states as needed
        state.x(3) = 1.0; // Initial speed estimate
        state.x(4) = 0.0; // Initial heading angle
        state.x(5) = 0.0; // Initial climb angle
        state.x(6) = 0.0; // Initial turn rate

        state.P = P_; // Use initial covariance

        return state;
    }

    double computeDistance(kf_state xx, sensor_measurement zz)
    {
        auto y = zz.z - h(xx.x); // Assumes length of xx.x = length of zz.z
        auto nrm = y.norm();
        if (debug_)
            printf("[CoordinatedTurnModel::computeDistance] Distance between state and measurement = %f \n", nrm);
        return nrm;
    }
};

#endif // COORDINATED_TURN_H
