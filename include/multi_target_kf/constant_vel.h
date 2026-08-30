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

// include/multi_target_kf/constant_vel.h
#ifndef CONSTANT_VEL_H
#define CONSTANT_VEL_H

#include "multi_target_kf/motion_model.h"
#include <vector>

/**
 * @brief Constant velocity model
 */
class ConstantVelModel : public MotionModel {
private:
    Eigen::VectorXd acc_variance_; /* 3D vector for acceleration variances */
    double sigma_a_, sigma_p_, sigma_v_;

    /* Log-storm suppression state for the "dt <= 0" warning on the
     * per-iteration path. Logging only -- see MotionModel::DtWarnState. */
    DtWarnState predictx_dt_warn_;

public:
    /* Constructor */
    ConstantVelModel() : sigma_a_(0.0), sigma_p_(0.0), sigma_v_(0.0) {
        NUM_STATES = 6;        // [px, py, pz, vx, vy, vz]
        NUM_MEASUREMENTS = 3;  // [px, py, pz]
        init();
        acc_variance_ = Eigen::VectorXd::Zero(3);
    }
    
    ~ConstantVelModel() override {}

    bool init() override {
        F_.resize(NUM_STATES, NUM_STATES);
        F_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
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
    
    // Model-specific setters
    bool setSigmaA(double sigma_a) {
        if (sigma_a <= 0) {
            printf("ERROR [ConstVel::setSigmaA] sigma_a <= 0\n");
            return false;
        }
        sigma_a_ = sigma_a;
        if (debug()) std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }
    
    bool setSigmaP(double sigma_p) {
        if (sigma_p <= 0) {
            printf("ERROR [ConstVel::setSigmaP] sigma_p <= 0\n");
            return false;
        }
        sigma_p_ = sigma_p;
        if (debug()) std::cout << " sigma_p: " << sigma_p_ << "\n";
        return true;
    }
    
    bool setSigmaV(double sigma_v) {
        if (sigma_v <= 0) {
            printf("ERROR [setSigmaV] sigma_v <= 0\n");
            return false;
        }
        sigma_v_ = sigma_v;
        if (debug()) std::cout << " sigma_v: " << sigma_v_ << "\n";
        return true;
    }
    
    bool setAccVar(Eigen::VectorXd a) {
        if (a.size() != 3) return false;
        acc_variance_.resize(3); 
        acc_variance_ = a;
        return true;
    }
    
    // Implementation of base class virtual methods
    Eigen::VectorXd f(Eigen::VectorXd x, double dt) override {
        if (debug()) printf("[ConstantVelModel::f] Calculating f\n");
        
        if (dt <= 0) {
            printf("[ConstantVelModel::f] dt is <= 0. Returning same x\n");
            return x;
        }
        
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        A.block(0, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);
        
        return (A * x);
    }
    
    // Convenience methods using stored dt_
    Eigen::VectorXd f() { return f(x_, dt_); }
    Eigen::VectorXd f(Eigen::VectorXd x) { return f(x, dt_); }
    
    Eigen::MatrixXd F(double dt) override {
        if (debug()) printf("[ConstantVelModel::F] Calculating F\n");
        
        F_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        F_.block(0, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);
        
        return F_;
    }
    
    Eigen::MatrixXd F() { return F_; }
    
    bool F(Eigen::MatrixXd M) override {
        if (M.cols() == NUM_STATES && M.rows() == NUM_STATES) {
            F_.resize(NUM_STATES, NUM_STATES);
            F_ = M;
            return true;
        }
        return false;
    }
    
    Eigen::VectorXd h(Eigen::VectorXd x) override {
        if (debug()) printf("[ConstantVelModel::h] Calculating h\n");
        
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // observing x
        H_(1, 1) = 1.0; // observing y
        H_(2, 2) = 1.0; // observing z
        
        Eigen::VectorXd z = H_ * x;
        if (debug()) std::cout << "h(x) = \n" << z << "\n";
        return z;
    }
    
    Eigen::MatrixXd H() override {
        if (debug()) printf("[ConstantVelModel::H] Returning H_\n");
        
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // observing x
        H_(1, 1) = 1.0; // observing y
        H_(2, 2) = 1.0; // observing z
        
        return H_;
    }
    
    bool H(Eigen::MatrixXd M) override {
        if (debug_) printf("[ConstVel::H] Setting H_\n");
        
        if (M.cols() == NUM_STATES && M.rows() == NUM_MEASUREMENTS) {
            H_.resize(NUM_MEASUREMENTS, NUM_STATES);
            H_ = M;
            return true;
        }
        return false;
    }
    
    Eigen::MatrixXd Q() { return Q_; }
    
    Eigen::MatrixXd Q(double dt) override {
        if (dt <= 0) return Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Initialize
        Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Construct the upper left block, Qpp
        Q_.block(0, 0, 3, 3) = dt * dt * dt / 3 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the upper right block, Qpv
        Q_.block(0, 3, 3, 3) = dt * dt / 2 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the lower left block, Qvp
        Q_.block(3, 0, 3, 3) = dt * dt / 2 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the lower right block, Qvv
        Q_.block(3, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);
        
        Q_ = sigma_a_ * sigma_a_ * Q_;
        
        return Q_;
    }
    
    bool Q(double dt, double sigma_a) {
        if (dt <= 0) {
            printf("ERROR [ConstVel::Q] dt <= 0\n");
            return false;
        }
        if (sigma_a <= 0) {
            printf("ERROR [ConstVel::Q] sigma_a <= 0\n");
            return false;
        }
        
        // Initialize
        Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Construct the upper left block, Qpp
        Q_.block(0, 0, 3, 3) = dt * dt * dt * dt / 4 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the upper right block, Qpv
        Q_.block(0, 3, 3, 3) = dt * dt * dt / 2 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the lower left block, Qvp
        Q_.block(3, 0, 3, 3) = dt * dt * dt / 2 * Eigen::MatrixXd::Identity(3, 3);
        // Construct the lower right block, Qvv
        Q_.block(3, 3, 3, 3) = dt * dt * Eigen::MatrixXd::Identity(3, 3);
        
        Q_ = sigma_a * sigma_a * Q_;
        
        if (debug()) std::cout << "Q: \n" << Q_ << "\n";
        
        return true;
    }
    
    bool Q(Eigen::MatrixXd M) override {
        if (M.cols() == NUM_STATES && M.rows() == NUM_STATES) {
            Q_.resize(NUM_STATES, NUM_STATES);
            Q_ = M;
            return true;
        }
        return false;
    }
    
    bool Q(std::vector<double> v) override {
        if (v.size() != NUM_STATES) {
            printf("ERROR [ConstVel::Q] Input vector size != NUM_STATES. %lu != %u\n", v.size(), NUM_STATES);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        for (unsigned int i = 0; i < NUM_STATES; i++) temp(i) = v[i];
        
        Q_ = temp.asDiagonal();
        return true;
    }
    
    Eigen::MatrixXd R() override {
        return R_;
    }
    
    bool R(Eigen::MatrixXd M) override {
        if (debug_) printf("[ConstVel::R] Setting R_ from a matrix\n");
        
        if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS) {
            R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
            R_ = M;
            return true;
        }
        return false;
    }
    
    bool R(std::vector<double> v) override {
        if (debug_) printf("[ConstVel::R] Setting diagonal R_ from a vector\n");
        
        if (v.size() != NUM_MEASUREMENTS) {
            printf("ERROR [ConstVel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u\n", v.size(), NUM_MEASUREMENTS);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, 1);
        for (unsigned int i = 0; i < NUM_MEASUREMENTS; i++) temp(i) = v[i];
        
        R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        R_ = temp.asDiagonal();
        if (debug_) std::cout << "R_=" << std::endl << R_ << std::endl;
        return true;
    }
    
    Eigen::MatrixXd P() override {
        return P_;
    }
    
    bool P(double sigma_p, double sigma_v) {
        if (sigma_p <= 0 || sigma_v <= 0) {
            printf("ERROR [ConstVel::P] sigma_p or sigma_v <= 0\n");
            return false;
        }
        
        if (debug()) {
            printf("[ConstantVelModel::P] Setting P from standard deviations\n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0, 0, 3, 3) = sigma_p * sigma_p * Eigen::MatrixXd::Identity(3, 3);
        P_.block(3, 3, 3, 3) = sigma_v * sigma_v * Eigen::MatrixXd::Identity(3, 3);
        
        if (debug()) std::cout << "P: \n" << P_ << "\n";
        
        return true;
    }
    
    bool P(Eigen::MatrixXd M) override {
        if (debug_) {
            printf("[ConstVel::P] Setting P from a matrix\n");
        }
        
        if (M.rows() == NUM_STATES && M.cols() == NUM_STATES) {
            P_.resize(NUM_STATES, NUM_STATES);
            P_ = M;
            if (debug_) std::cout << "P: " << std::endl << P_ << std::endl;
            return true;
        }
        return false;
    }
    
    Eigen::VectorXd getx() {
        return x_;
    }
    
    bool setx(Eigen::VectorXd v) {
        if (v.cols() == 1 && v.rows() == NUM_STATES) {
            x_ = v;
            return true;
        }
        return false;
    }
    
    kf_state predictX(kf_state s, double dt) override {
        if (debug_) printf("[ConstVel::predictX] Predicting x\n");
        
        if (dt <= 0) {
            reportNonPositiveDt(predictx_dt_warn_, "ConstVel::predictX", dt);
            return s;
        }
        clearNonPositiveDt(predictx_dt_warn_, "ConstVel::predictX");
        
        if (debug_) printf("[ConstVel::predictX] det(P) of current state: %f\n", s.P.determinant());
        
        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;
        
        auto FF = F(dt);
        xx.P = FF * s.P * FF.transpose() + Q(dt);
        xx.x = f(s.x, dt);
        
        if (debug_) {
            printf("[ConstVel::predictX] ||x_new - x_old|| = %f\n", (xx.x - s.x).norm());
            printf("[ConstVel::predictX] det(P) of predicted state: %f\n", xx.P.determinant());
            std::cout << "[predictX] old P = \n" << s.P << "\n";
            std::cout << "[predictX] old x = \n" << s.x << "\n";
            std::cout << "[predictX] new P = \n" << xx.P << "\n";
            std::cout << "[predictX] new x = \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    kf_state updateX(sensor_measurement z, kf_state s) override {
        if (debug_) printf("[ConstVel::updateX] Updating x\n");
        
        kf_state xx;
        xx.time_stamp = z.time_stamp;
        
        Eigen::VectorXd y = z.z - h(s.x); // innovation
        Eigen::MatrixXd S = H() * s.P * H().transpose() + R_; // innovation covariance
        Eigen::MatrixXd K = s.P * H().transpose() * S.inverse(); // optimal Kalman gain
        xx.x = s.x + K * y;
        xx.P = (Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) - K * H()) * s.P;
        
        if (debug_) {
            printf("[ConstVel::updateX] Done updating state\n");
            printf("[ConstVel::updateX] Norm of innovation y = %f\n", y.norm());
            std::cout << "[ConstVel::updateX] predicted state P: \n" << s.P << "\n";
            std::cout << "[ConstVel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstVel::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstVel::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstVel::updateX] corrected state: \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    kf_state updateX(sensor_measurement z, kf_state s, double dt) {
        if (debug_) printf("[ConstVel::updateX] Updating x\n");
        
        kf_state xx;
        xx.time_stamp = z.time_stamp;
        
        Eigen::VectorXd y = z.z - h(s.x); // innovation
        Eigen::MatrixXd S = H() * s.P * H().transpose() + dt * dt * R_; // innovation covariance
        Eigen::MatrixXd K = s.P * H().transpose() * S.inverse(); // optimal Kalman gain
        xx.x = s.x + K * y;
        xx.P = (Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) - K * H()) * s.P;
        
        if (debug_) {
            printf("[ConstVel::updateX] Done updating state\n");
            printf("[ConstVel::updateX] Norm of innovation y = %f\n", y.norm());
            std::cout << "[ConstVel::updateX] predicted state P: \n" << s.P << "\n";
            std::cout << "[ConstVel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstVel::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstVel::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstVel::updateX] corrected state: \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    double logLikelihood(kf_state xx, sensor_measurement z) override {
        Eigen::VectorXd y_hat = z.z - h(xx.x); // Innovation
        
        Eigen::MatrixXd S = R_ + H() * xx.P * H().transpose(); // Innovation covariance
        
        double det_S = S.determinant();
        if (det_S <= 0) {
            printf("ERROR [ConstantVelModel::logLikelihood] Non-positive definite S matrix. det(S) = %f\n", det_S);
            return -std::numeric_limits<double>::infinity();
        }
        
        double tmp = y_hat.transpose() * S.inverse() * y_hat;
        
        double LL = -0.5 * (tmp + log(det_S) + NUM_MEASUREMENTS * log(2 * M_PI)); // Log-likelihood
        
        if (debug_) printf("[ConstantVelModel::logLikelihood] Done computing logLikelihood. L= %f\n", LL);
        
        return LL;
    }
    
    kf_state initStateFromMeasurements(sensor_measurement z) override {
        if (debug_) printf("[ConstVel::initStateFromMeasurements] Initializing state from measurements\n");
        
        kf_state state;
        state.time_stamp = z.time_stamp;
        state.x = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        state.x(0) = z.z(0);
        state.x(1) = z.z(1);
        state.x(2) = z.z(2);
        
        state.P = P_;
        
        return state;
    }
    
    double computeDistance(kf_state xx, sensor_measurement zz) override {
        auto y = zz.z - h(xx.x); // assumes length of xx.x >= length of zz.z
        auto nrm = y.norm();
        if (debug_) printf("[ConstVel::computeDistance] Distance between state and measurement = %f\n", nrm);
        return nrm;
    }
};

#endif // CONSTANT_VEL_H