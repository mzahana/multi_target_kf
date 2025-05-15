// include/multi_target_kf/constant_accel.h
#ifndef CONSTANT_ACCEL_H
#define CONSTANT_ACCEL_H

#include "multi_target_kf/motion_model.h"
#include <vector>

/**
 * @brief Constant acceleration model
 * State vector: [px, py, pz, vx, vy, vz, ax, ay, az]
 * Measurement vector: [px, py, pz]
 */
class ConstantAccelModel : public MotionModel {
private:
    Eigen::VectorXd jerk_variance_; /* 3D vector for jerk variances */
    double sigma_j_, sigma_p_, sigma_v_, sigma_a_; /* Standard deviations for jerk, position, velocity, acceleration */

public:
    /* Constructor */
    ConstantAccelModel() : sigma_j_(0.0), sigma_p_(0.0), sigma_v_(0.0), sigma_a_(0.0) {
        NUM_STATES = 9;        // [px, py, pz, vx, vy, vz, ax, ay, az]
        NUM_MEASUREMENTS = 3;  // [px, py, pz]
        init();
        jerk_variance_ = Eigen::VectorXd::Zero(3);
    }
    
    ~ConstantAccelModel() override {}

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
    bool setSigmaJ(double sigma_j) {
        if (sigma_j <= 0) {
            printf("ERROR [ConstAccel::setSigmaJ] sigma_j <= 0\n");
            return false;
        }
        sigma_j_ = sigma_j;
        if (debug()) std::cout << " sigma_j: " << sigma_j_ << "\n";
        return true;
    }
    
    bool setSigmaP(double sigma_p) {
        if (sigma_p <= 0) {
            printf("ERROR [ConstAccel::setSigmaP] sigma_p <= 0\n");
            return false;
        }
        sigma_p_ = sigma_p;
        if (debug()) std::cout << " sigma_p: " << sigma_p_ << "\n";
        return true;
    }
    
    bool setSigmaV(double sigma_v) {
        if (sigma_v <= 0) {
            printf("ERROR [ConstAccel::setSigmaV] sigma_v <= 0\n");
            return false;
        }
        sigma_v_ = sigma_v;
        if (debug()) std::cout << " sigma_v: " << sigma_v_ << "\n";
        return true;
    }
    
    bool setSigmaA(double sigma_a) {
        if (sigma_a <= 0) {
            printf("ERROR [ConstAccel::setSigmaA] sigma_a <= 0\n");
            return false;
        }
        sigma_a_ = sigma_a;
        if (debug()) std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }
    
    bool setJerkVar(Eigen::VectorXd j) {
        if (j.size() != 3) return false;
        jerk_variance_.resize(3); 
        jerk_variance_ = j;
        return true;
    }
    
    // Implementation of base class virtual methods
    Eigen::VectorXd f(Eigen::VectorXd x, double dt) override {
        if (debug()) printf("[ConstantAccelModel::f] Calculating f\n");
        
        if (dt <= 0) {
            printf("[ConstantAccelModel::f] dt is <= 0. Returning same x\n");
            return x;
        }
        
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Update position with velocity and acceleration
        A.block(0, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);                  // x += v*dt
        A.block(0, 6, 3, 3) = 0.5 * dt * dt * Eigen::MatrixXd::Identity(3, 3);      // x += 0.5*a*dt^2
        
        // Update velocity with acceleration
        A.block(3, 6, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);                  // v += a*dt
        
        return (A * x);
    }
    
    // Convenience methods using stored dt_
    Eigen::VectorXd f() { return f(x_, dt_); }
    Eigen::VectorXd f(Eigen::VectorXd x) { return f(x, dt_); }
    
    Eigen::MatrixXd F(double dt) override {
        if (debug()) printf("[ConstantAccelModel::F] Calculating F\n");
        
        F_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Update position with velocity and acceleration
        F_.block(0, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);                  // x += v*dt
        F_.block(0, 6, 3, 3) = 0.5 * dt * dt * Eigen::MatrixXd::Identity(3, 3);      // x += 0.5*a*dt^2
        
        // Update velocity with acceleration
        F_.block(3, 6, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);                  // v += a*dt
        
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
        if (debug()) printf("[ConstantAccelModel::h] Calculating h\n");
        
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // observing x
        H_(1, 1) = 1.0; // observing y
        H_(2, 2) = 1.0; // observing z
        
        Eigen::VectorXd z = H_ * x;
        if (debug()) std::cout << "h(x) = \n" << z << "\n";
        return z;
    }
    
    Eigen::MatrixXd H() override {
        if (debug()) printf("[ConstantAccelModel::H] Returning H_\n");
        
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        H_(0, 0) = 1.0; // observing x
        H_(1, 1) = 1.0; // observing y
        H_(2, 2) = 1.0; // observing z
        
        return H_;
    }
    
    bool H(Eigen::MatrixXd M) override {
        if (debug_) printf("[ConstAccel::H] Setting H_\n");
        
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
        
        // Create a discretized continuous-time model Q matrix
        // For a constant acceleration model with jerk as process noise
        
        // Initialize
        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);
        
        // Calculate the coefficients for the position terms
        double dt3 = dt * dt * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;
        
        // Position variance terms (affected by jerk)
        Q_.block(0, 0, 3, 3) = (dt5 / 20.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Position-velocity covariance terms
        Q_.block(0, 3, 3, 3) = (dt4 / 8.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(3, 0, 3, 3) = (dt4 / 8.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Position-acceleration covariance terms
        Q_.block(0, 6, 3, 3) = (dt3 / 6.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(6, 0, 3, 3) = (dt3 / 6.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Velocity variance terms
        Q_.block(3, 3, 3, 3) = (dt3 / 3.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Velocity-acceleration covariance terms
        Q_.block(3, 6, 3, 3) = (dt * dt / 2.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(6, 3, 3, 3) = (dt * dt / 2.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Acceleration variance terms
        Q_.block(6, 6, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);
        
        // Scale by jerk variance
        Q_ = sigma_j_ * sigma_j_ * Q_;
        
        return Q_;
    }
    
    bool Q(double dt, double sigma_j) {
        if (dt <= 0) {
            printf("ERROR [ConstAccel::Q] dt <= 0\n");
            return false;
        }
        if (sigma_j <= 0) {
            printf("ERROR [ConstAccel::Q] sigma_j <= 0\n");
            return false;
        }
        
        sigma_j_ = sigma_j;
        return (Q(dt).rows() > 0);
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
            printf("ERROR [ConstAccel::Q] Input vector size != NUM_STATES. %lu != %u\n", v.size(), NUM_STATES);
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
        if (debug_) printf("[ConstAccel::R] Setting R_ from a matrix\n");
        
        if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS) {
            R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
            R_ = M;
            return true;
        }
        return false;
    }
    
    bool R(std::vector<double> v) override {
        if (debug_) printf("[ConstAccel::R] Setting diagonal R_ from a vector\n");
        
        if (v.size() != NUM_MEASUREMENTS) {
            printf("ERROR [ConstAccel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u\n", v.size(), NUM_MEASUREMENTS);
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
    
    bool P(double sigma_p, double sigma_v, double sigma_a) {
        if (sigma_p <= 0 || sigma_v <= 0 || sigma_a <= 0) {
            printf("ERROR [ConstAccel::P] sigma_p, sigma_v, or sigma_a <= 0\n");
            return false;
        }
        
        if (debug()) {
            printf("[ConstantAccelModel::P] Setting P from standard deviations\n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0, 0, 3, 3) = sigma_p * sigma_p * Eigen::MatrixXd::Identity(3, 3);
        P_.block(3, 3, 3, 3) = sigma_v * sigma_v * Eigen::MatrixXd::Identity(3, 3);
        P_.block(6, 6, 3, 3) = sigma_a * sigma_a * Eigen::MatrixXd::Identity(3, 3);
        
        if (debug()) std::cout << "P: \n" << P_ << "\n";
        
        return true;
    }
    
    // For compatibility with the base class signature
    bool P(Eigen::MatrixXd M) override {
        if (debug_) {
            printf("[ConstAccel::P] Setting P from a matrix\n");
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
        if (debug_) printf("[ConstAccel::predictX] Predicting x\n");
        
        if (dt <= 0) {
            printf("WARN [ConstAccel::predictX] dt = %f <= 0. Returning same state\n", dt);
            return s;
        }
        
        if (debug_) printf("[ConstAccel::predictX] det(P) of current state: %f\n", s.P.determinant());
        
        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;
        
        auto FF = F(dt);
        xx.P = FF * s.P * FF.transpose() + Q(dt);
        xx.x = f(s.x, dt);
        
        if (debug_) {
            printf("[ConstAccel::predictX] ||x_new - x_old|| = %f\n", (xx.x - s.x).norm());
            printf("[ConstAccel::predictX] det(P) of predicted state: %f\n", xx.P.determinant());
            std::cout << "[predictX] old P = \n" << s.P << "\n";
            std::cout << "[predictX] old x = \n" << s.x << "\n";
            std::cout << "[predictX] new P = \n" << xx.P << "\n";
            std::cout << "[predictX] new x = \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    kf_state updateX(sensor_measurement z, kf_state s) override {
        if (debug_) printf("[ConstAccel::updateX] Updating x\n");
        
        kf_state xx;
        xx.time_stamp = z.time_stamp;
        
        Eigen::VectorXd y = z.z - h(s.x); // innovation
        Eigen::MatrixXd S = H() * s.P * H().transpose() + R_; // innovation covariance
        Eigen::MatrixXd K = s.P * H().transpose() * S.inverse(); // optimal Kalman gain
        xx.x = s.x + K * y;
        xx.P = (Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) - K * H()) * s.P;
        
        if (debug_) {
            printf("[ConstAccel::updateX] Done updating state\n");
            printf("[ConstAccel::updateX] Norm of innovation y = %f\n", y.norm());
            std::cout << "[ConstAccel::updateX] predicted state P: \n" << s.P << "\n";
            std::cout << "[ConstAccel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstAccel::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstAccel::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstAccel::updateX] corrected state: \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    kf_state updateX(sensor_measurement z, kf_state s, double dt) {
        if (debug_) printf("[ConstAccel::updateX] Updating x with dt scaling\n");
        
        kf_state xx;
        xx.time_stamp = z.time_stamp;
        
        Eigen::VectorXd y = z.z - h(s.x); // innovation
        Eigen::MatrixXd S = H() * s.P * H().transpose() + dt * dt * R_; // innovation covariance scaled by dt^2
        Eigen::MatrixXd K = s.P * H().transpose() * S.inverse(); // optimal Kalman gain
        xx.x = s.x + K * y;
        xx.P = (Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) - K * H()) * s.P;
        
        if (debug_) {
            printf("[ConstAccel::updateX] Done updating state\n");
            printf("[ConstAccel::updateX] Norm of innovation y = %f\n", y.norm());
            std::cout << "[ConstAccel::updateX] predicted state P: \n" << s.P << "\n";
            std::cout << "[ConstAccel::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstAccel::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstAccel::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstAccel::updateX] corrected state: \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    double logLikelihood(kf_state xx, sensor_measurement z) override {
        Eigen::VectorXd y_hat = z.z - h(xx.x); // Innovation
        
        Eigen::MatrixXd S = R_ + H() * xx.P * H().transpose(); // Innovation covariance
        
        double det_S = S.determinant();
        if (det_S <= 0) {
            printf("ERROR [ConstantAccelModel::logLikelihood] Non-positive definite S matrix. det(S) = %f\n", det_S);
            return -std::numeric_limits<double>::infinity();
        }
        
        double tmp = y_hat.transpose() * S.inverse() * y_hat;
        
        double LL = -0.5 * (tmp + log(det_S) + NUM_MEASUREMENTS * log(2 * M_PI)); // Log-likelihood
        
        if (debug_) printf("[ConstantAccelModel::logLikelihood] Done computing logLikelihood. L= %f\n", LL);
        
        return LL;
    }
    
    kf_state initStateFromMeasurements(sensor_measurement z) override {
        if (debug_) printf("[ConstAccel::initStateFromMeasurements] Initializing state from measurements\n");
        
        kf_state state;
        state.time_stamp = z.time_stamp;
        state.x = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        
        // Set position from measurement
        state.x(0) = z.z(0);
        state.x(1) = z.z(1);
        state.x(2) = z.z(2);
        
        // Initialize velocity and acceleration to zero
        // Positions 3-8 remain at zero
        
        // Set initial covariance
        if (sigma_p_ > 0 && sigma_v_ > 0 && sigma_a_ > 0) {
            P(sigma_p_, sigma_v_, sigma_a_);
        }
        
        state.P = P_;
        
        return state;
    }
    
    double computeDistance(kf_state xx, sensor_measurement zz) override {
        auto y = zz.z - h(xx.x);
        auto nrm = y.norm();
        if (debug_) printf("[ConstAccel::computeDistance] Distance between state and measurement = %f\n", nrm);
        return nrm;
    }
};

#endif // CONSTANT_ACCEL_H