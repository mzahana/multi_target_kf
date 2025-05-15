// include/multi_target_kf/adaptive_accel_ukf.h
#ifndef ADAPTIVE_ACCEL_UKF_H
#define ADAPTIVE_ACCEL_UKF_H

#include "multi_target_kf/motion_model.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <deque>

/**
 * @brief Nearly-Constant Acceleration Model with Adaptive Process Noise using UKF
 * State vector: [px, py, pz, vx, vy, vz, ax, ay, az]
 * Measurement vector: [px, py, pz]
 */
class AdaptiveAccelUKF : public MotionModel {
private:
    // UKF specific parameters
    double alpha_;        /* Spread of sigma points around mean */
    double beta_;         /* Prior knowledge about distribution (2.0 is optimal for Gaussian) */
    double kappa_;        /* Secondary scaling parameter (typically 0 or 3-n) */
    double lambda_;       /* Combined scaling parameter */
    unsigned int L_;      /* Number of state dimensions */
    
    // Weights for mean and covariance computation
    std::vector<double> Wm_;  /* Weights for mean */
    std::vector<double> Wc_;  /* Weights for covariance */
    
    // UKF sigma points
    Eigen::MatrixXd sigma_points_;  /* Matrix to store sigma points */
    
    // Adaptive process noise parameters
    double jerk_std_;           /* Standard deviation of jerk */
    double jerk_adaptive_max_;  /* Maximum adaptive jerk value */
    double adaptive_threshold_; /* Threshold for maneuver detection */
    double adaptive_decay_;     /* Decay factor for adaptive noise */
    double current_jerk_std_;   /* Current adaptive jerk standard deviation */
    
    // Innovation sequence history for maneuver detection
    std::deque<Eigen::VectorXd> innovation_history_;
    unsigned int innovation_window_size_;
    
    // State parameters
    double sigma_p_, sigma_v_, sigma_a_; /* Initial state uncertainties */

public:
    /* Constructor */
    AdaptiveAccelUKF() : 
        alpha_(1.0), 
        beta_(2.0), 
        kappa_(0.0), 
        jerk_std_(5.0),
        jerk_adaptive_max_(20.0),
        adaptive_threshold_(2.5),
        adaptive_decay_(0.95),
        current_jerk_std_(5.0),
        innovation_window_size_(10),
        sigma_p_(0.5),
        sigma_v_(2.0),
        sigma_a_(1.0) 
    {
        NUM_STATES = 9;        // [px, py, pz, vx, vy, vz, ax, ay, az]
        NUM_MEASUREMENTS = 3;  // [px, py, pz]
        L_ = NUM_STATES;
        lambda_ = alpha_ * alpha_ * (L_ + kappa_) - L_;
        
        init();
        initWeights();
    }
    
    ~AdaptiveAccelUKF() override {}

    bool init() override {
        F_.resize(NUM_STATES, NUM_STATES);
        F_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_STATES);
        // Set observation matrix - we only observe position
        H_(0, 0) = 1.0; // observing x
        H_(1, 1) = 1.0; // observing y
        H_(2, 2) = 1.0; // observing z
        
        Q_.resize(NUM_STATES, NUM_STATES);
        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);
        
        P_.resize(NUM_STATES, NUM_STATES);
        P_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);
        
        R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        R_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        
        x_.resize(NUM_STATES, 1);
        x_ = Eigen::MatrixXd::Zero(NUM_STATES, 1);
        
        // Initialize sigma points matrix
        sigma_points_.resize(NUM_STATES, 2 * NUM_STATES + 1);
        sigma_points_ = Eigen::MatrixXd::Zero(NUM_STATES, 2 * NUM_STATES + 1);
        
        debug_ = false;
        dt_ = 0.01;
        current_t_ = 0.0;

        return true;
    }
    
    void initWeights() {
        Wm_.resize(2 * L_ + 1);
        Wc_.resize(2 * L_ + 1);
        
        // Set weights for mean and covariance
        Wm_[0] = lambda_ / (L_ + lambda_);
        Wc_[0] = Wm_[0] + (1 - alpha_*alpha_ + beta_);
        
        for (unsigned int i = 1; i < 2 * L_ + 1; i++) {
            Wm_[i] = 1.0 / (2 * (L_ + lambda_));
            Wc_[i] = Wm_[i];
        }
    }
    
    // Model-specific setters
    bool setAlpha(double alpha) {
        if (alpha <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setAlpha] alpha <= 0\n");
            return false;
        }
        alpha_ = alpha;
        lambda_ = alpha_ * alpha_ * (L_ + kappa_) - L_;
        initWeights();
        if (debug_) std::cout << "alpha: " << alpha_ << "\n";
        return true;
    }
    
    bool setBeta(double beta) {
        if (beta < 0) {
            printf("ERROR [AdaptiveAccelUKF::setBeta] beta < 0\n");
            return false;
        }
        beta_ = beta;
        initWeights();
        if (debug_) std::cout << "beta: " << beta_ << "\n";
        return true;
    }
    
    bool setKappa(double kappa) {
        kappa_ = kappa;
        lambda_ = alpha_ * alpha_ * (L_ + kappa_) - L_;
        initWeights();
        if (debug_) std::cout << "kappa: " << kappa_ << "\n";
        return true;
    }
    
    bool setJerkStd(double jerk_std) {
        if (jerk_std <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setJerkStd] jerk_std <= 0\n");
            return false;
        }
        jerk_std_ = jerk_std;
        current_jerk_std_ = jerk_std;
        if (debug_) std::cout << "jerk_std: " << jerk_std_ << "\n";
        return true;
    }
    
    bool setJerkAdaptiveMax(double jerk_adaptive_max) {
        if (jerk_adaptive_max <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setJerkAdaptiveMax] jerk_adaptive_max <= 0\n");
            return false;
        }
        jerk_adaptive_max_ = jerk_adaptive_max;
        if (debug_) std::cout << "jerk_adaptive_max: " << jerk_adaptive_max_ << "\n";
        return true;
    }
    
    bool setAdaptiveThreshold(double adaptive_threshold) {
        if (adaptive_threshold <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setAdaptiveThreshold] adaptive_threshold <= 0\n");
            return false;
        }
        adaptive_threshold_ = adaptive_threshold;
        if (debug_) std::cout << "adaptive_threshold: " << adaptive_threshold_ << "\n";
        return true;
    }
    
    bool setAdaptiveDecay(double adaptive_decay) {
        if (adaptive_decay <= 0 || adaptive_decay >= 1) {
            printf("ERROR [AdaptiveAccelUKF::setAdaptiveDecay] adaptive_decay must be in (0,1)\n");
            return false;
        }
        adaptive_decay_ = adaptive_decay;
        if (debug_) std::cout << "adaptive_decay: " << adaptive_decay_ << "\n";
        return true;
    }
    
    bool setInnovationWindowSize(unsigned int size) {
        if (size <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setInnovationWindowSize] size <= 0\n");
            return false;
        }
        innovation_window_size_ = size;
        if (debug_) std::cout << "innovation_window_size: " << innovation_window_size_ << "\n";
        return true;
    }
    
    bool setSigmaP(double sigma_p) {
        if (sigma_p <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setSigmaP] sigma_p <= 0\n");
            return false;
        }
        sigma_p_ = sigma_p;
        if (debug_) std::cout << "sigma_p: " << sigma_p_ << "\n";
        return true;
    }
    
    bool setSigmaV(double sigma_v) {
        if (sigma_v <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setSigmaV] sigma_v <= 0\n");
            return false;
        }
        sigma_v_ = sigma_v;
        if (debug_) std::cout << "sigma_v: " << sigma_v_ << "\n";
        return true;
    }
    
    bool setSigmaA(double sigma_a) {
        if (sigma_a <= 0) {
            printf("ERROR [AdaptiveAccelUKF::setSigmaA] sigma_a <= 0\n");
            return false;
        }
        sigma_a_ = sigma_a;
        if (debug_) std::cout << "sigma_a: " << sigma_a_ << "\n";
        return true;
    }
    
    // UKF methods
    void generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
        // Calculate matrix square root using Cholesky decomposition
        Eigen::MatrixXd L = P.llt().matrixL();
        
        // Set first sigma point at the mean
        sigma_points_.col(0) = x;
        
        // Set remaining sigma points
        for (unsigned int i = 0; i < L_; i++) {
            sigma_points_.col(i + 1) = x + std::sqrt(L_ + lambda_) * L.col(i);
            sigma_points_.col(i + 1 + L_) = x - std::sqrt(L_ + lambda_) * L.col(i);
        }
    }
    
    Eigen::VectorXd predictState(const Eigen::MatrixXd& sigma_points, double dt) {
        Eigen::MatrixXd X_pred = Eigen::MatrixXd::Zero(L_, 2 * L_ + 1);
        
        // Propagate each sigma point through the motion model
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            X_pred.col(i) = f(sigma_points.col(i), dt);
        }
        
        // Calculate predicted mean state
        Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(L_);
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            x_pred += Wm_[i] * X_pred.col(i);
        }
        
        // Store propagated sigma points for later use
        sigma_points_ = X_pred;
        
        return x_pred;
    }
    
    Eigen::MatrixXd predictCovariance(const Eigen::VectorXd& x_pred, const Eigen::MatrixXd& Q) {
        Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(L_, L_);
        
        // Calculate predicted covariance
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            Eigen::VectorXd diff = sigma_points_.col(i) - x_pred;
            P_pred += Wc_[i] * diff * diff.transpose();
        }
        
        // Add process noise
        P_pred += Q;
        
        return P_pred;
    }
    
    void predictMeasurement(const Eigen::VectorXd& x_pred, const Eigen::MatrixXd& P_pred,
                          Eigen::VectorXd& z_pred, Eigen::MatrixXd& S, Eigen::MatrixXd& Pxz) {
        // Transform sigma points to measurement space
        Eigen::MatrixXd Z_pred = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, 2 * L_ + 1);
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            Z_pred.col(i) = h(sigma_points_.col(i));
        }
        
        // Calculate predicted measurement mean
        z_pred = Eigen::VectorXd::Zero(NUM_MEASUREMENTS);
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            z_pred += Wm_[i] * Z_pred.col(i);
        }
        
        // Calculate innovation covariance
        S = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
        Pxz = Eigen::MatrixXd::Zero(L_, NUM_MEASUREMENTS);
        
        for (unsigned int i = 0; i < 2 * L_ + 1; i++) {
            Eigen::VectorXd z_diff = Z_pred.col(i) - z_pred;
            Eigen::VectorXd x_diff = sigma_points_.col(i) - x_pred;
            
            S += Wc_[i] * z_diff * z_diff.transpose();
            Pxz += Wc_[i] * x_diff * z_diff.transpose();
        }
        
        // Add measurement noise
        S += R_;
    }
    
    void updateAdaptiveProcessNoise(const Eigen::VectorXd& innovation) {
        // Add innovation to history
        innovation_history_.push_back(innovation);
        if (innovation_history_.size() > innovation_window_size_) {
            innovation_history_.pop_front();
        }
        
        // Not enough data to detect maneuvers yet
        if (innovation_history_.size() < 3) {
            return;
        }
        
        // Compute average innovation magnitude
        double avg_innov_mag = 0.0;
        for (const auto& innov : innovation_history_) {
            avg_innov_mag += innov.norm();
        }
        avg_innov_mag /= innovation_history_.size();
        
        // Check if maneuver detected (high innovation indicates model mismatch)
        if (avg_innov_mag > adaptive_threshold_) {
            // Increase process noise for jerk
            current_jerk_std_ = std::min(current_jerk_std_ * (1.0 + avg_innov_mag/adaptive_threshold_), 
                                         jerk_adaptive_max_);
            
            if (debug_) {
                printf("[AdaptiveAccelUKF::updateAdaptiveProcessNoise] Maneuver detected! Jerk std increased to %f\n", 
                       current_jerk_std_);
            }
        } else {
            // Gradually decrease process noise back to nominal
            current_jerk_std_ = jerk_std_ + adaptive_decay_ * (current_jerk_std_ - jerk_std_);
        }
    }
    
    // Implementation of base class virtual methods
    Eigen::VectorXd f(Eigen::VectorXd x, double dt) override {
        if (debug_) printf("[AdaptiveAccelUKF::f] Calculating f\n");
        
        if (dt <= 0) {
            printf("[AdaptiveAccelUKF::f] dt is <= 0. Returning same x\n");
            return x;
        }
        
        Eigen::VectorXd x_next = Eigen::VectorXd::Zero(NUM_STATES);
        
        // Extract components for readability
        Eigen::Vector3d pos = x.segment(0, 3);
        Eigen::Vector3d vel = x.segment(3, 3);
        Eigen::Vector3d acc = x.segment(6, 3);
        
        // Update position: p = p + v*dt + 0.5*a*dt^2
        x_next.segment(0, 3) = pos + vel * dt + 0.5 * acc * dt * dt;
        
        // Update velocity: v = v + a*dt
        x_next.segment(3, 3) = vel + acc * dt;
        
        // Acceleration remains the same in nearly-constant acceleration model
        x_next.segment(6, 3) = acc;
        
        return x_next;
    }
    
    // Convenience methods using stored dt_
    Eigen::VectorXd f() { return f(x_, dt_); }
    Eigen::VectorXd f(Eigen::VectorXd x) { return f(x, dt_); }
    
    Eigen::MatrixXd F(double dt) override {
        if (debug_) printf("[AdaptiveAccelUKF::F] Calculating F\n");
        
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
        if (debug_) printf("[AdaptiveAccelUKF::h] Calculating h\n");
        
        // For simplicity, our measurement model is linear
        // We only measure the position [px, py, pz]
        Eigen::VectorXd z = Eigen::VectorXd::Zero(NUM_MEASUREMENTS);
        z(0) = x(0); // x position
        z(1) = x(1); // y position
        z(2) = x(2); // z position
        
        if (debug_) std::cout << "h(x) = \n" << z << "\n";
        return z;
    }
    
    Eigen::MatrixXd H() override {
        if (debug_) printf("[AdaptiveAccelUKF::H] Returning H_\n");
        return H_;
    }
    
    bool H(Eigen::MatrixXd M) override {
        if (debug_) printf("[AdaptiveAccelUKF::H] Setting H_\n");
        
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
        Q_ = Eigen::MatrixXd::Zero(NUM_STATES, NUM_STATES);
        
        // Discretization of continuous process noise
        // For the nearly-constant acceleration model with jerk as process noise
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;
        
        // Position covariance (due to jerk)
        Q_.block(0, 0, 3, 3) = (dt5 / 20.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Position-velocity cross-covariance
        Q_.block(0, 3, 3, 3) = (dt4 / 8.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(3, 0, 3, 3) = (dt4 / 8.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Position-acceleration cross-covariance
        Q_.block(0, 6, 3, 3) = (dt3 / 6.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(6, 0, 3, 3) = (dt3 / 6.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Velocity covariance
        Q_.block(3, 3, 3, 3) = (dt3 / 3.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Velocity-acceleration cross-covariance
        Q_.block(3, 6, 3, 3) = (dt2 / 2.0) * Eigen::MatrixXd::Identity(3, 3);
        Q_.block(6, 3, 3, 3) = (dt2 / 2.0) * Eigen::MatrixXd::Identity(3, 3);
        
        // Acceleration covariance
        Q_.block(6, 6, 3, 3) = dt * Eigen::MatrixXd::Identity(3, 3);
        
        // Scale by current jerk variance (which might be adaptive)
        Q_ = current_jerk_std_ * current_jerk_std_ * Q_;
        
        return Q_;
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
            printf("ERROR [AdaptiveAccelUKF::Q] Input vector size != NUM_STATES. %lu != %u\n", v.size(), NUM_STATES);
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
        if (debug_) printf("[AdaptiveAccelUKF::R] Setting R_ from a matrix\n");
        
        if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS) {
            R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
            R_ = M;
            return true;
        }
        return false;
    }
    
    bool R(std::vector<double> v) override {
        if (debug_) printf("[AdaptiveAccelUKF::R] Setting diagonal R_ from a vector\n");
        
        if (v.size() != NUM_MEASUREMENTS) {
            printf("ERROR [AdaptiveAccelUKF::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u\n", v.size(), NUM_MEASUREMENTS);
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
            printf("ERROR [AdaptiveAccelUKF::P] sigma_p, sigma_v, or sigma_a <= 0\n");
            return false;
        }
        
        if (debug()) {
            printf("[AdaptiveAccelUKF::P] Setting P from standard deviations\n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0, 0, 3, 3) = sigma_p * sigma_p * Eigen::MatrixXd::Identity(3, 3);
        P_.block(3, 3, 3, 3) = sigma_v * sigma_v * Eigen::MatrixXd::Identity(3, 3);
        P_.block(6, 6, 3, 3) = sigma_a * sigma_a * Eigen::MatrixXd::Identity(3, 3);
        
        if (debug()) std::cout << "P: \n" << P_ << "\n";
        
        return true;
    }
    
    bool P(Eigen::MatrixXd M) override {
        if (debug_) {
            printf("[AdaptiveAccelUKF::P] Setting P from a matrix\n");
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
        if (debug_) printf("[AdaptiveAccelUKF::predictX] Predicting x\n");
        
        if (dt <= 0) {
            printf("WARN [AdaptiveAccelUKF::predictX] dt = %f <= 0. Returning same state\n", dt);
            return s;
        }
        
        if (debug_) printf("[AdaptiveAccelUKF::predictX] det(P) of current state: %f\n", s.P.determinant());
        
        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;
        
        // Generate sigma points
        generateSigmaPoints(s.x, s.P);
        
        // Predict state
        xx.x = predictState(sigma_points_, dt);
        
        // Compute adaptive process noise
        Eigen::MatrixXd Q_k = Q(dt);
        
        // Predict covariance
        xx.P = predictCovariance(xx.x, Q_k);
        
        if (debug_) {
            printf("[AdaptiveAccelUKF::predictX] ||x_new - x_old|| = %f\n", (xx.x - s.x).norm());
            printf("[AdaptiveAccelUKF::predictX] det(P) of predicted state: %f\n", xx.P.determinant());
            std::cout << "[predictX] old P = \n" << s.P << "\n";
            std::cout << "[predictX] old x = \n" << s.x << "\n";
            std::cout << "[predictX] new P = \n" << xx.P << "\n";
            std::cout << "[predictX] new x = \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    kf_state updateX(sensor_measurement z, kf_state s) override {
        if (debug_) printf("[AdaptiveAccelUKF::updateX] Updating x\n");
        
        kf_state xx;
        xx.time_stamp = z.time_stamp;
        
        // Predict measurement
        Eigen::VectorXd z_pred;
        Eigen::MatrixXd S, Pxz;
        predictMeasurement(s.x, s.P, z_pred, S, Pxz);
        
        // Compute Kalman gain
        Eigen::MatrixXd K = Pxz * S.inverse();
        
        // Compute innovation
        Eigen::VectorXd innovation = z.z - z_pred;
        
        // Update state and covariance
        xx.x = s.x + K * innovation;
        xx.P = s.P - K * S * K.transpose();
        
        // Update adaptive process noise based on innovation
        updateAdaptiveProcessNoise(innovation);
        
        if (debug_) {
            printf("[AdaptiveAccelUKF::updateX] Done updating state\n");
            printf("[AdaptiveAccelUKF::updateX] Norm of innovation = %f\n", innovation.norm());
            printf("[AdaptiveAccelUKF::updateX] Current jerk_std = %f\n", current_jerk_std_);
            std::cout << "[AdaptiveAccelUKF::updateX] predicted state P: \n" << s.P << "\n";
            std::cout << "[AdaptiveAccelUKF::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[AdaptiveAccelUKF::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[AdaptiveAccelUKF::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[AdaptiveAccelUKF::updateX] corrected state: \n" << xx.x << "\n";
        }
        
        return xx;
    }
    
    double logLikelihood(kf_state xx, sensor_measurement z) override {
        // Predict measurement
        Eigen::VectorXd z_pred;
        Eigen::MatrixXd S, Pxz;
        
        // First generate sigma points (needed for measurement prediction)
        generateSigmaPoints(xx.x, xx.P);
        
        // Then predict measurement and innovation covariance
        predictMeasurement(xx.x, xx.P, z_pred, S, Pxz);
        
        // Compute innovation
        Eigen::VectorXd y = z.z - z_pred;
        
        // Compute log-likelihood
        double det_S = S.determinant();
        if (det_S <= 0) {
            printf("ERROR [AdaptiveAccelUKF::logLikelihood] Non-positive definite S matrix. det(S) = %f\n", det_S);
            return -std::numeric_limits<double>::infinity();
        }
        
        double tmp = y.transpose() * S.inverse() * y;
        double LL = -0.5 * (tmp + log(det_S) + NUM_MEASUREMENTS * log(2 * M_PI));
        
        if (debug_) printf("[AdaptiveAccelUKF::logLikelihood] Done computing logLikelihood. L= %f\n", LL);
        
        return LL;
    }
    
    kf_state initStateFromMeasurements(sensor_measurement z) override {
        if (debug_) printf("[AdaptiveAccelUKF::initStateFromMeasurements] Initializing state from measurements\n");
        
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
        // This is simpler than log-likelihood and just computes Euclidean distance
        // between measurement and state projection
        Eigen::VectorXd z_pred = h(xx.x);
        auto y = zz.z - z_pred;
        auto nrm = y.norm();
        
        if (debug_) printf("[AdaptiveAccelUKF::computeDistance] Distance between state and measurement = %f\n", nrm);
        
        return nrm;
    }
};

#endif // ADAPTIVE_ACCEL_UKF_H