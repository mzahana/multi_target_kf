// include/multi_target_kf/motion_model.h (updated)
#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include "multi_target_kf/structs.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Abstract base class for all motion models
 */
class MotionModel {
protected:
    Eigen::MatrixXd F_; /* State transition matrix */
    Eigen::MatrixXd H_; /* Observation matrix */
    Eigen::MatrixXd Q_; /* Process covariance matrix */
    Eigen::MatrixXd P_; /* State covariance estimate */
    Eigen::MatrixXd R_; /* Measurements covariance matrix */
    Eigen::VectorXd x_; /* Current state vector */
    unsigned int NUM_STATES;    /* State dimension */
    unsigned int NUM_MEASUREMENTS; /* Measurements dimension */
    double dt_;           /* Prediction sampling time */
    double current_t_;    /* Current time stamp */
    bool debug_;          /* Debug flag */

public:
    // Fix initialization order to match the declaration order
    MotionModel() : NUM_STATES(0), NUM_MEASUREMENTS(0), dt_(0.01), current_t_(0.0), debug_(false) {}
    virtual ~MotionModel() {}

    virtual bool init() = 0;
    
    // Getters and setters
    virtual unsigned int numStates() const { return NUM_STATES; }
    virtual unsigned int numMeasurements() const { return NUM_MEASUREMENTS; }
    virtual bool debug() const { return debug_; }
    virtual void debug(bool d) { debug_ = d; }
    virtual bool setDt(double dt) {
        if (dt < 0) return false;
        dt_ = dt;
        return true;
    }
    virtual void setCurrentTimeStamp(double t) { current_t_ = t; }
    
    // Core KF methods to be implemented by derived classes
    virtual Eigen::VectorXd f(Eigen::VectorXd x, double dt) = 0; // State prediction
    virtual Eigen::VectorXd h(Eigen::VectorXd x) = 0; // Measurement model
    virtual Eigen::MatrixXd F(double dt) = 0; // State transition matrix
    virtual Eigen::MatrixXd H() = 0; // Measurement matrix
    
    // These were missing in the base class but were marked override in the derived classes
    virtual bool F(Eigen::MatrixXd M) = 0; // Set F from matrix
    virtual bool H(Eigen::MatrixXd M) = 0; // Set H from matrix
    
    virtual Eigen::MatrixXd Q(double dt) = 0; // Process noise covariance
    virtual bool Q(Eigen::MatrixXd M) = 0; // Set Q from matrix
    virtual bool Q(std::vector<double> v) = 0; // Set Q from vector
    virtual Eigen::MatrixXd R() = 0; // Measurement noise covariance
    virtual bool R(Eigen::MatrixXd M) = 0; // Set R from matrix
    virtual bool R(std::vector<double> v) = 0; // Set R from vector
    virtual Eigen::MatrixXd P() = 0; // State covariance
    virtual bool P(Eigen::MatrixXd M) = 0; // Set P from matrix
    
    // KF operations
    virtual kf_state predictX(kf_state s, double dt) = 0;
    virtual kf_state updateX(sensor_measurement z, kf_state s) = 0;
    virtual double logLikelihood(kf_state xx, sensor_measurement z) = 0;
    virtual kf_state initStateFromMeasurements(sensor_measurement z) = 0;
    virtual double computeDistance(kf_state xx, sensor_measurement zz) = 0;
};

#endif // MOTION_MODEL_H