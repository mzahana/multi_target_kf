// include/multi_target_kf/tracker_config.h
#ifndef TRACKER_CONFIG_H
#define TRACKER_CONFIG_H

#include <vector>
#include <string>
#include "multi_target_kf/model_factory.h"

/**
 * @brief Configuration structure for the Kalman filter tracker
 * Centralizes all configuration parameters in one place
 */
struct TrackerConfig {
    ModelType model_type;             // Type of motion model to use
    double dt_pred;                   // Prediction time step (in seconds)
    double V_max;                     // Maximum uncertainty before rejecting a track [m^3]
    double V_certain;                 // Maximum uncertainty for a track to be considered certain [m^3]
    int N_meas;                       // Minimum number of measurements to confirm a track
    double l_threshold;               // Measurement association log-likelihood threshold
    double dist_threshold;            // Maximum distance between a state & measurement to consider them as a match
    std::string tracking_frame;       // Coordinate frame for tracking
    bool do_update_step;              // Whether to perform the KF update step
    double measurement_off_time;      // Maximum time (seconds) with no measurement before filter is stopped
    bool use_track_id;                // Consider track ID in measurement-state association
    double track_measurement_timeout; // Maximum time (seconds) from last measurement before considering a track uncertain
    unsigned int state_buffer_size;   // Length of state buffer
    
    // Model parameters
    std::vector<double> q_diag;       // Diagonal elements of Q matrix
    std::vector<double> r_diag;       // Diagonal elements of R matrix
    double sigma_a;                   // Standard deviation of acceleration noise
    double sigma_p;                   // Standard deviation of position
    double sigma_v;                   // Standard deviation of velocity
    double sigma_j;                   // Standard deviation of jerk noise (for constant acceleration) - NEW
    
    bool debug;                       // Enable debug output
    
    /**
     * @brief Constructor with default values
     */
    TrackerConfig() :
        model_type(CONSTANT_VELOCITY),
        dt_pred(0.05),
        V_max(20.0),
        V_certain(1.0),
        N_meas(20),
        l_threshold(-2.0),
        dist_threshold(2.0),
        tracking_frame("map"),
        do_update_step(true),
        measurement_off_time(2.0),
        use_track_id(false),
        track_measurement_timeout(3.0),
        state_buffer_size(40),
        q_diag{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        r_diag{0.01, 0.01, 0.01},
        sigma_a(10.0),
        sigma_p(1.0),  // Make sure this is before sigma_j
        sigma_v(1.0),
        sigma_j(5.0),  // Move sigma_j after sigma_v to match declaration order
        debug(false)
    {}
    
    /**
     * @brief Print the configuration
     */
    void print() const {
        printf("Tracker Configuration:\n");
        printf("- Model type: %s\n", ModelFactory::getModelName(model_type));
        printf("- Prediction time step (dt_pred): %.3f seconds\n", dt_pred);
        printf("- Maximum uncertainty (V_max): %.3f\n", V_max);
        printf("- Certain track uncertainty (V_certain): %.3f\n", V_certain);
        printf("- Minimum measurements (N_meas): %d\n", N_meas);
        printf("- Log-likelihood threshold: %.3f\n", l_threshold);
        printf("- Distance threshold: %.3f\n", dist_threshold);
        printf("- Tracking frame: %s\n", tracking_frame.c_str());
        printf("- Update step enabled: %s\n", do_update_step ? "Yes" : "No");
        printf("- Measurement timeout: %.3f seconds\n", measurement_off_time);
        printf("- Use track ID: %s\n", use_track_id ? "Yes" : "No");
        printf("- Track measurement timeout: %.3f seconds\n", track_measurement_timeout);
        printf("- State buffer size: %u\n", state_buffer_size);
        printf("- sigma_a: %.3f\n", sigma_a);
        printf("- sigma_p: %.3f\n", sigma_p);
        printf("- sigma_v: %.3f\n", sigma_v);
        printf("- Debug mode: %s\n", debug ? "On" : "Off");
    }
};

#endif // TRACKER_CONFIG_H