# Multi-Target Kalman Filter Documentation

## Overview

The `multi_target_kf` package is a ROS 2 implementation of a linear Kalman filter for multi-target state estimation. The package supports multiple motion models and provides a framework for tracking multiple targets simultaneously. It uses the Hungarian algorithm for measurement-to-track association and implements a modular design that allows for easy extension with new motion models.

## Table of Contents

- [Installation](#installation)
- [Package Structure](#package-structure)
- [Motion Models](#motion-models)
  - [Available Models](#available-models)
  - [Model Parameters](#model-parameters)
- [Usage](#usage)
  - [Configuration](#configuration)
  - [Launch Files](#launch-files)
  - [ROS Topics](#ros-topics)
- [Extending the Package](#extending-the-package)
  - [Creating a New Motion Model](#creating-a-new-motion-model)
  - [Registering the New Model](#registering-the-new-model)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Testing](#testing)

## Installation

### Dependencies

- Eigen3
- ROS 2 (humble or later)
- Standard ROS 2 packages: `std_msgs`, `geometry_msgs`, `sensor_msgs`

### Building from Source

```bash
# Create a workspace (if you don't have one already)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/mzahana/multi_target_kf.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select multi_target_kf

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Package Structure

The package follows a modular design pattern centered around the following key components:

- **Motion Models**: Implement different kinematic behaviors (e.g., constant velocity, constant acceleration)
- **KF Tracker**: Core implementation of the Kalman filter with multi-target tracking capabilities
- **ROS Interface**: Bridges between ROS and the KF tracker implementation
- **Hungarian Algorithm**: Used for optimal assignment between tracks and measurements

The directory structure is as follows:

```
multi_target_kf/
├── include/multi_target_kf/        # C++ header files
│   ├── constant_accel.h            # Constant acceleration model
│   ├── constant_vel.h              # Constant velocity model
│   ├── hungarian.h                 # Hungarian algorithm for assignment
│   ├── kf_tracker.h                # Core Kalman filter tracker
│   ├── model_factory.h             # Factory for creating motion models
│   ├── models.h                    # Collection of all models
│   ├── motion_model.h              # Base class for motion models
│   ├── structs.h                   # Data structure definitions
│   ├── tracker_config.h            # Configuration for the tracker
│   └── tracker_ros.h               # ROS interface
├── src/                            # C++ implementation files
│   ├── hungarian.cpp               # Hungarian algorithm implementation
│   ├── kf_tracker.cpp              # Tracker implementation
│   ├── tracker_node.cpp            # ROS node entry point
│   └── tracker_ros.cpp             # ROS interface implementation
├── config/                         # Configuration files
│   ├── kf_param.yaml               # General parameters
│   ├── kf_param_accel.yaml         # Constant acceleration parameters
│   ├── kf_param_vel.yaml           # Constant velocity parameters
│   └── tracker_visualization.rviz   # RViz configuration
├── launch/                         # Launch files
│   ├── kf_const_accel.launch.py    # Launch with constant acceleration model
│   ├── kf_const_vel.launch.py      # Launch with constant velocity model
│   ├── kf_rviz.launch.py           # Launch with RViz visualization
│   └── modular_kf.launch.py        # Modular launch with configurable parameters
├── msg/                            # Custom ROS messages
│   ├── KFTrack.msg                 # Single track message
│   └── KFTracks.msg                # Multiple tracks message
├── scripts/                        # Python scripts
│   └── simulate_circle.py          # Simulation script for testing
└── test/                           # Unit tests
    └── test_motion_models.cpp      # Tests for motion models
```

## Motion Models

### Available Models

The package currently provides two motion models:

1. **Constant Velocity Model (CONSTANT_VELOCITY)**
   - State vector: `[px, py, pz, vx, vy, vz]`
   - Assumes targets move with constant velocity
   - Process noise is modeled as random accelerations

2. **Constant Acceleration Model (CONSTANT_ACCELERATION)**
   - State vector: `[px, py, pz, vx, vy, vz, ax, ay, az]`
   - Assumes targets move with constant acceleration
   - Process noise is modeled as random jerks

Both models use a measurement vector `[px, py, pz]` which represents 3D position measurements.

### Model Parameters

#### Common Parameters

- `dt_pred`: Prediction time step (seconds)
- `V_max`: Maximum uncertainty before rejecting a track (m³)
- `V_certain`: Minimum uncertainty for confirming a track (m³)
- `N_meas`: Minimum number of measurements to confirm a track
- `l_threshold`: Measurement association log-likelihood threshold
- `dist_threshold`: Maximum distance between a state & measurement for association
- `r_diag`: Diagonal elements of measurement noise covariance matrix R
- `sigma_p`: Standard deviation of position for initial state covariance

#### Constant Velocity Model Parameters

- `sigma_a`: Standard deviation of acceleration noise
- `sigma_v`: Standard deviation of velocity for initial state covariance

#### Constant Acceleration Model Parameters

- `sigma_j`: Standard deviation of jerk noise
- `sigma_v`: Standard deviation of velocity for initial state covariance
- `sigma_a`: Standard deviation of acceleration for initial state covariance

## Usage

### Configuration

The package can be configured through YAML parameter files. Example configurations are provided in the `config/` directory:

- `kf_param_vel.yaml`: Configuration for the constant velocity model
- `kf_param_accel.yaml`: Configuration for the constant acceleration model

Here's an example configuration for the constant velocity model:

```yaml
/kf_tracker_node:
  ros__parameters:
    # Motion model selection: 0 = CONSTANT_VELOCITY
    model_type: 0
    
    # Rate of prediction step
    dt_pred: 0.02
    
    # Debugging
    print_debug_msg: False
    
    # Uncertainty thresholds
    V_max: 100.0          # Maximum uncertainty before rejecting a track [m^3]
    V_certain: 30.0       # Minimum uncertainty for confirming a track [m^3]
    
    # Track confirmation and filtering
    N_meas: 10                      # Minimum number of measurements to confirm a track
    l_threshold: -10.0              # Measurement association log-likelihood threshold
    dist_threshold: 5.0             # Maximum distance between a state & measurement to consider them as a match
    track_mesurement_timeout: 1.5   # Maximum time (seconds) from last measurement before considering a track uncertain
    
    # State estimation parameters
    sigma_a: 10.0    # Standard deviation of acceleration noise (for constant velocity model)
    sigma_p: 0.5     # Standard deviation of position (for initial state covariance)
    sigma_v: 2.0     # Standard deviation of velocity (for initial state covariance)
    
    # Diagonal elements of the process covariance matrix Q
    q_diag: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    
    # Diagonal elements of the measurement covariance matrix R
    r_diag: [0.01, 0.01, 0.01]
    
    # Individual measurement covariance components (for convenience)
    r_diag_x: 0.01
    r_diag_y: 0.01
    r_diag_z: 0.01
    
    # Frame settings
    tracking_frame: 'observer/odom'  # Provide estimate w.r.t this frame
    target_frameid: 'tag'            # Frame name of the detected target
```

### Launch Files

The package provides several launch files:

1. **kf_const_vel.launch.py**: Launch the tracker with a constant velocity model
   ```bash
   ros2 launch multi_target_kf kf_const_vel.launch.py
   ```

2. **kf_const_accel.launch.py**: Launch the tracker with a constant acceleration model
   ```bash
   ros2 launch multi_target_kf kf_const_accel.launch.py
   ```

3. **kf_rviz.launch.py**: Launch the tracker with RViz visualization
   ```bash
   ros2 launch multi_target_kf kf_rviz.launch.py
   ```

4. **modular_kf.launch.py**: Launch with configurable parameters
   ```bash
   # Example: Launch with custom parameters
   ros2 launch multi_target_kf modular_kf.launch.py model_type:=1 detections_topic:=my_measurements
   ```

### ROS Topics

#### Subscribed Topics

- `/measurement/pose_array` (`geometry_msgs/PoseArray`): Input measurements for the tracker

#### Published Topics

- `/kf/good_tracks_pose_array` (`geometry_msgs/PoseArray`): Filtered track positions (certain tracks only)
- `/kf/all_tracks_pose_array` (`geometry_msgs/PoseArray`): All track positions
- `/kf/good_tracks` (`multi_target_kf/KFTracks`): Detailed information about certain tracks
- `/kf/all_tracks` (`multi_target_kf/KFTracks`): Detailed information about all tracks

#### Custom Messages

- `KFTrack.msg`: Single track information including position, velocity, acceleration (when applicable), and covariance
- `KFTracks.msg`: Array of `KFTrack` messages with a header

## Extending the Package

### Creating a New Motion Model

To create a new motion model, follow these steps:

1. **Create a new header file** in the `include/multi_target_kf/` directory, e.g., `my_new_model.h`
2. **Implement the model** by inheriting from the `MotionModel` base class

Example template for a new motion model:

```cpp
// include/multi_target_kf/my_new_model.h
#ifndef MY_NEW_MODEL_H
#define MY_NEW_MODEL_H

#include "multi_target_kf/motion_model.h"
#include <vector>

class MyNewModel : public MotionModel {
private:
    // Model-specific parameters
    double param1_, param2_;

public:
    /* Constructor */
    MyNewModel() : param1_(0.0), param2_(0.0) {
        NUM_STATES = 9;        // Set the size of your state vector
        NUM_MEASUREMENTS = 3;  // Usually 3 for position measurements
        init();
    }
    
    ~MyNewModel() override {}

    bool init() override {
        // Initialize matrices and vectors
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
    bool setParam1(double p) {
        if (p <= 0) return false;
        param1_ = p;
        return true;
    }
    
    bool setParam2(double p) {
        if (p <= 0) return false;
        param2_ = p;
        return true;
    }
    
    // Implementation of base class virtual methods
    Eigen::VectorXd f(Eigen::VectorXd x, double dt) override {
        // Implement your state prediction function
        // This defines how states evolve over time
        
        // Example for a simple model:
        if (debug()) printf("[MyNewModel::f] Calculating f\n");
        
        if (dt <= 0) {
            printf("[MyNewModel::f] dt is <= 0. Returning same x\n");
            return x;
        }
        
        // IMPLEMENT YOUR STATE TRANSITION
        // For example:
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        
        // Custom state transition logic
        
        return (A * x);
    }
    
    // Other required method implementations
    // ...

    // Full implementation should include all pure virtual methods from MotionModel
    // See existing models like constant_vel.h or constant_accel.h for complete examples
};

#endif // MY_NEW_MODEL_H
```

### Registering the New Model

After creating your model, you need to:

1. **Update the `ModelType` enum** in `model_factory.h`:

```cpp
enum ModelType {
    CONSTANT_VELOCITY = 0,
    CONSTANT_ACCELERATION = 1,
    MY_NEW_MODEL = 2,  // Add your new model type
};
```

2. **Include your model header** in `model_factory.h`:

```cpp
#include "multi_target_kf/my_new_model.h"
```

3. **Add your model to the factory** in `model_factory.h`:

```cpp
static MotionModel* createModel(ModelType type) {
    switch (type) {
        case CONSTANT_VELOCITY:
            return new ConstantVelModel();
        case CONSTANT_ACCELERATION:
            return new ConstantAccelModel();
        case MY_NEW_MODEL:  // Add your new case
            return new MyNewModel();
        default:
            return nullptr;
    }
}

static const char* getModelName(ModelType type) {
    switch (type) {
        case CONSTANT_VELOCITY:
            return "Constant Velocity";
        case CONSTANT_ACCELERATION:
            return "Constant Acceleration";
        case MY_NEW_MODEL:  // Add your new case
            return "My New Model";
        default:
            return "Unknown Model";
    }
}
```

4. **Create a configuration file** for your model in the `config/` directory

5. **(Optional) Create a launch file** for your model in the `launch/` directory

## API Reference

### Key Classes

#### MotionModel

Abstract base class for all motion models. Defines the interface for state estimation.

```cpp
class MotionModel {
public:
    // Core KF methods to be implemented by derived classes
    virtual Eigen::VectorXd f(Eigen::VectorXd x, double dt) = 0; // State prediction
    virtual Eigen::VectorXd h(Eigen::VectorXd x) = 0; // Measurement model
    virtual Eigen::MatrixXd F(double dt) = 0; // State transition matrix
    virtual Eigen::MatrixXd H() = 0; // Measurement matrix
    
    // KF operations
    virtual kf_state predictX(kf_state s, double dt) = 0;
    virtual kf_state updateX(sensor_measurement z, kf_state s) = 0;
    virtual double logLikelihood(kf_state xx, sensor_measurement z) = 0;
    virtual kf_state initStateFromMeasurements(sensor_measurement z) = 0;
    virtual double computeDistance(kf_state xx, sensor_measurement zz) = 0;
    
    // Other methods...
};
```

#### KFTracker

Core implementation of the Kalman filter tracker.

```cpp
class KFTracker {
public:
    KFTracker(const TrackerConfig& config = TrackerConfig());
    KFTracker(ModelType model_type);
    ~KFTracker();
    
    const TrackerConfig& getConfig() const;
    bool setConfig(const TrackerConfig& config);
    ModelType getModelType() const;
    bool setModel(ModelType model_type);
    
    // Filter operations
    void filterLoop(double t);
    
    // Other methods...
};
```

#### TrackerROS

ROS interface for the Kalman filter tracker.

```cpp
class TrackerROS : public rclcpp::Node {
public:
    TrackerROS();
    ~TrackerROS();
    
private:
    // Methods for ROS communication
    void loadParameters();
    void poseArrayCallback(const geometry_msgs::msg::PoseArray & msg);
    void publishCertainTracks(void);
    void publishAllTracks(void);
    void filterLoop(void);
    void paramsTimerCallback(void);
};
```

### Data Structures

#### kf_state

Stores the current Kalman filter state estimate.

```cpp
struct kf_state {
   double time_stamp;
   Eigen::VectorXd x; // State estimate
   Eigen::MatrixXd P; // State estimate covariance
};
```

#### sensor_measurement

Stores a sensor measurement.

```cpp
struct sensor_measurement {
   double time_stamp; 
   unsigned int id; // Optional. Associated measurement ID, e.g. Apriltag ID
   Eigen::VectorXd z; // Measurements, e.g. 3D position
   Eigen::MatrixXd R; // Measurement covariance matrix
};
```

#### kf_track

Stores a track with its state history.

```cpp
struct kf_track {
   unsigned int id; // Unique track ID
   std::string frame_id;
   kf_state current_state;
   std::vector<kf_state> buffer;
   unsigned int n; // Number of received measurements
   double last_measurement_time;
};
```

#### TrackerConfig

Configuration structure for the Kalman filter tracker.

```cpp
struct TrackerConfig {
    ModelType model_type;
    double dt_pred;
    double V_max;
    double V_certain;
    int N_meas;
    double l_threshold;
    double dist_threshold;
    std::string tracking_frame;
    bool do_update_step;
    double measurement_off_time;
    bool use_track_id;
    double track_measurement_timeout;
    unsigned int state_buffer_size;
    std::vector<double> q_diag;
    std::vector<double> r_diag;
    double sigma_a;
    double sigma_p;
    double sigma_v;
    double sigma_j;
    bool debug;
    
    // Constructor and methods...
};
```

## Examples

### Basic Usage

Here's a basic example of how to use the package:

1. Launch the Kalman filter tracker with RViz visualization:
   ```bash
   ros2 launch multi_target_kf kf_rviz.launch.py
   ```
   It also runs simulation node that generates synthetic measurements with some noise

2. You should see the tracks being visualized in RViz, with red arrows representing all tracks and green arrows representing certain tracks.

### Custom Parameter Configuration

To use custom parameters:

1. Create a custom YAML file (or copy and modify an existing one):
   ```bash
   cp ~/ros2_ws/src/multi_target_kf/config/kf_param_vel.yaml ~/my_params.yaml
   # Edit ~/my_params.yaml to adjust parameters as needed
   ```

2. Launch the tracker with your custom parameters:
   ```bash
   ros2 launch multi_target_kf modular_kf.launch.py kf_yaml:=~/my_params.yaml
   ```

## Testing

The package includes unit tests for the motion models. To run the tests:

```bash
cd ~/ros2_ws
colcon test --packages-select multi_target_kf
colcon test-result --verbose
```

You can also write your own tests for new motion models by following the examples in `test/test_motion_models.cpp`.