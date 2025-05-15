// test/test_motion_models.cpp
#include <gtest/gtest.h>
#include "multi_target_kf/model_factory.h"

class MotionModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code that will be called before each test
    }

    void TearDown() override {
        // Cleanup code that will be called after each test
        if (model_ != nullptr) {
            delete model_;
            model_ = nullptr;
        }
    }

    MotionModel* model_ = nullptr;
};

TEST_F(MotionModelTest, ConstantVelocityModelCreation) {
    model_ = ModelFactory::createModel(CONSTANT_VELOCITY);
    ASSERT_NE(model_, nullptr);
    EXPECT_EQ(model_->numStates(), 6);
    EXPECT_EQ(model_->numMeasurements(), 3);
}

TEST_F(MotionModelTest, ConstantAccelerationModelCreation) {
    model_ = ModelFactory::createModel(CONSTANT_ACCELERATION);
    ASSERT_NE(model_, nullptr);
    EXPECT_EQ(model_->numStates(), 9);
    EXPECT_EQ(model_->numMeasurements(), 3);
}

TEST_F(MotionModelTest, ConstantVelocityPrediction) {
    model_ = ModelFactory::createModel(CONSTANT_VELOCITY);
    ASSERT_NE(model_, nullptr);
    
    // Initialize a state vector with known position and velocity
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7; // [x, y, z, vx, vy, vz]
    
    // Predict one second ahead
    Eigen::VectorXd x_pred = model_->f(x, 1.0);
    
    // Expected result: position += velocity * dt
    EXPECT_NEAR(x_pred(0), 1.5, 1e-5); // x + vx*dt
    EXPECT_NEAR(x_pred(1), 2.6, 1e-5); // y + vy*dt
    EXPECT_NEAR(x_pred(2), 3.7, 1e-5); // z + vz*dt
    
    // Velocity should remain unchanged
    EXPECT_NEAR(x_pred(3), 0.5, 1e-5);
    EXPECT_NEAR(x_pred(4), 0.6, 1e-5);
    EXPECT_NEAR(x_pred(5), 0.7, 1e-5);
}

TEST_F(MotionModelTest, ConstantAccelerationPrediction) {
    model_ = ModelFactory::createModel(CONSTANT_ACCELERATION);
    ASSERT_NE(model_, nullptr);
    
    // Initialize a state vector with known position, velocity, and acceleration
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7, 0.1, 0.2, 0.3; // [x, y, z, vx, vy, vz, ax, ay, az]
    
    // Predict one second ahead
    Eigen::VectorXd x_pred = model_->f(x, 1.0);
    
    // Expected result: position += velocity * dt + 0.5 * acceleration * dt^2
    EXPECT_NEAR(x_pred(0), 1.5 + 0.5 * 0.1, 1e-5); // x + vx*dt + 0.5*ax*dt^2
    EXPECT_NEAR(x_pred(1), 2.6 + 0.5 * 0.2, 1e-5); // y + vy*dt + 0.5*ay*dt^2
    EXPECT_NEAR(x_pred(2), 3.7 + 0.5 * 0.3, 1e-5); // z + vz*dt + 0.5*az*dt^2
    
    // Velocity += acceleration * dt
    EXPECT_NEAR(x_pred(3), 0.5 + 0.1, 1e-5); // vx + ax*dt
    EXPECT_NEAR(x_pred(4), 0.6 + 0.2, 1e-5); // vy + ay*dt
    EXPECT_NEAR(x_pred(5), 0.7 + 0.3, 1e-5); // vz + az*dt
    
    // Acceleration should remain unchanged
    EXPECT_NEAR(x_pred(6), 0.1, 1e-5);
    EXPECT_NEAR(x_pred(7), 0.2, 1e-5);
    EXPECT_NEAR(x_pred(8), 0.3, 1e-5);
}

TEST_F(MotionModelTest, ConstantVelocityMeasurement) {
    model_ = ModelFactory::createModel(CONSTANT_VELOCITY);
    ASSERT_NE(model_, nullptr);
    
    // Initialize a state vector
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7; // [x, y, z, vx, vy, vz]
    
    // Compute expected measurement
    Eigen::VectorXd z = model_->h(x);
    
    // Expected result: only position is observed
    ASSERT_EQ(z.size(), 3);
    EXPECT_NEAR(z(0), 1.0, 1e-5); // x
    EXPECT_NEAR(z(1), 2.0, 1e-5); // y
    EXPECT_NEAR(z(2), 3.0, 1e-5); // z
}

TEST_F(MotionModelTest, ConstantAccelerationMeasurement) {
    model_ = ModelFactory::createModel(CONSTANT_ACCELERATION);
    ASSERT_NE(model_, nullptr);
    
    // Initialize a state vector
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7, 0.1, 0.2, 0.3; // [x, y, z, vx, vy, vz, ax, ay, az]
    
    // Compute expected measurement
    Eigen::VectorXd z = model_->h(x);
    
    // Expected result: only position is observed
    ASSERT_EQ(z.size(), 3);
    EXPECT_NEAR(z(0), 1.0, 1e-5); // x
    EXPECT_NEAR(z(1), 2.0, 1e-5); // y
    EXPECT_NEAR(z(2), 3.0, 1e-5); // z
}

TEST_F(MotionModelTest, ConstantVelocityPredictAndUpdate) {
    model_ = ModelFactory::createModel(CONSTANT_VELOCITY);
    ASSERT_NE(model_, nullptr);
    
    // Initialize state
    kf_state state;
    state.time_stamp = 0.0;
    state.x = Eigen::VectorXd::Zero(6);
    state.x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7; // [x, y, z, vx, vy, vz]
    state.P = Eigen::MatrixXd::Identity(6, 6);
    
    // Set up model parameters
    ConstantVelModel* vel_model = static_cast<ConstantVelModel*>(model_);
    vel_model->setSigmaA(0.1);
    vel_model->setSigmaP(0.1);
    vel_model->setSigmaV(0.1);
    std::vector<double> r_diag = {0.01, 0.01, 0.01};
    vel_model->R(r_diag);
    
    // Predict
    double dt = 1.0;
    kf_state predicted_state = model_->predictX(state, dt);
    
    // Create a measurement with some noise
    sensor_measurement meas;
    meas.time_stamp = 1.0;
    meas.z = Eigen::VectorXd::Zero(3);
    meas.z << 1.6, 2.7, 3.8; // Slightly different from the prediction
    
    // Update
    kf_state updated_state = model_->updateX(meas, predicted_state);
    
    // The updated state should be closer to the measurement than the prediction
    EXPECT_NEAR(updated_state.x(0), meas.z(0), 0.1);
    EXPECT_NEAR(updated_state.x(1), meas.z(1), 0.1);
    EXPECT_NEAR(updated_state.x(2), meas.z(2), 0.1);
}

TEST_F(MotionModelTest, ConstantAccelerationPredictAndUpdate) {
    model_ = ModelFactory::createModel(CONSTANT_ACCELERATION);
    ASSERT_NE(model_, nullptr);
    
    // Initialize state
    kf_state state;
    state.time_stamp = 0.0;
    state.x = Eigen::VectorXd::Zero(9);
    state.x << 1.0, 2.0, 3.0, 0.5, 0.6, 0.7, 0.1, 0.2, 0.3; // [x, y, z, vx, vy, vz, ax, ay, az]
    state.P = Eigen::MatrixXd::Identity(9, 9);
    
    // Set up model parameters
    ConstantAccelModel* accel_model = static_cast<ConstantAccelModel*>(model_);
    accel_model->setSigmaJ(0.1);
    accel_model->setSigmaP(0.1);
    accel_model->setSigmaV(0.1);
    accel_model->setSigmaA(0.1);
    std::vector<double> r_diag = {0.01, 0.01, 0.01};
    accel_model->R(r_diag);
    
    // Predict
    double dt = 1.0;
    kf_state predicted_state = model_->predictX(state, dt);
    
    // Create a measurement with some noise
    sensor_measurement meas;
    meas.time_stamp = 1.0;
    meas.z = Eigen::VectorXd::Zero(3);
    meas.z << 1.65, 2.8, 3.95; // Slightly different from the prediction
    
    // Update
    kf_state updated_state = model_->updateX(meas, predicted_state);
    
    // The updated state should be closer to the measurement than the prediction
    EXPECT_NEAR(updated_state.x(0), meas.z(0), 0.1);
    EXPECT_NEAR(updated_state.x(1), meas.z(1), 0.1);
    EXPECT_NEAR(updated_state.x(2), meas.z(2), 0.1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}