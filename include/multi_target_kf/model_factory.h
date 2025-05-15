// include/multi_target_kf/model_factory.h
#ifndef MODEL_FACTORY_H
#define MODEL_FACTORY_H

#include "multi_target_kf/motion_model.h"
#include "multi_target_kf/constant_vel.h"
#include "multi_target_kf/constant_accel.h"
#include "multi_target_kf/ukf_adaptive_accel.h"

/**
 * @brief Enumeration of available motion model types
 */
enum ModelType {
    CONSTANT_VELOCITY = 0,
    CONSTANT_ACCELERATION = 1,
    ADAPTIVE_ACCEL_UKF = 2,
    // Add more model types here as they become available
};

/**
 * @brief Factory class for creating motion model instances
 */
class ModelFactory {
public:
    /**
     * @brief Creates and returns a motion model instance of the specified type
     * @param type The type of motion model to create
     * @return Pointer to the created motion model, or nullptr if type is invalid
     */
    static MotionModel* createModel(ModelType type) {
        switch (type) {
            case CONSTANT_VELOCITY:
                return new ConstantVelModel();
            case CONSTANT_ACCELERATION:
                return new ConstantAccelModel(); // Add this case
            case ADAPTIVE_ACCEL_UKF:
                return new AdaptiveAccelUKF();
            // Add more cases for future models
            default:
                return nullptr;
        }
    }
    
    /**
     * @brief Get the name of a model type
     * @param type The model type
     * @return String representation of the model type
     */
    static const char* getModelName(ModelType type) {
        switch (type) {
            case CONSTANT_VELOCITY:
                return "Constant Velocity";
            case CONSTANT_ACCELERATION:
                return "Constant Acceleration";
            case ADAPTIVE_ACCEL_UKF:
                return "Adaptive Acceleration UKF";
            // Add more cases for future models
            default:
                return "Unknown Model";
        }
    }
};

#endif // MODEL_FACTORY_H