// include/multi_target_kf/tracker_ros.h
#ifndef TRACKER_ROS_H
#define TRACKER_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "multi_target_kf/msg/kf_track.hpp"
#include "multi_target_kf/msg/kf_tracks.hpp"
#include "multi_target_kf/kf_tracker.h"
#include "multi_target_kf/msg/detection.hpp"
#include "multi_target_kf/msg/detections.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief ROS interface for the Kalman filter tracker
 */
class TrackerROS : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    TrackerROS();
    
    /**
     * @brief Destructor
     */
    ~TrackerROS();
    
private:
    KFTracker *kf_tracker_;                    /**< Pointer to the KF tracker */
    TrackerConfig config_;                     /**< Tracker configuration */
    std::string target_frameid_;               /**< Target frame name which will be post-fixed by the target unique number e.g. "tag", post-fixed will be like "tag1" */
    bool listen_tf_;                           /**< listens to TF to find transforms of received measurements w.r.t. tracking_frame_ */

    rclcpp::TimerBase::SharedPtr kf_loop_timer_;   /**< Timer for the KF loop */
    rclcpp::TimerBase::SharedPtr params_timer_;    /**< Timer for parameter updates */

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr good_poses_pub_;           /**< ROS publisher for KF estimated (certain) tracks positions */
    rclcpp::Publisher<multi_target_kf::msg::KFTracks>::SharedPtr good_tracks_pub_;         /**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_poses_pub_;            /**< ROS publisher for KF estimated (ALL) tracks positions */
    rclcpp::Publisher<multi_target_kf::msg::KFTracks>::SharedPtr all_tracks_pub_;          /**< ROS publisher for KF estimated tracks positions, using custom KFTracks.msg */
   
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;        /**< Subscriber to measurements. */
    rclcpp::Subscription<multi_target_kf::msg::Detections>::SharedPtr detections_sub_;     /**< Subscriber to detection messages */

    

    /**
     * @brief Load parameters from ROS parameter server
     */
    void loadParameters();

    /**
     * @brief Measurement ROS Callback. Updates measurement_set_
     * 
     * @param msg Holds PoseArray measurement
     */
    void poseArrayCallback(const geometry_msgs::msg::PoseArray & msg);

    /**
     * @brief Publish certain tracks as PoseArray and KFTracks
     */
    void publishCertainTracks(void);
    
    /**
     * @brief Publish all tracks as PoseArray and KFTracks
     */
    void publishAllTracks(void);

    /**
     * @brief Executes the KF predict and update steps.
     */
    void filterLoop(void);

    /**
     * @brief Parameters timer's callback to update parameters from ROS parameter server
     */
    void paramsTimerCallback(void);

    /**
     * @brief Detections callback for detection messages with additional information
     */
    void detectionsCallback(const multi_target_kf::msg::Detections & msg);
    
    /**
     * @brief Convert enhanced_measurement to sensor_measurement
     */
    sensor_measurement convertToBasicMeasurement(const enhanced_measurement& enhanced);
    
    /**
     * @brief Convert Detection message to enhanced_measurement
     */
    enhanced_measurement convertFromDetectionMsg(const multi_target_kf::msg::Detection& detection_msg);
};

#endif // TRACKER_ROS_H