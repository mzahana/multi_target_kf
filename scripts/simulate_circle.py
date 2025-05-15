#!/usr/bin/env python3

import numpy as np
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose

class TrajectorySimulation(Node):

    def __init__(self):
        # Initiate the node
        super().__init__("trajectory_simulation_node")

        # Declare and read parameters
        self.declare_parameter("frame_id", "observer/odom")
        self.declare_parameter("num_targets", 3)
        self.declare_parameter("measurement_noise", 0.05)  # meters
        self.declare_parameter("process_noise", 0.1)       # m/s^2
        self.declare_parameter("publish_rate", 10.0)       # Hz
        self.declare_parameter("occlusion_probability", 0.0)  # Probability of occlusion
        self.declare_parameter("disappear_probability", 0.0)  # Probability of target disappearance

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._num_targets = self.get_parameter("num_targets").get_parameter_value().integer_value
        self._measurement_noise = self.get_parameter("measurement_noise").get_parameter_value().double_value
        self._process_noise = self.get_parameter("process_noise").get_parameter_value().double_value
        self._occlusion_probability = self.get_parameter("occlusion_probability").get_parameter_value().double_value
        self._disappear_probability = self.get_parameter("disappear_probability").get_parameter_value().double_value
        
        # Publishing rate
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._dt = 1.0 / publish_rate
        
        # Initialize targets
        self._targets = []
        for i in range(self._num_targets):
            target_type = random.choice(["circle", "line", "sine"])
            
            # Initialize target properties with some randomness
            target = {
                'id': i,
                'type': target_type,
                'visible': True,
                'x': random.uniform(-3.0, 3.0),
                'y': random.uniform(-3.0, 3.0),
                'z': random.uniform(1.0, 5.0),
                'vx': random.uniform(-0.5, 0.5),
                'vy': random.uniform(-0.5, 0.5),
                'vz': random.uniform(-0.1, 0.1),
                'theta': random.uniform(0, 2*np.pi),
                'radius': random.uniform(1.0, 3.0),
                'angular_speed': random.uniform(0.5, 1.5),
                'amplitude': random.uniform(1.0, 2.0),
                'frequency': random.uniform(0.2, 0.5)
            }
            self._targets.append(target)
        
        # Create publisher
        self._poses_pub = self.create_publisher(PoseArray, 'measurement/pose_array', 10)
        
        # Create timer
        self._timer = self.create_timer(self._dt, self.timer_callback)
        
        self.get_logger().info(f"Trajectory simulation started with {self._num_targets} targets")

    def update_target(self, target):
        # Add process noise (random accelerations)
        ax = np.random.normal(0, self._process_noise)
        ay = np.random.normal(0, self._process_noise)
        az = np.random.normal(0, self._process_noise)
        
        # Update velocity based on acceleration
        target['vx'] += ax * self._dt
        target['vy'] += ay * self._dt
        target['vz'] += az * self._dt
        
        # Update position based on target type
        if target['type'] == 'circle':
            # Circular trajectory
            target['theta'] += target['angular_speed'] * self._dt
            target['x'] = target['radius'] * np.cos(target['theta'])
            target['y'] = target['radius'] * np.sin(target['theta'])
            
        elif target['type'] == 'line':
            # Linear trajectory
            target['x'] += target['vx'] * self._dt
            target['y'] += target['vy'] * self._dt
            
            # Boundary checking - reverse direction if hitting boundaries
            if abs(target['x']) > 5.0:
                target['vx'] = -target['vx']
            if abs(target['y']) > 5.0:
                target['vy'] = -target['vy']
                
        elif target['type'] == 'sine':
            # Sinusoidal trajectory
            target['theta'] += target['angular_speed'] * self._dt
            target['x'] = target['theta']  # Move along x-axis
            target['y'] = target['amplitude'] * np.sin(target['frequency'] * target['theta'])
            
            # Boundary checking - reset if too far
            if target['x'] > 10.0:
                target['x'] = -10.0
        
        # Update z position with a slight oscillation
        target['z'] += target['vz'] * self._dt
        
        # Boundary checking for z
        if target['z'] < 0.5 or target['z'] > 5.5:
            target['vz'] = -target['vz']
            
        # Random visibility changes
        if random.random() < self._disappear_probability:
            target['visible'] = not target['visible']
            action = "appeared" if target['visible'] else "disappeared"
            self.get_logger().info(f"Target {target['id']} has {action}")
            
        return target

    def timer_callback(self):
        poses_msg = PoseArray()
        poses_msg.header.frame_id = self._frame_id
        poses_msg.header.stamp = self.get_clock().now().to_msg()
        
        for target in self._targets:
            # Update target position
            target = self.update_target(target)
            
            # Skip if not visible
            if not target['visible']:
                continue
                
            # Skip if occluded (random occlusion)
            if random.random() < self._occlusion_probability:
                continue
                
            # Create pose message with noise
            pose_msg = Pose()
            pose_msg.position.x = target['x'] + np.random.normal(0, self._measurement_noise)
            pose_msg.position.y = target['y'] + np.random.normal(0, self._measurement_noise)
            pose_msg.position.z = target['z'] + np.random.normal(0, self._measurement_noise)
            
            # Set orientation - could be used to indicate direction
            pose_msg.orientation.w = 1.0
            
            poses_msg.poses.append(pose_msg)
        
        # Publish poses
        self._poses_pub.publish(poses_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()