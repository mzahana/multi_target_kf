#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose

class CircleTrajectory(Node):

    def __init__(self):
        # Initiate the node
        super().__init__("circle_traj_node")

        self._radius =2.0 # meters
        self._angular_speed = 1.0 # rad/s
        self._altitude = 3.0 # meters
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0 # rad
        self._dt = 0.1 # seconds

        self._poses_msg = PoseArray()

        self.declare_parameter("frame_id", "map")
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._timer = self.create_timer(self._dt, self.timerCallback)

        self._poses_pub = self.create_publisher( PoseArray,'measurement/pose_array',10)

    def timerCallback(self):
        self._x = self._radius + np.cos(self._theta)
        self._y = self._radius + np.sin(self._theta)

        pose_msg = Pose()
        pose_msg.position.x = self._x
        pose_msg.position.y = self._y
        pose_msg.position.z = self._altitude

        self._theta += self._angular_speed * self._dt

        self._poses_msg = PoseArray()
        self._poses_msg.poses.append(pose_msg)
        self._poses_msg.header.frame_id = self._frame_id
        self._poses_msg.header.stamp = self.get_clock().now().to_msg()
        self._poses_pub.publish(self._poses_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectory()
    node.get_logger().info("Circular trajectory node has started")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()  