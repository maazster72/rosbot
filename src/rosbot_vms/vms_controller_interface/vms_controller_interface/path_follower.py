#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Path, Odometry
import rclpy
from rclpy.node import Node
import math
from scipy.interpolate import CubicSpline
import numpy as np
import vms_controller_interface.vms_controller as controller
import vms_controller_interface.vms_controller_util as util

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.plan = None
        self.current_pose = PoseStamped()
        self.current_goal_pose_index = 0
        self.goal_pose = PoseStamped()
        self.distance_to_goal = float("inf")
        self.threshold_linear = 0.1

        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Subscribe to the /goal_pose topic
        self.odom_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for goal pose
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to control movement towards the next goal
        self.timer = self.create_timer(0.02, self.move_towards_goal)

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)
        # Logging
        self.get_logger().info(f"New goal pose received: {self.goal_pose}")

    def plan_callback(self, msg: Path):
        self.current_goal_pose_index = 0
        self.goal_pose_publisher.publish(msg.poses[self.current_goal_pose_index])
        self.plan = msg
        # Logging
        self.get_logger().info(f"New path received: {self.plan}")

    def odom_callback(self, msg: Odometry):
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation = msg.pose.pose.orientation
    
    def move_towards_goal(self):
        if self.plan:
            self.get_logger().info(f"Current Goal: {self.goal_pose}")
            self.get_logger().info(f"Current pose (Odomoetry): {self.current_pose}")
            self.get_logger().info(f"Distance to current goal: {self.distance_to_goal}")
            self.navThroughPoses()
        elif self.goal_pose:
            # Logging
            self.get_logger().info(f"Goal: {self.goal_pose}")
            self.get_logger().info(f"Current pose (Odomoetry): {self.current_pose}")
            self.get_logger().info(f"Distance to goal: {self.distance_to_goal}")
            self.navToPose()
        else:
            self.cmd_vel_publisher.publish(Twist())

    def navToPose(self):
        if abs(self.distance_to_goal) < self.threshold_linear:
            self.goal_pose = None
            self.distance_to_goal = None
            # Logging
            self.get_logger().info(f"Goal pose reached. Executed plan successfully")
        else:
            self.compute_cmd_velocity()

    def navThroughPoses(self):
        if abs(self.distance_to_goal) < self.threshold_linear:
            # Logging
            self.get_logger().info(f"Goal pose reached. ({self.current_goal_pose_index + 1}/{len(self.plan.poses)})")
            if self.current_goal_pose_index != (len(self.plan.poses) - 1):
                self.current_goal_pose_index += 1
                self.goal_pose_publisher.publish(self.plan.poses[self.current_goal_pose_index])
            else:
                self.plan = None
                self.current_goal_pose_index = None
                self.goal_pose = None
                self.distance_to_goal = None
                # Logging
                self.get_logger().info(f"Goal pose reached. Executed plan successfully")
        else:
            self.compute_cmd_velocity()

    def compute_cmd_velocity(self):
        # Compute cmd_vel
        cmd_vel = controller.move_to_target(self.current_pose, self.goal_pose)
        # Logging
        self.get_logger().info(f"Moving with cmd_vel: {cmd_vel}")
        # Publish cmd_vel
        self.cmd_vel_publisher.publish(cmd_vel)
        # Update distance
        self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()

    # Spin the node to listen for messages
    rclpy.spin(path_follower)

    # Shutdown when done
    path_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()