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
from tf2_ros import TransformListener, Buffer
import tf_transformations
from rclpy.time import Time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.plan = None
        self.current_pose = PoseStamped()
        self.current_goal_pose_index = 0
        self.goal_pose = PoseStamped()
        self.distance_to_goal = float("inf")
        self.threshold_linear = 0.05
        self.threshold_angular = 0.1
        self.rotate = False
        self.linear = False

        # Create a TransformListener and Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set the fixed frame (e.g., map) and target frame (e.g., base_link)
        self.fixed_frame = 'map'
        self.target_frame = 'base_link'

        # Create a timer to periodically check for transforms
        self.timer = self.create_timer(0.02, self.update_current_pose)

        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/vms_plan',
            self.plan_callback,
            10)

        # Subscribe to the /odom topic
        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     10)

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
        if(self.check_if_rotate_needed(self.current_pose, self.goal_pose)):
            self.rotate = True
            self.linear = False
            # Logging
            self.get_logger().info(f"Rotate mode activated")
        else:
            self.rotate = False
            self.linear = True
            # Logging
            self.get_logger().info(f"Linear mode activated")

    def plan_callback(self, msg: Path):
        self.current_goal_pose_index = 0
        self.goal_pose_publisher.publish(msg.poses[self.current_goal_pose_index])
        self.plan = msg
        # Logging
        # self.get_logger().info(f"New path received: {self.plan}")

    def update_current_pose(self):
        try:
            # Look up the transform from fixed_frame to target_frame
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.target_frame,
                now
            )
            
            # Update current_pose with translation and rotation from transform
            self.current_pose.header.stamp = transform.header.stamp
            self.current_pose.header.frame_id = self.fixed_frame
            
            # Set position
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
            self.current_pose.pose.position.z = transform.transform.translation.z
            
            # Set orientation
            self.current_pose.pose.orientation = transform.transform.rotation

            # Log the updated pose
            # self.get_logger().info(f"Updated Pose: {self.current_pose}")
        
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    # def odom_callback(self, msg: Odometry):
    #     self.current_pose.pose.position = msg.pose.pose.position
    #     self.current_pose.pose.orientation = msg.pose.pose.orientation
    
    def move_towards_goal(self):
        if self.plan:
            # self.get_logger().info(f"Current Goal: {self.goal_pose}")
            # self.get_logger().info(f"Current pose: {self.current_pose}")
            self.get_logger().info(f"Distance to current goal: {self.distance_to_goal}")
            self.navThroughPoses()
        elif self.goal_pose:
            # Logging
            # self.get_logger().info(f"Goal: {self.goal_pose}")
            # self.get_logger().info(f"Current pose (Odomoetry): {self.current_pose}")
            # self.get_logger().info(f"Distance to goal: {self.distance_to_goal}")
            self.navToPose()
        else:
            self.cmd_vel_publisher.publish(Twist())

    def navToPose(self):
        if self.current_pose and self.goal_pose:
            # Update distance
            self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)
        if abs(self.distance_to_goal) < self.threshold_linear:
            self.goal_pose = None
            self.distance_to_goal = float("inf")
            # Logging
            self.get_logger().info(f"Goal pose reached. Executed plan successfully")
        else:
            self.compute_cmd_velocity()

    def navThroughPoses(self):
        if self.current_pose and self.goal_pose:
            # Update distance
            self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)
        if abs(self.distance_to_goal) < self.threshold_linear:
            # Logging
            self.get_logger().info(f"Goal pose reached. ({self.current_goal_pose_index + 1}/{len(self.plan.poses)})")
            if self.current_goal_pose_index != (len(self.plan.poses) - 1):
                self.current_goal_pose_index += 1
                self.goal_pose_publisher.publish(self.plan.poses[self.current_goal_pose_index])
                if(self.check_if_rotate_needed(self.current_pose, self.goal_pose)):
                    self.linear = False
                    self.rotate = True
                    # Logging
                    self.get_logger().info(f"Linear mode deactivated")
                    self.get_logger().info(f"Rotate mode activated")
                else:
                    self.rotate = False
                    self.linear = True
                    # Logging
                    self.get_logger().info(f"Linear mode activated")
            else:
                self.plan = None
                self.current_goal_pose_index = None
                self.goal_pose = None
                self.distance_to_goal = float("inf")
                # Logging
                self.get_logger().info(f"Goal pose reached. Executed plan successfully")
        else:
            self.compute_cmd_velocity()

    def compute_cmd_velocity(self):
        # Compute cmd_vel
        if self.rotate:
            self.get_logger().info(f"Rotate mode...")
            cmd_vel = controller.orient_to_target(self.current_pose, self.goal_pose)
            if(self.check_if_rotate_needed(self.current_pose, self.goal_pose)):
                self.linear = False
                self.rotate = True
                # Logging
                self.get_logger().info(f"Linear mode deactivated")
                self.get_logger().info(f"Rotate mode activated")
            else:
                self.rotate = False
                self.linear = True
                # Logging
                self.get_logger().info(f"Rotate mode deactivated")
                self.get_logger().info(f"Linear mode activated")
        elif self.current_pose and self.goal_pose and self.linear:
            self.get_logger().info(f"Linear mode...")
            cmd_vel = controller.move_to_target(self.current_pose, self.goal_pose)
        else:
            cmd_vel = Twist()
        # Logging
        self.get_logger().info(f"Moving with cmd_vel: {cmd_vel}")
        # Publish cmd_vel
        self.cmd_vel_publisher.publish(cmd_vel)

    def check_if_rotate_needed(self, current_pose, target_pose):
        yaw_rotation = controller.compute_required_yaw_rotation(current_pose, target_pose)
        self.get_logger().info(f"Yaw rotation: {yaw_rotation}")
        if abs(yaw_rotation) > self.threshold_angular:
            return True
        else:
            return False

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