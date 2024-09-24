#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Initialize robot's current position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_goal_index = 0  # Track the current goal
        self.plan = []  # Store the list of waypoints in the plan

        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Flag to indicate if the initial position is set
        self.initial_position_set = False

        # Timer to control movement towards the next goal
        self.timer = self.create_timer(0.1, self.move_towards_goal)

    def plan_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received an empty plan')
            return

        # Set the robot's initial position to the first pose if not set
        if not self.initial_position_set:
            initial_pose = msg.poses[0].pose.position
            self.robot_x = initial_pose.x
            self.robot_y = initial_pose.y
            self.initial_position_set = True
            self.get_logger().info(f'Initial position set to: {self.robot_x}, {self.robot_y}')

        # Skip the first pose and store the rest of the plan
        self.plan = msg.poses[1:]
        self.current_goal_index = 0  # Reset the goal index

    def move_towards_goal(self):
        if self.current_goal_index >= len(self.plan):
            self.get_logger().info('Plan completed.')
            return

        # Get the next pose in the plan
        next_pose = self.plan[self.current_goal_index]
        target_x = next_pose.pose.position.x
        target_y = next_pose.pose.position.y

        # Calculate the difference
        delta_x = target_x - self.robot_x
        delta_y = target_y - self.robot_y

        # Calculate the distance to the target
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        angle_to_target = math.atan2(delta_y, delta_x)

        # Create a Twist message for velocity control
        cmd_msg = Twist()

        if distance > 0.1:  # Threshold to move towards the goal
            cmd_msg.linear.x = min(0.5, distance)  # Move at max speed of 0.5 m/s
            cmd_msg.angular.z = angle_to_target  # Rotate towards the target
        else:
            # If goal is reached, stop and move to the next goal
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.get_logger().info(f'Reached goal pose: {target_x}, {target_y}')
            self.current_goal_index += 1  # Move to the next goal

        # Publish the velocity command
        self.cmd_vel_publisher.publish(cmd_msg)

        # Update the robot's current position based on the movement
        self.robot_x += cmd_msg.linear.x * math.cos(angle_to_target)
        self.robot_y += cmd_msg.linear.x * math.sin(angle_to_target)

        # Log the robot's current position
        self.get_logger().info(f'Robot current position: {self.robot_x}, {self.robot_y}')


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
