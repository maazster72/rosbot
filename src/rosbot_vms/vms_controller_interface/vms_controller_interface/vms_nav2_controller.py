#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import vms_controller_interface.vms_controller_util as util

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.plan = None
        self.current_pose = PoseStamped()
        self.current_goal_pose_index = 0
        self.goal_pose = PoseStamped()
        self.threshold_linear = 0.1

        # Initialise BasicNavigator
        self.navigator = BasicNavigator()

        # Set our initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/vms_plan',
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

        # Timer to check goal status
        self.timer = self.create_timer(0.05, self.check_goal_status)

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        # Logging
        self.get_logger().info(f"New goal pose received: {self.goal_pose}")

    def plan_callback(self, msg: Path):
        self.plan = msg
        for pose in self.plan.poses:
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
        waypoints = [pose for pose in msg.poses]
        
        # Logging
        self.get_logger().info(f"New path received with {len(waypoints)} waypoints.")
        
        # Send waypoints to SimpleCommander
        self.navigator.goThroughPoses(waypoints)

    def odom_callback(self, msg: Odometry):
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation = msg.pose.pose.orientation

    def check_goal_status(self):
        # Check if SimpleCommander is active and monitor the current status
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Successfully reached all waypoints.")
            self.navigator.setInitialPose(self.current_pose)
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Navigation canceled.")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Navigation failed.")
        else:
            self.get_logger().info("Navigation has an unknown result.")

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