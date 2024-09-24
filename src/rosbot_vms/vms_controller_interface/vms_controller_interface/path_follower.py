from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

"""
Navigation to follow a path from the /plan topic
"""

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.navigator = BasicNavigator()
        
        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

    def plan_callback(self, msg: Path):
        # Get the first pose from the plan
        if len(msg.poses) == 0:
            self.get_logger().warn('Received an empty plan')
            return
        
        initial_pose = msg.poses[0]

        # Set the initial pose to the first pose in the plan
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.setInitialPose(initial_pose)

        # Wait until the navigation system is active
        self.navigator.waitUntilNav2Active()

        # Command the robot to follow the received path
        self.navigator.followPath(msg)

        i = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f"Distance to goal: {feedback.distance_to_goal:.3f}m, Speed: {feedback.speed:.3f}m/s")
            i += 1

        # Handle the result of the navigation task
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Unknown return status!')

        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    
    # Spin the node so it can listen to the /plan topic
    rclpy.spin(path_follower)
    
    # Shutdown when done
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
