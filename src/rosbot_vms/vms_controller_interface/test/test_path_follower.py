import unittest
import launch
import launch_testing
from launch_ros.actions import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import rclpy
import time


class PathFollowerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        """Set up the test node and publisher/subscriber pairs."""
        self.node = RclpyNode('test_node')
        self.cmd_vel_subscriber = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.goal_pose_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        self.plan_publisher = self.node.create_publisher(
            Path,
            '/vms_plan',
            10
        )
        self.cmd_vel_message = None
        self.received_messages = []

    def tearDown(self):
        self.node.destroy_node()

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel messages."""
        self.received_messages.append(msg)
        self.cmd_vel_message = msg

    def test_goal_pose_navigation(self):
        """Test that the robot publishes correct cmd_vel when navigating to a goal pose."""
        # Publish a goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.orientation.w = 1.0
        self.goal_pose_publisher.publish(goal_pose)

        # Allow time for the message to be processed
        time.sleep(1.0)

        # Check that the /cmd_vel topic received a message
        self.assertIsNotNone(self.cmd_vel_message, "No cmd_vel message received")
        self.assertGreater(len(self.received_messages), 0, "No messages received")

    def test_plan_navigation(self):
        """Test that the robot can follow a series of poses in a path."""
        # Publish a path
        path = Path()
        path.header.frame_id = "map"
        for i in range(3):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = i * 1.0
            pose.pose.position.y = i * 1.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.plan_publisher.publish(path)

        # Allow time for the message to be processed
        time.sleep(1.0)

        # Check that the /cmd_vel topic is publishing as the robot follows the path
        self.assertGreater(len(self.received_messages), 0, "No cmd_vel messages received during path navigation")

    def test_reset_after_completion(self):
        """Test that the robot stops after completing the plan."""
        # Publish a path with a single pose
        path = Path()
        path.header.frame_id = "map"
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
        self.plan_publisher.publish(path)

        # Allow time for the robot to process the plan and complete it
        time.sleep(2.0)

        # Verify that the robot stops after completing the plan
        last_cmd_vel = self.cmd_vel_message
        self.assertIsNotNone(last_cmd_vel, "No final cmd_vel message received")
        self.assertEqual(last_cmd_vel.linear.x, 0.0, "Robot did not stop after completing the plan")
        self.assertEqual(last_cmd_vel.angular.z, 0.0, "Robot did not stop after completing the plan")

    def test_no_plan_or_goal(self):
        """Test that the robot publishes no cmd_vel when no goal or plan is active."""
        # Wait to ensure no actions are happening
        time.sleep(1.0)

        # Verify no messages are being published
        self.assertEqual(len(self.received_messages), 0, "Unexpected cmd_vel messages were published")


def generate_test_description():
    """Launch the PathFollower node."""
    path_follower_node = Node(
        package='vms_controller_interface',
        executable='path_follower',
        name='path_follower',
        output='screen',
    )

    return launch.LaunchDescription([
        path_follower_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_follower_node': path_follower_node,
    }


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('vms_controller_interface', 'test_path_follower', PathFollowerTest)
