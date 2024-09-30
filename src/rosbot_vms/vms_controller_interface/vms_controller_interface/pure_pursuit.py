import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.lookahead_distance = 1.5  # meters, tune this as needed
        self.linear_speed = 1.0        # meters per second, can adjust based on speed needs
        self.goal_reached_threshold = 0.2  # meters, distance within which the goal is considered reached

        # Subscriber to the path
        self.path_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Initialize variables
        self.current_path = None
        self.current_pose = None

        # Timer to update control commands
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, path_msg):
        """Callback for receiving the path to follow"""
        self.get_logger().info("Received a new path!")
        
        # Store the received path
        self.current_path = path_msg.poses  # List of PoseStamped
        
        # Set the current pose as the first pose in the path
        if len(self.current_path) > 0:
            self.current_pose = self.current_path[0].pose
            self.get_logger().info(f"Initial pose set to: x={self.current_pose.position.x}, y={self.current_pose.position.y}")
        else:
            self.get_logger().warn("Received an empty path!")


    def control_loop(self):
        """Main control loop for path tracking"""
        if self.current_path is None or len(self.current_path) == 0:
            self.get_logger().warn("No path to follow.")
            return

        if self.current_pose is None:
            self.get_logger().warn("No current pose available.")
            return

        # Find the look-ahead target pose
        target_pose = self.get_lookahead_point()

        if target_pose is None:
            self.get_logger().info("Goal reached!")
            self.publish_stop()
            return

        # Compute the control commands (linear and angular velocities)
        linear_velocity, angular_velocity = self.compute_control_command(target_pose)

        # Publish the Twist message with the control commands
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def get_lookahead_point(self):
        """Find the look-ahead point on the path that is ahead of the vehicle by the look-ahead distance"""
        for pose_stamped in self.current_path:
            pose = pose_stamped.pose
            distance = self.get_distance_to_pose(pose)

            if distance >= self.lookahead_distance:
                return pose

        # If no look-ahead point is found, return the last point
        return self.current_path[-1].pose

    def get_distance_to_pose(self, target_pose):
        """Compute the distance from the current position to the target pose"""
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def compute_control_command(self, target_pose):
        """Compute the linear and angular velocities using the Pure Pursuit algorithm"""
        # Extract the current and target positions
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        # Compute the heading to the target point
        dx = target_x - current_x
        dy = target_y - current_y
        target_heading = math.atan2(dy, dx)

        # Compute the error in the orientation (heading error)
        current_heading = self.get_yaw_from_pose(self.current_pose)
        heading_error = self.normalize_angle(target_heading - current_heading)

        # Compute linear and angular velocities
        linear_velocity = self.linear_speed
        angular_velocity = 2 * heading_error  # Adjust gain if needed for sharper turns

        return linear_velocity, angular_velocity

    def get_yaw_from_pose(self, pose):
        """Extract the yaw (rotation around the z-axis) from a pose's orientation"""
        orientation_q = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        return yaw

    def euler_from_quaternion(self, orientation_q):
        """Convert a quaternion into Euler angles (roll, pitch, yaw)"""
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return 0.0, 0.0, yaw  # Return only yaw (z-axis rotation)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_stop(self):
        """Publish a stop command (zero velocities)"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
