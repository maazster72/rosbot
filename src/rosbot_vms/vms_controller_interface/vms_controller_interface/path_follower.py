#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
import math
from scipy.interpolate import CubicSpline
import numpy as np


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.k = 1.0  # Stanley control gain
        self.max_steering_angle = 0.5 # Max steering angle in radians

        self.plan = None
        self.current_pose = PoseStamped()
        self.smoothed_path = None

        self.goal_achieved = True

        # Subscribe to the /plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for smoothed path
        self.smooth_path_publisher = self.create_publisher(Path, '/smooth_path', 10)

        # # Publisher for current pose
        # self.current_pose

        # Timer to control movement towards the next goal
        self.timer = self.create_timer(0.1, self.move_towards_goal)

    # def pose_callback(self, msg: PoseStamped):
    #     self.current_pose = msg

    def plan_callback(self, msg: Path):
        self.plan = msg

        self.goal_achieved = False

        self.get_logger().info(f"Received path with {len(msg.poses)} poses")
        try:
            self.smoothed_path = self.smooth_nav_path(self.plan)
            self.smooth_path_publisher.publish(self.smoothed_path)
            self.get_logger().info(f"Published smoothed path with {len(self.smoothed_path.poses)} poses")

            self.current_pose = self.smoothed_path.poses[0]
            self.get_logger().info(f"Starting position: {self.current_pose}")

        except ValueError as e:
            self.get_logger().warn(f"Smoothing failed: {e}")

    def move_towards_goal(self):
        if self.goal_achieved:
            # Stop the robot by setting cmd_vel to zero
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)

        elif self.current_pose and self.smoothed_path:
            # Compute and publish cmd velocities to follow the smoothed path
            cmd_vel = self.compute_cmd_velocities(self.current_pose, self.smoothed_path)
            self.cmd_vel_publisher.publish(cmd_vel)
            self.get_logger().info(f"Moving with following cmd_vel: {cmd_vel}")

            self.update_current_pose(cmd_vel)
            self.get_logger().info(f"Current position: {self.current_pose}")

            self.check_goal_reached()

    def smooth_nav_path(self, input_path, num_points=100):        
        # Extract x and y positions from the input path
        x_points = [pose.pose.position.x for pose in input_path.poses]
        y_points = [pose.pose.position.y for pose in input_path.poses]

        # Ensure there are enough points to interpolate
        if len(x_points) < 3 or len(y_points) < 3:
            raise ValueError("Not enough points to perform spline interpolation")
        
        # Perform cubic spline interpolation
        t = np.arange(len(x_points))  # Time (or index) array for the original waypoints
        cs_x = CubicSpline(t, x_points)  # Spline for x-coordinates
        cs_y = CubicSpline(t, y_points)  # Spline for y-coordinates
        
        # Generate new points for the smoothed path
        t_new = np.linspace(0, len(x_points) - 1, num_points)
        smooth_x = cs_x(t_new)
        smooth_y = cs_y(t_new)
        
        # Reconstruct the smoothed path as nav_msgs/Path
        smoothed_path = Path()
        smoothed_path.header = input_path.header  # Copy over the header
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.pose.position.x = smooth_x[i]
            pose.pose.position.y = smooth_y[i]
            pose.pose.position.z = 0.0  # Assuming 2D navigation; set z to 0
            
            # Optionally set orientation (you could calculate it based on the path direction)
            pose.pose.orientation.w = 1.0  # Default orientation (unit quaternion)
            
            smoothed_path.poses.append(pose)
        
        return smoothed_path

    def compute_cmd_velocities(self, current_pose, smoothed_path: Path, lookahead_distance: float = 0.5) -> Twist:
        cmd_vel = Twist()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        
        # Extract the current yaw (orientation) from the quaternion
        orientation_q = current_pose.pose.orientation
        _, _, current_yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Find the closest point on the path (for cross-track error calculation)
        closest_point = None
        closest_distance = float('inf')
        closest_index = 0

        for i, pose in enumerate(smoothed_path.poses):
            waypoint_x = pose.pose.position.x
            waypoint_y = pose.pose.position.y
            
            # Calculate distance from current position to the path point
            distance = math.hypot(waypoint_x - current_x, waypoint_y - current_y)
            
            # Update closest point
            if distance < closest_distance:
                closest_distance = distance
                closest_point = pose
                closest_index = i

        if closest_point is None:
            return cmd_vel  # Return zero velocities if no path point is found

        # Compute cross-track error (perpendicular distance to the path)
        cross_track_error = closest_distance

        # Compute the heading to the target point (the point we are tracking)
        target_x = closest_point.pose.position.x
        target_y = closest_point.pose.position.y
        path_yaw = math.atan2(target_y - current_y, target_x - current_x)

        # Heading error (difference between current heading and path heading)
        heading_error = path_yaw - current_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Stanley control law: Steering = heading error + atan(k * cross-track error / velocity)
        velocity = 0.5  # Assume a constant velocity (can be dynamically adjusted)
        steering_angle = heading_error + math.atan2(self.k * cross_track_error, velocity)

        # Limit the steering angle to a maximum value
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

        # Set linear and angular velocities based on the Stanley controller
        cmd_vel.linear.x = target_x
        cmd_vel.linear.y = target_y
        cmd_vel.linear.z = closest_point.pose.position.z
        cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        return cmd_vel

    def update_current_pose(self, cmd_vel: Twist):
        # Assuming a simple motion model for the robot
        dt = 0.1  # Time interval (e.g., 100 ms)
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Calculate current orientation (yaw) from quaternion
        orientation_q = self.current_pose.pose.orientation
        _, _, current_theta = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Update position based on cmd_vel
        current_x += cmd_vel.linear.x * math.cos(current_theta) * dt
        current_y += cmd_vel.linear.x * math.sin(current_theta) * dt

        # Update orientation (yaw) based on cmd_vel
        current_theta += cmd_vel.angular.z * dt
        
        # Normalise yaw to be within [-pi, pi]
        current_theta = (current_theta + math.pi) % (2 * math.pi) - math.pi

        # Create updated pose
        self.current_pose.pose.position.x = current_x
        self.current_pose.pose.position.y = current_y
        self.current_pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, current_theta)

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        # Calculate the quaternion components
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Compute the quaternion
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(x=x, y=y, z=z, w=w)

    def check_goal_reached(self):
        if not self.smoothed_path:
            return  # No path to follow

        # Get the last waypoint in the smoothed path
        last_waypoint = self.smoothed_path.poses[-1]  # Last waypoint

        # Calculate distance to the last waypoint
        last_x = last_waypoint.pose.position.x
        last_y = last_waypoint.pose.position.y
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        distance_to_goal = math.hypot(last_x - current_x, last_y - current_y)
        self.get_logger().info(f"Distance from current position to goal: {distance_to_goal}")

        # Check if the robot has reached the last waypoint (goal)
        if distance_to_goal < 0.1:  # Example threshold (10 cm)
            self.goal_achieved = True
            self.smoothed_path = None
            self.get_logger().info("Reached the goal.")

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