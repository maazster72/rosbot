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
        self.poses = None
        self.current_pose = PoseStamped()
        self.current_target_pose_index = None
        self.current_target_pose = None
        self.distance_to_target = None
        self.goal_pose = None
        self.goal_achieved = True
        self.distance_to_goal = None

        self.threshold_linear = 0.375
        self.threshold_angular = 0.5

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

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for goal pose
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to control movement towards the next goal
        self.timer = self.create_timer(0.1, self.move_towards_goal)

    def plan_callback(self, msg: Path):
        self.plan = msg
        self.poses = self.plan.poses[1:]
        self.current_target_pose_index = 0
        self.current_target_pose = self.poses[self.current_target_pose_index]
        self.distance_to_target = util.get_distance_to_target_pose(self.current_pose, self.current_target_pose)

        self.goal_pose = self.poses.pop()
        self.goal_pose_publisher.publish(self.goal_pose)

        self.goal_achieved = False
        self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)
        
        # Logging
        self.get_logger().info(f"Received path: {self.plan}")
        self.get_logger().info(f"Initial pose: {self.current_pose}")
        self.get_logger().info(f"Goal Pose: {self.goal_pose}")
        self.get_logger().info(f"Poses to follow: {self.poses}")
        self.get_logger().info(f"Distance to goal pose: {self.distance_to_goal}")

    def odom_callback(self, msg: Odometry):
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation = msg.pose.pose.orientation

        # Logging
        # self.get_logger().info(f"Updated pose: {self.current_pose}")
    
    def move_towards_goal(self):
        cmd_vel = Twist()

        if self.plan:
            # Logging
            # self.get_logger().info(f"Current target pose ({self.current_target_pose_index + 1}/{len(self.poses) + 1}): {self.current_target_pose}")
            self.get_logger().info(f"Distance to target pose: {self.distance_to_target}")
            # self.get_logger().info(f"Distance to goal pose: {self.distance_to_goal}")
            # Check if goal has been reached
            if abs(self.distance_to_goal) < self.threshold_linear:
                self.plan = None
                self.poses = None
                self.current_pose = PoseStamped()
                self.current_target_pose_index = None
                self.current_target_pose = None
                self.distance_to_target = None
                self.goal_pose = None
                self.goal_achieved = True
                self.distance_to_goal = None
                # Logging
                self.get_logger().info(f"Goal pose reached. Executed plan successfully")
            # Check if target pose has been reached
            elif abs(self.distance_to_target) < self.threshold_linear:
                # Logging
                self.get_logger().info(f"Pose {self.current_target_pose_index + 1}/{len(self.poses) + 1} complete.")
                # Check if any more poses to follow
                if len(self.poses) > 0:
                    self.current_target_pose = self.poses.pop()
                    self.current_target_pose_index += 1
                    # Logging
                    self.get_logger().info(f"New target pose: {self.current_target_pose}")
            else:
                if self.check_if_rotate_needed(self.current_pose, self.current_target_pose):
                    cmd_vel = controller.orient_to_target(self.current_pose, self.current_target_pose)
                    # Logging
                    self.get_logger().info(f"Rotating to target...")
                else:
                    # Compute cmd_vel
                    cmd_vel = controller.move_to_target(self.current_pose, self.current_target_pose)
                    # Logging
                    self.get_logger().info(f"cmd_vel: {cmd_vel}")
            
                # Update distances
                self.distance_to_target = util.get_distance_to_target_pose(self.current_pose, self.current_target_pose)
                self.distance_to_goal = util.get_distance_to_target_pose(self.current_pose, self.goal_pose)

        self.cmd_vel_publisher.publish(cmd_vel)

    def compute_cmd_velocities(self, current_pose, target_pose):
        cmd_vel = Twist()

        # Get current positions and orientations as list
        current_position_list, current_orientation_list = self.poseToLists(current_pose) 
        # Logging
        self.get_logger().info(f"Current position : {current_position_list}")
        self.get_logger().info(f"Current orientation : {current_orientation_list}")
        
        # Get target position and orientation
        target_position_list, target_orientation_list = self.poseToLists(target_pose) 
        # Logging
        self.get_logger().info(f"Target position: {target_position_list}")
        self.get_logger().info(f"Target orientation: {target_orientation_list}")

        # Compute linear and angular velocity
        linear_velocity = controller.compute_linear_velocity(current_position_list, target_position_list)
        angular_velocity = controller.compute_angular_velocity(current_orientation_list, target_orientation_list)

        # Logging
        self.get_logger().info(f"Linear_velocity: {linear_velocity}")
        self.get_logger().info(f"Angular velocity: {angular_velocity}")

        # Assign cmd_vel
        cmd_vel.linear.x = linear_velocity[0]
        cmd_vel.linear.y = linear_velocity[1]
        cmd_vel.linear.z = linear_velocity[2]

        cmd_vel.angular.x = angular_velocity[0]
        cmd_vel.angular.y = angular_velocity[1]
        cmd_vel.angular.z = angular_velocity[2]

        # cmd_vel.angular = self.euler_to_quaternion(angular_velocity[0], angular_velocity[1], angular_velocity[2])
        
        # Logging
        self.get_logger().info(f"cmd_vel: {cmd_vel}")

        return cmd_vel

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

    def check_goal_reached(self):
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

    def check_target_pose_reached(self, current_pose, target_pose, threshold=1e-3):
        # Calculate distance to the target pose from the current pose
        distance_to_target = math.hypot(target_pose.pose.position.x - current_pose.pose.position.x, target_pose.pose.position.y - current_pose.pose.position.y, target_pose.pose.position.z - current_pose.pose.position.z)
        self.get_logger().info(f"Distance from current position to target: {distance_to_target}")

        # Check if the robot has reached the last target pose
        if distance_to_target < threshold:
            self.get_logger().info(f"Reached the target {target_pose} with threshold {threshold}")
            return True
        
        return False

    def check_if_rotate_needed(self, current_pose, target_pose):
        yaw_rotation = controller.compute_required_yaw_rotation(current_pose, target_pose)
        self.get_logger().info(f"Yaw rotation: {yaw_rotation}")
        if abs(yaw_rotation) > self.threshold_angular:
            return True
        
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