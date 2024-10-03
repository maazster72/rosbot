import math
from geometry_msgs.msg import PoseStamped, Twist, Quaternion

def get_distance_to_target_pose(current_pose, target_pose):
    return math.hypot(
        target_pose.pose.position.x - current_pose.pose.position.x,
        target_pose.pose.position.y - current_pose.pose.position.y,
        target_pose.pose.position.z - current_pose.pose.position.z
        )

def poseToLists(pose):
    position = pose.pose.position
    orientation = pose.pose.orientation
    position_list = [position.x, position.y, position.y]
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

    return position_list, orientation_list

def update_current_pose(current_pose, cmd_vel):
    # Assuming a simple motion model for the robot
    dt = 0.1  # Time interval (e.g., 100 ms)
    current_x = current_pose.pose.position.x
    current_y = current_pose.pose.position.y
    current_z = current_pose.pose.position.z

    # Calculate current orientation (yaw) from quaternion
    orientation_q = current_pose.pose.orientation
    _, _, current_theta = quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    # Update position based on cmd_vel
    current_x += cmd_vel.linear.x * math.cos(current_theta) * dt
    current_y += cmd_vel.linear.x * math.sin(current_theta) * dt

    # Update orientation (yaw) based on cmd_vel
    current_theta += cmd_vel.angular.z * dt

    # Normalise yaw to be within [-pi, pi]
    current_theta = (current_theta + math.pi) % (2 * math.pi) - math.pi

    # Create updated pose
    current_pose.pose.position.x = current_x
    current_pose.pose.position.y = current_y
    current_pose.pose.position.z = current_z
    current_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, current_theta)

    return current_pose

def quaternion_to_euler(x, y, z, w):
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

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
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