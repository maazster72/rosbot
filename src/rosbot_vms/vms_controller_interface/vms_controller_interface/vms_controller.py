import numpy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
import vms_controller_interface.vms_controller_util as util

def compute_linear_velocity(current_position, target_position, K_v=1.25):
        delta_position = numpy.array(target_position) - numpy.array(current_position)
        distance = numpy.linalg.norm(delta_position)
        direction = delta_position / distance

        # If the distance is non-zero
        if abs(distance) > 0.375:
            v = K_v * direction * distance
        else:
            v = numpy.zeros(3)

        return v

def compute_angular_velocity(current_orientation, target_orientation, K_omega=1.0):
    # Convert quaternions to rotation objects
    rotation_current = Rotation.from_quat(current_orientation)
    rotation_target = Rotation.from_quat(target_orientation)

    # Compute relative rotation
    relative_rotation = rotation_target * rotation_current.inv()

    # Get axis-angle from the relative rotation
    axis, angle = quaternion_to_axis_angle(relative_rotation.as_quat())

    # Compute angular velocity
    if angle > 0.5:     # If there is a non-zero angular difference
        omega = K_omega * numpy.array(axis) * angle
    else:
        omega = numpy.zeros(3)  # No rotation needed
    
    return omega

def compute_yaw_from_orientation(quaternion):
    """Extract yaw (rotation about z-axis) from a quaternion."""
    rotation = Rotation.from_quat(quaternion)
    euler = rotation.as_euler('xyz', degrees=False)
    return euler[2]  # Yaw is the third element (rotation about the z-axis)

def compute_required_yaw_rotation(current_pose, target_pose):
    current_position, current_orientation = util.poseToLists(current_pose)
    target_position, _ = util.poseToLists(target_pose)

    # Compute the direction vector to the target
    delta_position = numpy.array(target_position) - numpy.array(current_position)
    target_yaw = numpy.arctan2(delta_position[1], delta_position[0])  # Angle to face target in x-y plane

    # Extract the current yaw from the quaternion
    current_yaw = compute_yaw_from_orientation(current_orientation)

    # Compute the angular difference (yaw rotation needed)
    yaw_diff = target_yaw - current_yaw

    # Normalise the yaw difference to the range [-pi, pi]
    yaw_diff = (yaw_diff + numpy.pi) % (2 * numpy.pi) - numpy.pi

    return yaw_diff  # This is the required yaw rotation (in radians)

def quaternion_to_axis_angle(quaternion):
    qx, qy, qz, qw = quaternion
    angle = 2 * numpy.arccos(qw)
    s = numpy.sqrt(1 - qw**2)

    if s < 1e-6:
        x, y, z = 1, 0, 0
    else:
        x = qx / s
        y = qy / s
        z = qz / s

    return (x, y , z), angle

def orient_to_target(current_pose, target_pose):
    cmd_vel = Twist()

    cmd_vel.angular.z = compute_required_yaw_rotation(
        current_pose,
        target_pose
        )

    return cmd_vel

def move_to_target(current_pose, target_pose):
    current_position, current_orientation = util.poseToLists(current_pose)
    target_position, target_orientation = util.poseToLists(target_pose)

    cmd_vel = Twist()

    linear_velocity = compute_linear_velocity(current_position, target_position)
    angular_velocity = compute_angular_velocity(current_orientation, target_orientation)

    cmd_vel.linear.x = linear_velocity[0]
    cmd_vel.linear.y = linear_velocity[1]
    cmd_vel.linear.z = linear_velocity[2]

    # cmd_vel.angular.x = angular_velocity[0]
    # cmd_vel.angular.y = angular_velocity[1]
    # cmd_vel.angular.z = angular_velocity[2]

    return cmd_vel
