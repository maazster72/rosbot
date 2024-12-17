# vms_controller_interface

The `vms_controller_interface` is a ROS 2 Python package designed to receive route points and calculate the necessary velocities for the robot to follow a given path. It then sends the computed velocity commands to control the robot's movement.

This package integrates seamlessly with other components in a robotic system, enabling precise path following behavior based on received route points.

## Features
- Receives a sequence of route points (latitude, longitude, altitude, etc.).
- Calculates the velocities (`vx`, `vy`, `wz`) required for the robot to follow the route.
- Sends velocity commands to the robot's controller for real-time movement.