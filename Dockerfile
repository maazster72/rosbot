# Use the DustyNV ROS Humble base image for Jetson
FROM dustynv/ros:humble-ros-base-l4t-r34.1.1

# Set the environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 dependencies, Nav2 stack, RViz, Gazebo, and Gedit
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros-control \
    gedit \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install any other dependencies required by your project
# You can add additional RUN commands here to install other packages

# Source the ROS 2 setup script by default
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up a workspace (optional but recommended)
WORKDIR /workspace
RUN mkdir -p src

# Optionally clone and build your ROS 2 packages
# Example:
# RUN git clone https://github.com/maazster72/rosbot.git src/your_package
# RUN source /opt/ros/humble/setup.bash && colcon build

# Set the entrypoint to automatically source ROS 2 and enter a bash shell
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

