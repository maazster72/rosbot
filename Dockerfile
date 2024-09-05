# Use the Jetson L4T base image compatible with Ubuntu 20.04
FROM nvcr.io/nvidia/l4t-base:r35.1.0

# Set the environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=galactic
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GIT_REPO=https://github.com/maazster72/rosbot.git 

# Install basic utilities, build tools, ROS 2, and other necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        gnupg2 \
        lsb-release \
        curl \
        git \
        sudo \
        python3-pip \
        build-essential \
        x11-apps \
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        mesa-utils \
        gedit && \
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - && \
    echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-$ROS_DISTRO-desktop \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-gazebo-ros-pkgs\
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-joint-state-publisher \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*
    
# Create the runtime directory
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Install colcon and its extensions via pip
RUN pip3 install -U \
    colcon-common-extensions \
    colcon-ros-bundle

# Source the ROS 2 setup script by default
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Set up a workspace
WORKDIR /workspace
RUN mkdir -p src

# Clone the repository and copy the contents of the 'src' directory
RUN git clone $GIT_REPO /tmp/rosbot && \
    cp -r /tmp/rosbot/src/* /workspace/src/ && \
    rm -rf /tmp/rosbot

# Build the workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

# Source the workspace's setup.bash
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to automatically source ROS 2, the workspace environment, and enter a bash shell
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /workspace/install/setup.bash && exec bash"]

