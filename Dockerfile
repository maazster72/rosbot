# Use the Jetson L4T base image compatible with Ubuntu 20.04
FROM nvcr.io/nvidia/l4t-base:r35.1.0

# Set the environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=galactic
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install basic utilities and set up the system
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        gnupg2 \
        lsb-release \
        curl \
        git \
        sudo \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*

# Install colcon and its extensions via pip
RUN pip3 install -U \
    colcon-common-extensions \
    colcon-ros-bundle

# Add the ROS 2 repository and its key
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - && \
    echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
        ros-galactic-desktop \
        ros-galactic-navigation2 \
        ros-galactic-nav2-bringup \
        ros-galactic-rmw-cyclonedds-cpp \
        && rm -rf /var/lib/apt/lists/*

# Install additional necessary packages
RUN apt-get update && apt-get install -y \
    x11-apps \
    gedit \
    && rm -rf /var/lib/apt/lists/*
    
# Source the ROS 2 setup script by default
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# Set up a workspace
WORKDIR /workspace
RUN mkdir -p src

# Clone the repository
RUN git clone https://github.com/maazster72/rosbot.git src/

# Build all the packages in the workspace
RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && colcon build"

# Source the workspace's setup.bash
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to automatically source ROS 2, the workspace environment, and enter a bash shell
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/galactic/setup.bash && source /workspace/install/setup.bash && exec bash"]

