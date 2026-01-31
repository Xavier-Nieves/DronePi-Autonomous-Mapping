# Dockerfile for Unitree L1 4D LiDAR with ROS2 Humble
# Following Point-LIO ROS2 setup: https://github.com/dfloreaa/point_lio_ros2
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies following Point-LIO requirements
RUN apt-get update && apt-get install -y \
    git \
    wget \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    ros-humble-rviz2 \
    libpcl-dev \
    libeigen3-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p /root/ros2_ws/src

# Clone Livox ROS Driver 2 (required dependency)
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && \
    make install

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
    cd ws_livox/src/livox_ros_driver2 && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && ./build.sh humble"

# Clone Unitree L1 LiDAR SDK
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/unitreerobotics/unilidar_sdk.git

# Build the C++ SDK first (needed by ROS2 wrapper)
WORKDIR /root/ros2_ws/src/unilidar_sdk/unitree_lidar_sdk
RUN mkdir -p build && cd build && \
    cmake .. && make -j$(nproc)

# Clone Point-LIO ROS2
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/dfloreaa/point_lio_ros2.git

# Build only the ROS2 packages (excluding ROS1)
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/ws_livox/install/setup.bash && \
    colcon build --symlink-install \
    --packages-select unitree_lidar_ros2 point_lio"

# Set up udev rules for USB serial port
RUN echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/99-unitree-lidar.rules

# Source ROS2 and workspaces in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_livox/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /root/ros2_ws

# Default command
CMD ["/bin/bash"]
