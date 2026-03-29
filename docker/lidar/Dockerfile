# Following Point-LIO ROS2 setup: https://github.com/dfloreaa/point_lio_ros2
# Dockerfile for Unitree L1 4D LiDAR with ROS 2 Jazzy (Ubuntu 24.04 Noble)
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    git \
    wget \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-visualization-msgs \
    ros-jazzy-rviz2 \
    libpcl-dev \
    libeigen3-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws
RUN mkdir -p /root/ros2_ws/src

# Livox SDK2
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && \
    make install

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
    cd ws_livox/src/livox_ros_driver2 && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && ./build.sh jazzy"

# Unitree L1 SDK
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/unitreerobotics/unilidar_sdk.git

WORKDIR /root/ros2_ws/src/unilidar_sdk/unitree_lidar_sdk
RUN mkdir -p build && cd build && \
    cmake .. && make -j$(nproc)

# Point-LIO ROS 2
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/dfloreaa/point_lio_ros2.git

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    source /root/ws_livox/install/setup.bash && \
    colcon build --symlink-install \
    --packages-select unitree_lidar_ros2 point_lio"

RUN echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/99-unitree-lidar.rules

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_livox/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]

