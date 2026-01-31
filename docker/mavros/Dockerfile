# Dockerfile for MAVROS and PX4 Integration with ROS2 Humble
# Enables MAVLink communication with PX4/ArduPilot flight controllers
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install MAVROS and dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-geographic-msgs \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required by MAVROS)
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p /root/ros2_ws/src

# Set up udev rules for USB serial port (Pixhawk/PX4)
RUN echo 'KERNEL=="ttyACM*", MODE="0666"' > /etc/udev/rules.d/99-px4.rules && \
    echo 'KERNEL=="ttyUSB*", MODE="0666"' >> /etc/udev/rules.d/99-px4.rules

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /root/ros2_ws

# Default command
CMD ["/bin/bash"]
