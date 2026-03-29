# Enables MAVLink communication with PX4/ArduPilot flight controllers
# Dockerfile for MAVROS and PX4 Integration with ROS 2 Jazzy (Ubuntu 24.04 Noble)
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-geographic-msgs \
    udev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

WORKDIR /root/ros2_ws
RUN mkdir -p /root/ros2_ws/src

RUN echo 'KERNEL=="ttyACM*", MODE="0666"' > /etc/udev/rules.d/99-px4.rules && \
    echo 'KERNEL=="ttyUSB*", MODE="0666"' >> /etc/udev/rules.d/99-px4.rules

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]
