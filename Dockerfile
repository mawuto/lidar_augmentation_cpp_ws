# ============================================================
#  Base: ROS Noetic on Ubuntu 20.04 (official OSRF image)
# ============================================================
FROM osrf/ros:noetic-desktop-full

# ── Labels ──────────────────────────────────────────────────
LABEL maintainer="your.email@example.com"
LABEL description="LiDAR Augmentation package — ROS Noetic (Ubuntu 20.04)"
LABEL version="1.0.0"

# ── Avoid interactive prompts during apt ─────────────────────
ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    # Package dependencies (matching your package.xml)
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-message-filters \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-roslib \
    ros-noetic-rosbag \
    ros-noetic-topic-tools \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-reconfigure \
    # System libs (matching your CMakeLists.txt)
    libpcl-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    # Python tools (your scripts/ folder)
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-yaml \
    python3-opencv \
    python3-pip \
    # Testing
    ros-noetic-rostest \
    && rm -rf /var/lib/apt/lists/*

# ── Create catkin workspace ───────────────────────────────────
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# ── Copy the package into the workspace ──────────────────────
COPY src/lidar_augmentation /catkin_ws/src/lidar_augmentation

# ── Build ────────────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CXX_FLAGS='-O3 -march=native'"

# ── Source workspace on every bash session ───────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# ── Entrypoint: source workspace then run command ────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
