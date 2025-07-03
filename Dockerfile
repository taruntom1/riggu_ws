FROM ros:kilted-ros-base-noble

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
        git \
        build-essential \
        libgl1-mesa-dev \
        qt6-base-dev \
        libqt6serialport6 \
        qt6-serialport-dev && \
        ccache && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache


# Set up workspace
WORKDIR /ros2_ws

# Copy source code
COPY ./src /ros2_ws/src

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build
