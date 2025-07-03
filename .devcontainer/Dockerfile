FROM osrf/ros:kilted-desktop-full

# User setup arguments
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user, install sudo, and configure permissions in a single layer
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME

# Install system dependencies and development tools in a single optimized layer
RUN sudo usermod --append --groups dialout,video $USERNAME && \
    sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
        # Version control and build essentials
        git \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        # Graphics and Qt dependencies
        libgl1-mesa-dev \
        qt6-base-dev \
        libqt6serialport6 \
        qt6-serialport-dev \
        # Time synchronization
        ntp \
        ntpdate \
        # Development and debugging tools
        gdb \
        gdbserver \
        ccache \
        clang \
        clang-format \
        valgrind \
        # Testing and coverage tools
        lcov \
        gcovr \
        cppcheck && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# ROS dependencies and shell configuration
RUN rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source install/setup.bash" >> ~/.bashrc

# Set environment variables for ROS and compilation optimization
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache
