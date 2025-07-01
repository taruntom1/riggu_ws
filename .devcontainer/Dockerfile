FROM osrf/ros:kilted-desktop-full

# Add ubuntu user with same UID and GID as your host system
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

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

# Prepend sudo where root is needed
# Add user to dialout and video groups, update, upgrade, install git and Qt dependencies
RUN sudo usermod --append --groups dialout,video $USERNAME && \
    sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
        git \
        build-essential \
        libgl1-mesa-dev \
        qt6-base-dev \
        libqt6serialport6 \
        qt6-serialport-dev \
        ntp \
        ntpdate && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# ROS dependencies
RUN rosdep update

# Source ROS setup in future shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# Set zenoh as the default rmw
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
