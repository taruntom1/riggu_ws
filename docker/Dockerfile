# syntax=docker/dockerfile:1
# riggu dev/sim/CI image (amd64). See docker/README.md.
#   base: Jazzy desktop-full + every ROS dependency of the workspace + DriveLink
#   dev:  tools and a non-root user matching the host UID/GID
# Keep the apt list in sync with the package.xml files; CI runs rosdep to catch drift.

FROM osrf/ros:jazzy-desktop-full AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# DriveLink apt repository; the signing key is pinned by fingerprint.
ARG DRIVELINK_KEY_URL=https://taruntom1.github.io/drivelink_v2/drivelink-archive-keyring.asc
ARG DRIVELINK_KEY_FPR=0B7ABAC1B5A995FB3512E0686964AFD8333BB3D6
RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
 && install -d -m 0755 /etc/apt/keyrings \
 && curl -fsSLo /etc/apt/keyrings/drivelink-archive-keyring.asc "$DRIVELINK_KEY_URL" \
 && gpg --show-keys --with-colons /etc/apt/keyrings/drivelink-archive-keyring.asc \
      | awk -F: '$1=="fpr"{print $10}' | grep -qx "$DRIVELINK_KEY_FPR" \
 && . /etc/os-release \
 && printf 'Types: deb\nURIs: https://taruntom1.github.io/drivelink_v2/\nSuites: %s\nComponents: main\nSigned-By: /etc/apt/keyrings/drivelink-archive-keyring.asc\n' \
      "$VERSION_CODENAME" > /etc/apt/sources.list.d/drivelink.sources \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-xacro \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-robot-localization \
      ros-jazzy-twist-mux \
      ros-jazzy-joy \
      ros-jazzy-teleop-twist-joy \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rplidar-ros \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-drivelink-bringup \
 && rm -rf /var/lib/apt/lists/*

FROM base AS dev
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=riggu
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo bash-completion less nano vim git openssh-client \
      gdb ccache shellcheck mesa-utils vulkan-tools iproute2 iputils-ping \
      python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Replace the image's default 'ubuntu' user (UID 1000) with one matching the host.
RUN if id ubuntu &>/dev/null; then userdel -r ubuntu; fi \
 && if ! getent group "$USER_GID" >/dev/null; then groupadd --gid "$USER_GID" "$USERNAME"; fi \
 && useradd --uid "$USER_UID" --gid "$USER_GID" --create-home --shell /bin/bash "$USERNAME" \
 && for g in video render dialout input; do \
      if getent group "$g" >/dev/null; then usermod -aG "$g" "$USERNAME"; fi; done \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME" \
 && chmod 0440 "/etc/sudoers.d/$USERNAME"

USER $USERNAME
RUN rosdep update --rosdistro jazzy \
 && cat >> "$HOME/.bashrc" <<'EOF'
# Jazzy + workspace + zenoh profile (sim unless RIGGU_PROFILE says otherwise).
if [[ -f /ws/scripts/env.sh ]]; then source /ws/scripts/env.sh "${RIGGU_PROFILE:-sim}"; else source /opt/ros/jazzy/setup.bash; fi
[[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
EOF
WORKDIR /ws
CMD ["bash"]
