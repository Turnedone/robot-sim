#!/usr/bin/env bash
# Installs ROS 2 Jazzy, Gazebo Harmonic (via ros_gz), Nav2 and the TurtleBot3
# simulation on Ubuntu 24.04 (for example in WSL2 on Windows 11).
# Follows the official ROS 2 Jazzy installation guide.
#
# Run as root, e.g. from Windows:
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/path/to/robot-sim/tools/wsl/setup-ros2-gazebo.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "== locale and base tools"
apt-get update
apt-get install -y locales software-properties-common curl
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

echo "== ROS 2 apt source (ros2-apt-source package)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
              grep -F '"tag_name"' | awk -F'"' '{print $4}')
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
    dpkg -i /tmp/ros2-apt-source.deb
fi
apt-get update
apt-get upgrade -y

echo "== ROS 2 Jazzy desktop, dev tools, Gazebo Harmonic bridge, Nav2, TurtleBot3 simulation"
apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-teleop-twist-keyboard

echo "== rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

echo "== done"
set +u  # ROS's setup script uses unset variables
. /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
gz sim --version 2>/dev/null | head -1 || true
