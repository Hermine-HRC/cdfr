#!/usr/bin/env bash

# Download and install ROS2 Jazzy and all the necessary for using the ROS2 part of the cdfr project

# Available arguments:
# --no-cleanup: do not clean up after the installation (mostly for GitHub Actions for saving time)

no_cleanup=0
for arg in "$@" # Loop through arguments
do
    case "$arg" in
    --no-cleanup)
        no_cleanup=1
        ;;
    esac
done

if [ ! $(cat /etc/os-release | grep "VERSION_ID=\"24.04\"") ]
then
    echo "This script is only to be used on Ubuntu 24.04 (Noble Numbat) for installing ROS2 Jazzy"
    exit 1
fi

if [ ! $(locale | grep "LANG=en_US.UTF-8") ]; # If UTF-8 is not en_US
then
    echo "========== Setting UTF-8 to en_US =========="
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "========== UTF-8 set to en_US =========="
else
    echo "========== UTF-8 already OK =========="
fi

# ROS2 installation with project dependencies
echo "========== Installing ROS2 Jazzy =========="
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-dev-tools
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base
echo "========== Installing project dependencies and Gazebo =========="
sudo apt install -y python3-rosdep python3-pytest-rerunfailures
sudo apt install -y ruby lcov libunwind-dev python3-pip
sudo apt install -y \
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-tf-transformations \
    ros-jazzy-ros-gz # Gazebo
sudo apt install -y python3-bleak # Python deps
echo "========== ROS2 Jazzy and Gazebo installed =========="
   
if [[ ! $(cat ~/.bashrc | grep "source /opt/ros/jazzy/setup.bash") ]]; # If ROS is not already sourced in the .bashrc
then
    echo "========== ROS2 sourcing append to the .bashrc =========="
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

if [ $no_cleanup -eq 0 ]
then
    echo "========== Cleaning up =========="
    sudo apt autoremove -y
    echo "========== Cleaned up =========="
fi

echo "========== Installation finished =========="
echo "Source your .bashrc file to enjoy ROS2 on this terminal session"
