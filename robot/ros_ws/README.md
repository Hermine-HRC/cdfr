[![ROS2 test packages](https://github.com/Hermine-HRC/cdfr/actions/workflows/ros2_build.yml/badge.svg)](https://github.com/Hermine-HRC/cdfr/actions/workflows/ros2_build.yml)
[![codecov](https://codecov.io/github/Hermine-HRC/cdfr/graph/badge.svg?token=KYAK8502V8)](https://codecov.io/github/Hermine-HRC/cdfr)

Code ROS de l'herminebot

- [Versions](#versions)
- [Herminebot model](#herminebot-model)
  - [Accepted values](#accepted-values)
  - [Default value](#default-value)
  - [Command line usage](#command-line-usage)
- [ROS dependencies](#ros-dependencies)
- [Build dependencies](#build-dependencies)
- [Python dependencies](#python-dependencies)

# Versions

- `ROS2 Jazzy`
- `Ubuntu 24.04`

# Herminebot model

The choice of the herminebot model is done by setting the environment variable `HERMINEBOT_MODEL`.

## Accepted values

- `diff`

## Default value

`diff`

## Command line usage

```bash
export HERMINEBOT_MODEL=diff
```

# ROS dependencies

Installation:

```bash
sudo apt install \
ros-${ROS_DISTRO}-robot-localization \
ros-${ROS_DISTRO}-navigation2 \
ros-${ROS_DISTRO}-nav2-bringup \
ros-${ROS_DISTRO}-nav2-map-server \
ros-${ROS_DISTRO}-tf-transformations \
ros-${ROS_DISTRO}-ros-gz
```

Gazebo:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

# Build dependencies

```bash
sudo apt-get install -y ruby
```

# Python dependencies

```bash
pip3 install -r requirements.txt
```
