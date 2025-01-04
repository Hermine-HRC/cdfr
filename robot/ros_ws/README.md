[![ROS2 test packages](https://github.com/Hermine-HRC/cdfr/actions/workflows/ros2_build.yml/badge.svg)](https://github.com/Hermine-HRC/cdfr/actions/workflows/ros2_build.yml)

Code ROS de l'herminebot

- [Versions](#versions)
- [Herminebot model](#herminebot-model)
  - [Accepted values](#accepted-values)
  - [Default value](#default-value)
  - [Command line usage](#command-line-usage)
- [ROS dependencies](#ros-dependencies)
- [Build dependencies](#build-dependencies)

# Versions

- `ROS2 Humble`
- `Ubuntu 22.04`

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

# Build dependencies

```bash
sudo apt-get install -y ruby
```

# Python dependencies

```bash
pip3 install -r requirements.txt
```
