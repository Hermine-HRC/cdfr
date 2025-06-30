# scripts

This folder contains useful scripts that can be used to facilitate some actions.

# Scripts to source

Some scripts contain shell functions so they need to be sourced to use these functions.

Here is the list of file to source:

* ros_functions.sh

# ros2_project_install.sh

This script can be used to install ROS2 and all the necessary dependencies to be used for this project.
It can be launched from anywhere without affecting the destination of installed files.

Before executing this script, you have to make the script executable.

```
chmod +x ros2_project_install.sh
```

Then you can launch the script with root privilges to avoid a repeat of sudo passwords prompts.

```
sudo ./ros2_project_install.sh
```