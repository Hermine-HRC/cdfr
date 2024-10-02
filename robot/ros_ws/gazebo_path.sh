# File to source from 'ros_ws' once by terminal to enable personal models in Gazebo

source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/robot_gazebo/models/
