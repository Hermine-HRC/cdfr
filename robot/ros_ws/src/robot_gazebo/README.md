# robot_gazebo

This package contains the necessary to simulate the environment of the theme 
*The Show Must On* in Gazebo.

To test on Gazebo run once in your terminal:

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/directory/models
```

And then:

```gazebo the_show_must_go_on.world```

To test the world from a launch file: 

```ros2 launch robot_gazebo world.launch.py```
