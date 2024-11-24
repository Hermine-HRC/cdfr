# herminebot_gazebo

This package contains the necessary to simulate the environment of the theme 
*The Show Must On* in Gazebo.

To test on Gazebo run once in your terminal:

```source /usr/share/gazebo/setup.sh```

And then:

```gazebo the_show_must_go_on.world```

To test the world from a launch file: 

```ros2 launch herminebot_gazebo world.launch.py```

Note:

If you are not using the launch file you will have to export `GAZEBO_MODEL_PATH`

```export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/directory/models```

# Spawn the herminebot

```ros2 launch herminebot_gazebo spawn_herminebot.launch.py```

# Worlds built

The worlds are directly built in the install directory using embedded ruby with the build rules from the template `world.erb`.

For test built, execute:

```bash
erb <path/to/world/dir>/world.erb > test.world
```

Three worlds are generated:
* World with all game elements and 6 beacons
* World with all game elements and the blue beacons
* World with all game elements and the yellow beacons
