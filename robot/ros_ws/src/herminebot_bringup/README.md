# herminebot_bringup

This package allows to launch the different packages from one launch file.

# Run real robot

## With nav2

```ros2 launch herminebot_bringup real.launch.py```

## Without nav2

```ros2 launch herminebot_bringup real.launch.py use_nav2:=False```

## With team selection

```ros2 launch herminebot_bringup real.launch.py world_color:=blue```

# Run simulation 

## With nav2

```ros2 launch herminebot_bringup simulation.launch.py```

## Without nav2

```ros2 launch herminebot_bringup simulation.launch.py use_nav2:=False```

## With team selection

```ros2 launch herminebot_bringup simulation.launch.py world_color:=blue```

## Spawn the herminebot at a coordinate

```ros2 launch herminebot_bringup simulation.launch.py x:=-1.2 y:=-0.8 yaw:=1.57```

And in a second terminal for nav2 (supposing *ros_functions.sh* has been sourced):

```set_pose -1.2 -0.8 1.57```
