# herminebot_navigation

This package contains the navigation stack used by the robot.

Usage:

 ```ros2 launch herminebot_navigation navigation.launch.py```

# New nodes

## Map Modifier

This node is the server for adding or removing obstacle objects in the map through a 
[keepout filter](https://docs.nav2.org/configuration/packages/costmap-plugins/keepout_filter.html) mask.

### Parameters

|      Parameter       |      Type       |  Default value   | Unit  |                    Description                    |
|:--------------------:|:---------------:|:----------------:|:-----:|:-------------------------------------------------:|
| `mask_filter_topic`  |     string      |  "/mask_filter"  |  N/A  |     The topic where the map must be published     |
| `initial_mask_topic` |     string      | "/elements_mask" |  N/A  | The topic where the initial mask is subscribed to |

# Modified plugins

## Regulated Pure Pursuit Controller

This controller provided by nav2 is modified to allow the robot to first move forward and backward **and** to rotate
to the expected final heading at the same time. This plugin also implement a choice to move forward or backward 
depending on the final angle so the final rotation is minimized.

See [the official documentation](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) for more
details about the base of the controller.

### Plugin

`hrc_rpp_controller::RegulatedPurePursuitController`

### New parameters

|          Parameter           |  Type  | Default value | Unit  |                                     Description                                      |
|:----------------------------:|:------:|:-------------:|:-----:|:------------------------------------------------------------------------------------:|
| `rotate_to_path_angular_vel` | double |      1.8      | rad/s | The rotation velocity at which the robot should rotate for reaching the path heading |

### Usage modified parameters

* `rotate_to_heading_angular_vel`: This parameter is now used only when rotating to heading 
and is not used for rotation to path heading.

## Costmap Obstacle Layer

This costmap layer provided by nav2 is modified to allow frequent update of obstacles 
and to increase the inflation around the obstacles.

See [the official documentation](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
for more details about the base of the layer.

### Plugin

`hrc_costmap_2d::ObstacleLayer`

### New parameters

|       Parameter       |     Type     | Default value | Unit  |                         Description                          |
|:---------------------:|:------------:|:-------------:|:-----:|:------------------------------------------------------------:|
| `cost_scaling_factor` |    double    |      1.0      | meter |     The exponential decay factor across inflation radius     |
|  `inflation_radius`   |    double    |      0.0      | meter |         The radius to inflate around the obstacles.          |
|       `polygon`       | double array |  empty array  | meter | The polygon where measured points must be in to be inflated. |

## Costmap Keepout Filter

This costmap filter provided by nav2 is modified to add an inflation radius around keepout areas.

See the [official documentation](https://docs.nav2.org/configuration/packages/costmap-plugins/keepout_filter.html)
for more details about the base of the filter.

### Plugin

`hrc_costmap_2d::KeepoutFilter`

### New parameters

|     Parameter      |     Type     | Default value | Unit  |                    Description                    |
|:------------------:|:------------:|:-------------:|:-----:|:-------------------------------------------------:|
| `inflation_radius` |    double    |      0.1      | meter | The radius to inflate around the robot footprint. |

# New plugins

## Regulated Rotation Controller

This controller aims to rotate the robot to the expected final heading using a PID controller.
It has to be used with a second controller that will be used to move the robot. This controller can only
be used by omnidirectional robots because the second controller has to be able to move the robot in any direction as
the controller will be used to rotate the robot to the final heading.

### Plugin

`hrc_regulated_rotation_controller::RegulatedRotationController`

### Parameters

|      Parameter       |  Type  | Default value | Unit  |                               Description                                |
|:--------------------:|:------:|:-------------:|:-----:|:------------------------------------------------------------------------:|
| `primary_controller` | string |      ""       |  N/A  | Internal controller plugin to use for actual control behavior for moving |
|       `p_gain`       | double |      0.2      |  N/A  |                            Proportional gain                             |
|       `i_gain`       | double |     0.02      |  N/A  |                              Integral gain                               |
|       `d_gain`       | double |      2.0      |  N/A  |                             Derivative gain                              |
|  `max_rotation_vel`  | double |      1.5      | rad/s |            The maximum rotation velocity the robot can reach             |
