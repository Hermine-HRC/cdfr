# herminebot_navigation

This package contains the navigation stack used by the robot.

Usage:

 ```ros2 launch herminebot_navigation navigation.launch.py```

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
