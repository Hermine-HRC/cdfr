# herminebot_navigation

This package contains the navigation stack used by the robot.

Usage:

 ```ros2 launch herminebot_navigation navigation.launch.py```

# Modified plugins

## Regulated Pure Pursuit Controller

This controller provided by nav2 is modified to allow the robot to first move forward and backward **and** to rotate
to the expected final heading at the same time.

See [the official documentation](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) for more
details about the base of the controller.

### Plugin

`hrc_rpp_controller::RegulatedPurePursuitController`

### New configuration

|           Parameter           |  Type  | Default value | Unit |                                                           Description                                                            |
|:-----------------------------:|:------:|:-------------:|:----:|:--------------------------------------------------------------------------------------------------------------------------------:|
| `max_rotation_before_reverse` | double |     1.58      | rad  | Maximum absolute angle where the robot wil rotate to align the path. Beyond this value, the robot will follow the path backward. |
