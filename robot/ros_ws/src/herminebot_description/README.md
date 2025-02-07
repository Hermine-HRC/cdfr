# herminebot_description

This package contains the urdf description of the robot.

# ServicesCommonServer

This node aims to provide a common server for services that are shared among the nodes.

## Services provided

* `get_robot_pose`

# OmnibotCmdConverter

This node aims to convert the Twist message to a set of wheel velocities and convert joint states to Twist 
for three-wheeled robots.

## Parameters

|              Parameter              |  Type  |                         Default value                          | Unit |                                                                  Description                                                                  |
|:-----------------------------------:|:------:|:--------------------------------------------------------------:|:----:|:---------------------------------------------------------------------------------------------------------------------------------------------:|
|           `cmd_in_topic`            | string |                           "/cmd_vel"                           | N/A  |                                                    Topic name for the input Twist message                                                     |
|          `cmd_out_topics`           | array  |  ["/wheel_cmd/front", "/wheel_cmd/left", "/wheel_cmd/right"]   | N/A  | Topic names for the output wheels velocities. The order is important, the first should be along x-axis positive and then the one on the left. |
|        `wheels_twist_topic`         | string |                        "/wheels_twist"                         | N/A  |                                                    Topic name for the output Twist message                                                    |
|        `joint_states_topic`         | string |                        "/joint_states"                         | N/A  |                                                     Topic name for the input joint states                                                     |
|           `wheel_joints`            | array  | ["front_wheel_joint", "left_wheel_joint", "right_wheel_joint"] | N/A  |          Names of the wheel joints. The order is important, the first should be along x-axis positive and then the one on the left.           |
| `wheels_velocity_covariance_matrix` | array  |                      Array full of zeros                       |  SI  |                   Covariance matrix of the wheel velocities (vx, vy, vz, vroll, vpitch, vyaw). The total length must be 36.                   |
|           `wheel_radius`            | float  |                              0.1                               |  m   |                                                             Radius of the wheels                                                              |
|           `robot_radius`            | float  |                              0.5                               |  m   |                                                              Radius of the robot                                                              |
