# herminebot_hardware

This package contains the interface between the hardware components of the robot and the ROS system.

# Usage

It is made to be used with a physical robot contrary to *herminebot_gazebo* which is made to be used with a simulation
environment.

```bash
ros2 launch herminebot_hardware hardware.launch.py
```

# i2c_interface

The I2CInterface node interfaces ROS with the I2C components.

## Configuration

|              Parameter               |      Type      |   Default value   | Unit  |                              Description                               |
|:------------------------------------:|:--------------:|:-----------------:|:-----:|:----------------------------------------------------------------------:|
|            `wheel_radius`            |     float      |        0.0        | meter |                    Radius of the herminebot wheels                     |
|          `wheel_separation`          |     float      |        0.0        | meter |                Separation between the herminebot wheels                |
|           `coder_adresses`           |  array of int  |    empty array    |  N/A  | The I2C address of the coders (right and left). The size **must be 2** |
|       `coder_subsciption_rate`       |     float      |       1000        |  Hz   |                 The rate at which the coders are read                  |
|        `coder_message_ratio`         |     float      |        10         |  N/A  |   The ratio between the number of messages values and the real value   |
|           `odometry_topic`           |     string     | "/wheel/odometry" |  N/A  |              The topic on which the odometry is published              |
|         `odometry_frame_id`          |     string     |      "odom"       |  N/A  |                      The frame id of the odometry                      |
|      `odometry_frame_child_id`       |     string     | "base_footprint"  |  N/A  |                     The frame id of the herminebot                     |
|     `odometry_covariance_matrix`     | array of float |    empty array    |  N/A  |      The covariance matrix of the odometry (the size must be 36)       |
|           `cmd_vel_topic`            |     string     |    "/cmd_vel"     |  N/A  |         The topic on which the velocity commands are received          |
|           `i2c_frequency`            |      int       |      100000       |  Hz   |      The I2C frequency. Values must be 100000, 400000 or 1000000       |
|             `imu_topic`              |     string     |    "/imu/data"    |  N/A  |                     The topic to send the IMU data                     |
|       `imu_subscription_rate`        |     float      |      1000.0       |  Hz   |                  The rate at which the data are fetch                  |
|          `imu_i2c_address`           |      int       |       0x68        |  N/A  |                       The I2C address of the IMU                       |
|            `imu_frame_id`            |     string     |    "imu_link"     |  N/A  |                        The frame id of the IMU                         |
| `imu_acceleration_covariance_matrix` | array of float |    empty array    |  N/A  |   The covariance matrix of the IMU acceleration (the size must be 9)   |
| `imu_orientation_covariance_matrix`  | array of float |    empty array    |  NA   |   The covariance matrix of the IMU orientation (the size must be 9)    |
