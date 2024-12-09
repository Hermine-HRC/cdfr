# herminebot_head

This package contains the necessary for decision-making and actions management.

# HeadNode

The HeadNode reads a json sequence file to know which actions must be realized and command the actions.

It needs to receive a *True* value on the topic defined by `start_actions_topic` 
to start the realization of the actions.

## Configuration

|           Parameter            | Type  |   Default value    |  Unit   |                                 Description                                 |
|:------------------------------:|:-----:|:------------------:|:-------:|:---------------------------------------------------------------------------:|
|    `action_manager_period`     | float |        0.5         | seconds |              Period at which the managing process is realized               |
|      `blue_sequence_file`      |  str  |    empty string    |   N/A   |                   Absolute path to the blue sequence file                   |
|       `global_frame_id`        |  str  |        map         |   N/A   |                                Global frame                                 |
|        `restart_topic`         |  str  |      /restart      |   N/A   |                   Topic to listen for restarting the node                   |
|  `sequence_default_filename`   |  str  |   demo_seq.json    |   N/A   |                            Default sequence file                            |
|     `start_actions_topic`      |  str  | /can_start_actions |   N/A   |                  Topic to listen for starting the actions                   |
|          `stop_time`           | float |        99.0        | seconds |                Time at which the herminebot must be stopped                 |
|         `team_colors`          | list  |     empty list     |   N/A   |           The colors that can be selected for the team selection            |
| `time_to_enable_laser_sensors` | float |        0.0         | seconds | Time at which to start the laser sensors. Set the value < 0 to never enable |
|         `use_sim_time`         | bool  |       False        |   N/A   |                Whether to use simulation time (Gazebo clock)                |
|     `yellow_sequence_file`     |  str  |    emtpy string    |   N/A   |                  Absolute path to the yellow sequence file                  |

# Sequence parameters

The sequence file is a json file. See `sequences/demo_seq.json` for a full implementation example.

The file is read be the *HeadNode* and specific parameters are mandatory for use.

## Parameters

The root must have 2 keys:

* `setup`: Contains a dictionary with what must be done at initialization and once all the actions are realized
* `actions`: All the actions to realize in a list

### Setup

|        Parameter         | Type  |        Default value         |  Unit   |                                                                                   Description                                                                                   |
|:------------------------:|:-----:|:----------------------------:|:-------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|        `actions`         | list  |          empty list          |   N/A   |             All the actions to realize for setup. See the [action section](#actions) for more details. Some actions type are not authorized for setup: `wait_until`             |
|        `end_pose`        | dict  | $${\color{red} mandatory}$$  |   N/A   |                            Dictionary containing the final position where the herminebot have to go. See the [pose section](#pose) for more details                             |
| `end_pose_reached_score` |  int  |              0               |   N/A   |                                                                      Score for reaching the final position                                                                      |
|   `end_pose_tolerance`   | dict  |          empty dict          |   N/A   | Dictionary containing the goal position tolerance and desired speed to reach the end position. See `xy_tolerance`, `yaw_tolerance` and `desired_speed` in the *actions* section |
|  `go_to_end_pose_time`   | float | $${\color{red} mandatory}$$  | seconds |                                                            Time at which the herminebot will go to the end position                                                             |
|      `initial_pose`      | dict  | $${\color{red} mandatory}$$  |   N/A   |                                                 Position where the robot will start. See [pose section](#pose) for more details                                                 |

### Actions

#### Common

Common parameters for all actions type.

| Parameter | Type  |        Default value        |  Unit   |                                                                                          Description                                                                                          |
|:---------:|:-----:|:---------------------------:|:-------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| `comment` |  str  |        empty string         |   N/A   |                                                                            Comment to display during the execution                                                                            |
| `depend`  | list  |         empty list          |   N/A   |                                                                        List of actions ids the action is dependent to                                                                         |
|   `id`    |  int  | $${\color{red} mandatory}$$ |   N/A   |                                                              Identification of the action. The id must be unique to each action.                                                              |
|  `score`  |  int  |              0              |   N/A   |                                                                                 Score if the action succeeds                                                                                  |
|  `skip`   | bool  |            False            |   N/A   |                                                               Whether to skip the action. **Should only be used for debugging**                                                               |
| `timeout` | float |            -1.0             | seconds |                                                                  Maximum time to realize the action otherwise it is canceled                                                                  |
|  `type`   |  str  |        no_type_given        |   N/A   | The type of the action. Authorized types: [drive](#drive), [gothrough](#gothrough), [goto](#goto), [set_pose](#set_pose), [spin](#spin), [wait_for](#wait_for) and [wait_until](#wait_until)  |

#### drive

Move the robot on its current heading. `distance` **and** `speed` must be negative for going backward.

| Parameter  | Type  | Default value |  Unit  |     Description      |
|:----------:|:-----:|:-------------:|:------:|:--------------------:|
| `distance` | float |      0.0      | meters |  Distance to drive   |
|  `speed`   | float |     0.025     |  m/s   | Speed at which drive |

#### gothrough

Navigate through positions.

|    Parameter    | Type  |                Default value                |  Unit   |                                 Description                                 |
|:---------------:|:-----:|:-------------------------------------------:|:-------:|:---------------------------------------------------------------------------:|
| `desired_speed` | float |  The speed defined for the RPP controller   |   m/s   |                  Speed at which the robot must try to move                  |
|     `poses`     | list  |         $${\color{red} mandatory}$$         |   N/A   | List of positions to go through. See [pose section](#pose) for more details |
| `xy_tolerance`  | float | The tolerance defined for the goal_checker  | meters  |           Position tolerance to consider the position as reached            |
| `yaw_tolerance` | float |  The tolerance define for the goal_checker  | radians |             Angle tolerance to consider the position as reached             |

#### goto

Go to a position.

|    Parameter    | Type  |                Default value                |  Unit   |                          Description                          |
|:---------------:|:-----:|:-------------------------------------------:|:-------:|:-------------------------------------------------------------:|
| `desired_speed` | float |  The speed defined for the RPP controller   |   m/s   |           Speed at which the robot must try to move           |
|     `pose`      | dict  |         $${\color{red} mandatory}$$         |   N/A   | Position to go to. See [pose section](#pose) for more details |
| `xy_tolerance`  | float | The tolerance defined for the goal_checker  | meters  |    Position tolerance to consider the position as reached     |
| `yaw_tolerance` | float |  The tolerance define for the goal_checker  | radians |      Angle tolerance to consider the position as reached      |

#### set_pose

Set the current position of the robot.

| Parameter | Type |        Default value        | Unit |                            Description                            |
|:---------:|:----:|:---------------------------:|:----:|:-----------------------------------------------------------------:|
|  `pose`   | dict | $${\color{red} mandatory}$$ | N/A  | Position of the robot. See [pose section](#pose) for more details |

#### spin

Rotate the robot of an angle.

| Parameter | Type  | Default value |  Unit   |   Description   |
|:---------:|:-----:|:-------------:|:-------:|:---------------:|
|  `angle`  | float |      0.0      | radians | Angle to rotate |

#### wait_for

Do nothing for the duration.

| Parameter  | Type  | Default value |  Unit   |            Description             |
|:----------:|:-----:|:-------------:|:-------:|:----------------------------------:|
| `duration` | float |      0.0      | seconds | Time during the robot does nothing |

#### wait_until

Wait until the time is reached.

| Parameter | Type  | Default value |  Unit   |            Description            |
|:---------:|:-----:|:-------------:|:-------:|:---------------------------------:|
|  `time`   | float |      0.0      | seconds | Time until the robot does nothing |

#### preempt

Preempt an object

This action has no parameter.

### pose

*pose* is a dictionary type that **must** contain these 3 parameters: 

| Parameter | Type  |        Default value        |  Unit   |      Description      |
|:---------:|:-----:|:---------------------------:|:-------:|:---------------------:|
|    `x`    | float | $${\color{red} mandatory}$$ | meters  | Position along x axis |
|    `y`    | float | $${\color{red} mandatory}$$ | meters  | Position along y axis |
|   `yaw`   | float | $${\color{red} mandatory}$$ | radians |  Angle around z axis  |
