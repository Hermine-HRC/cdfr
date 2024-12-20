# hrc_interfaces

The package contains new ros interfaces for messages, services and actions.

# Actions

## MoveElevators

Action message for moving elevators.

### Input

|       Name        |            Type             |   Unit   |                  Description                   |
|:-----------------:|:---------------------------:|:--------:|:----------------------------------------------:|
| `time_allowance`  | builtin_interfaces/Duration | Duration |       Time allowed to execute the action       |
|  `elevators_ids`  |           int32[]           |   N/A    |       List of the elevators ids to move        |
| `elevators_poses` |          float64[]          |  meter   | List of the positions the elevators must reach |

### Output

|      Name       |   Type    | Unit  |                       Description                       |
|:---------------:|:---------:|:-----:|:-------------------------------------------------------:|
| `current_poses` | float64[] | meter | The current position of the elevators that have to move |

## Preempt

Action message for an object preemption.

### Input

|      Name       |  Type  | Unit |       Description        |
|:---------------:|:------:|:----:|:------------------------:|
| `behavior_tree` | string | N/A  | The behavior tree to use |

### Output

|          Name          | Type  | Unit |          Description          |
|:----------------------:|:-----:|:----:|:-----------------------------:|
| `number_of_recoveries` | int16 | N/A  | The number of recoveries done |

# Messages

# Services

## GetTeamColor

Service message for getting the team color.

### Command line usage

```bash
ros2 service call /get_team_color hrc_interfaces/srv/GetTeamColor
```

### Input

No input asked for this service.

### Output

|     Name     |  Type  | Unit |      Description      |
|:------------:|:------:|:----:|:---------------------:|
| `team_color` | string | N/A  | The color of the team |

## ManageObjectsMap

Service message for adding and removing obstacle objects on the map.

### Command line usage

```bash
ros2 service call /manage_object_map hrc_interfaces/srv/ManageObjectsMap "{
points_objects_to_remove: [{x: 0.15, y: 0.09}],
new_objects: [{points: [{x: -0.2, y: -0.1}, {x: -0.2, y: 0.1}, {x: 0.2, y: 0.1}, {x: 0.2, y: -0.1}]}]
}"
```

### Input

|            Name            |       Type        | Unit  |                                       Description                                        |
|:--------------------------:|:-----------------:|:-----:|:----------------------------------------------------------------------------------------:|
|       `new_objects`        | Array of polygons | meter |                 An array containing polygons points to add as obstacles                  |
| `points_objects_to_remove` |  Array of points  | meter | An array containing points and all the polygons containing at least one point is removed |

### Output

There is no output for this service

## GetRobotPose

Service message for getting the robot pose.

### Command line usage

```bash
ros2 service call /get_robot_pose hrc_interfaces/srv/GetRobotPose "{base_frame: 'map', robot_frame: 'base_link'}"
```

### Input

|     Name      |  Type  | default value | Unit |      Description       |
|:-------------:|:------:|:-------------:|:----:|:----------------------:|
| `base_frame`  | string |     "map"     | N/A  |  The frame of the map  |
| `robot_frame` | string |  "base_link"  | N/A  | The frame of the robot |

### Output

|     Name     |        Type         |     Unit      |        Description        |
|:------------:|:-------------------:|:-------------:|:-------------------------:|
| `robot_pose` | geometry_msg/Pose2D | meter/radians | The position of the robot |
