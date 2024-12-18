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

### Input

No input asked for this service.

### Output

|     Name     |  Type  | Unit |      Description      |
|:------------:|:------:|:----:|:---------------------:|
| `team_color` | string | N/A  | The color of the team |
