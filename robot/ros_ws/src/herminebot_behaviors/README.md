# herminebot_behavior

This package contains behavior plugins for the herminebot.

# Behavior trees

## Preemption

Behavior tree for object preemption.

Default behavior tree: `behavior_tree/preemption.xml`

### Command line call

```bash
ros2 action send_goal "preemption_navigator" hrc_interfaces/action/Preempt '{}'
```

# Behavior plugins

## MoveElevators

This plugin allows to command the elevators to reach a position. The id list match the poses list in the order.

### Command line

```bash
ros2 action send_goal "move_elevators" hrc_interfaces/action/MoveElevators "{
    time_allowance: {sec: 3, nanosec: 0}, 
    elevators_ids: [0],
    elevators_poses: [0.1]
}"
```

### XML behavior tree

```xml
<MoveElevators time_allowance="3.0" elevators_ids="0;2" elevators_poses="0.1;9"/>
```

### Configuration

|      Parameter      |  Type  | Default value | Unit  |                                        Description                                         |
|:-------------------:|:------:|:-------------:|:-----:|:------------------------------------------------------------------------------------------:|
| `position_accuracy` | double |     0.01      | meter | Difference between the current pose and the goal pose at which it is considered as reached |

## ManageMap

This plugin allows to modify the elements mask map. The mask can be modified with positions relative to the robot position.

### XML behavior tree

```xml
<ManageMap 
    new_objects="
        [[-0.5,-0.2], [-0.5,0.2], [-0.4,0.2], [-0.4,-0.2]];
        [[-0.9,-0.2], [-0.9,0.2], [-0.7,0.2], [-0.7,-0.2]]"
    points_objects_to_remove="[[0.15, 0.0], [0.1, 0.1]]"
    is_robot_relative="true"
/>
```

## OmniDrive

This plugin allows to move the robot in any direction.

### Command line

```bash
ros2 action send_goal "omni_drive" hrc_interfaces/action/OmniDrive "{
    target: {x: 0.1, y: 0.1},
    speed: 0.1,
    time_allowance: {sec: 3, nanosec: 0}
}"
```

### XML behavior tree

```xml
<OmniDrive target_x="0.1" target_y="0.1" speed="0.1" time_allowance="3.0"/>
```
