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
