# FILE TO SOURCE

# Publish on topic '/initial_pose' to set the current position of the robot manually
# Takes as arguments: x in m, y in m and yaw in rad
set_pose () {
    if [ "$#" -ne 3 ];
	then
		echo "You must give 'x', 'y' and 'yaw' as arguments"
		return 1
	fi

	x=$1
	y=$2
	yaw=$3

	ros2 topic pub --times 1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{
        header: {frame_id: 'map'},
        pose: {
            pose: {
                position: { x: $x, y: $y, z: 0.0 },
                orientation: { z: $(echo "s($yaw / 2)" | bc -l), w: $(echo "c($yaw / 2)" | bc -l) }
            },
            covariance: [
                0.001, 0, 0, 0, 0, 0,
                0, 0.001, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.001
            ]
        }
    }"
}

# Publish on topic '/goal_pose' to send a goal position to the robot
# Takes as arguments: x in m, y in m and yaw in rad
goto () {
    if [ "$#" -ne 3 ];
	then
		echo "You must give 'x', 'y' and 'yaw' as arguments"
		return 1
	fi

	x=$1
	y=$2
	yaw=$3

	ros2 topic pub -t 1 /goal_pose geometry_msgs/PoseStamped "{
        header: {stamp: {sec: 0}, frame_id: 'map'},
        pose: {
            position: {x: $x, y: $y, z: 0.0},
            orientation: { z: $(echo "s($yaw / 2)" | bc -l), w: $(echo "c($yaw / 2)" | bc -l) }
        }
    }"
}

start_actions () {
    ros2 service call /start_actions hrc_interfaces/srv/StartActions
}

restart() {
    ros2 topic pub -t 1 /restart hrc_interfaces/msg/Restart
}
