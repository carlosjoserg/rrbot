# rrbot
Comprehensive example to show recent capabilities in ROS/Gazebo/MoveIt!

## Dependencies

## Uses

* Bring up the `single_rrbot` (it loads the simulated one by default):

`roslaunch rrbot_launch bringupRRBOT.launch`

* Move the `single_rrbot` using `ros-controls`

Open `rqt`, `Plugins`->`Robot Tools` -> `Joint trajectory controller` and move the `single_rrbot` robot.

* Move the `single_rrbot` using MoveIt!

Load the planning environment:

`roslaunch single_rrbot_moveit_config move_group.launch`

Open the pre-configured rviz and use the MoveIt! display:

`roslaunch single_rrbot_moveit_config moveit_rviz.launch`
