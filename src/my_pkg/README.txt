To launch navigation:
	In one terminal start gazebo:
		$ export TURTLEBOT3_MODEL=burger
		$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
	In second terminal publish desired position, where "z" is the desired orientation, such as:
		$ rostopic pub /dpos geometry_msgs/Point32 '{x: -1, y: 2, z: 3.14}'
	In third terminal start the navigation node:
		$ cd catkin_ws
		$ catkin_make
		$ rosrun my_pkg my_pkg_navi_node
