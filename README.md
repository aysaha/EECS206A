# EECS206A
Introduction to Robotics Final Project

#Getting started with path controller:

	*Clone repo/pull changes
	*catkin_make in root directory
	*source devel/setup.bash in setup directory

	*add "export ROS_MASTER_URI=http://[TurtleID].local:11311" line to your .bashrc (and source .bashrc)

	*ssh into the turtlebot: ssh turtlebot@[TurtleID].local (pwd : EE106A19)
	*In the turtlebot, launch bringup sequence : roslaunch turtlebot_bringup minimal.launch --screen
	*All following instructions are executed on the local machine

	*Launch the realsense camera: roslaunch lab4_cam run_cam.launch
	*Launch the ar_tags tracker: roslaunch lab4_cam ar_track.launch

	*Launch rviz, add a camera->image_raw topic, and a TF object. Set the fixed marker as the fixed frame, disable all tf frames except for usb_cam and both ar markers

#Running path controller

	*Optionnaly, define a goal frame relative to the fixed ar_marker: rosrun tf static_transform_publisher x y z yaw pitch roll fixed_marker_id goal_frame 100

	*Run the controller: rosrun control motion_controller.py turtlebot_frame goal_frame