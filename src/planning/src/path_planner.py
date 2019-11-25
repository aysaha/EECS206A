#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf2_ros


def plan(destination, steps=1000, alpha=1):
	waypoints = []
	curr = np.array([0.0, 0.0])
	prev = np.array([-1.0, -0.0])

	print("destination = " + str(destination))
	x = destination[0]
	y = destination[1]

	for i in range(steps):
		# velocity vector
		theta = np.arctan2(-y, -x)
		mag = np.sqrt(x * x + y * y) / steps
		v = np.array([mag * np.cos(theta), mag * np.sin(theta)])

		# momentum vector
		m = curr - prev
		m = m / np.sqrt(m[0] * m[0] + m[1] * m[1]) / steps

		vect = alpha * v + (1 - alpha) * m
		waypoints.append(curr + vect)

		prev = curr
		curr = curr + vect

	for waypoint in waypoints:
		print(str(waypoint[0]) + " " + str(waypoint[1]))

	return waypoints

def planner(source_frame, target_frame):
	# create ROS publisher
	#turtlebot = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

	# create tf buffer primed with a tf listener
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	try:
   		# get the transform from source_frame to target_frame
   		transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
   		path = plan((transform.transform.translation.x, transform.transform.translation.y))

        #turtlebot.publish(command)
  	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		print(e)
		pass


def main(args):
    # check for correct number of arguments
    if len(args) != 3:
        print("Usage: path_planner source_frame target_frame")
        exit(1)

    # initialize ROS node
    rospy.init_node('path_planner', anonymous=True)

    planner(sys.argv[1], sys.argv[2])


if __name__ == '__main__':
    main(sys.argv)