#!/usr/bin/env python

import sys
import rospy
import tf2_ros


def plan(destination, steps=10):
	waypoint = []
	position = (0, 0)

	for i in range(steps):
		theta = np.arctan2(y, x)
		waypoint.append((position[0] * np.cos(theta), position[1] * np.sin(theta))) 

	return waypoint

def planner(source_frame, target_frame):
	# create ROS publisher
    #turtlebot = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

	# create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

	try:
   		# get the transform from source_frame to target_frame
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        path = plan((transform.transform.translation.x, transform.transform.translation.y))
        print(path)
        #turtlebot.publish(command)
  	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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