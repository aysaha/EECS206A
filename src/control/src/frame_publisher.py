#!/usr/bin/env python

#Ensures that the goal frame does not disappear when the turtlebot is blocking the view of the camera



import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import tf2_msgs.msg

#Create a twist that holds the last updated frame
last_transform = TransformStamped()
last_transform.header.stamp.secs = -1

def publish_frame(frame):

	global last_transform

	pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

	#Listen for tf updates
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	# create a 10Hz timer
	timer = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		try:
			# get the transform from source_frame to target_frame
			last_transform = tf_buffer.lookup_transform('usb_cam', frame, rospy.Time())

			if rospy.Time.now().secs - last_transform.header.stamp.secs > 0:
				raise tf2_ros.LookupException

			else:
				print("Updating tranform")

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

			if last_transform.header.stamp.secs == -1:
				print("No previous transform to publish!")
			else:
				print("Camera view is blocked, publishing most revent available transform")
				last_transform.header.stamp = rospy.Time.now()
				tfm = tf2_msgs.msg.TFMessage([last_transform])
				pub.publish(tfm)

		timer.sleep()



def main(args):
    # check for correct number of arguments
    if len(args) != 2:
        print("Usage: goal_frame")
        exit(1)

    # initialize ROS node
    rospy.init_node('goal_frame_publisher', anonymous=True)
    
    # run controller
    publish_frame(sys.argv[1])


if __name__ == '__main__':
    main(sys.argv)
