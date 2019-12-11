#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def main():
    # initialize ROS node
    rospy.init_node('frame_publisher')
    
    # load data from parameter server
    try:
        goal_frame = rospy.get_param('/state_observer/goal_frame')
    except Exception as e:
        print("[frame_publisher]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS publisher
    publisher = rospy.Publisher('/tf', TFMessage, queue_size=1)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # initialize frame
    tf_frame = TransformStamped()

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            # get transformation from goal frame to camera frame
            tf_frame = tf_buffer.lookup_transform('usb_cam', goal_frame, rospy.Time())

            if rospy.Time.now().secs - tf_frame.header.stamp.secs > 0:
                raise Exception
        except:
            # check if valid transform exists
            if tf_frame.child_frame_id:
                tf_frame.header.stamp = rospy.Time.now()
                publisher.publish(TFMessage([tf_frame]))

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
