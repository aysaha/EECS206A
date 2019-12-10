#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


START_FRAME = None
END_FRAME = None


def main():
    global START_FRAME, END_FRAME
    # initialize ROS node
    rospy.init_node('frame_publisher')
    
    # load data from parameter server
    try:
        start_frame = rospy.get_param('/state_observer/start_frame')
        end_frame = rospy.get_param('/state_observer/end_frame')
    except Exception as e:
        print("[frame_publisher]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS publisher
    publisher = rospy.Publisher('/tf', TFMessage, queue_size=1)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # initialize frames
    START_FRAME = TransformStamped()
    END_FRAME = TransformStamped()

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # publish start frame
        try:
            START_FRAME = tf_buffer.lookup_transform('usb_cam', start_frame, rospy.Time())

            if rospy.Time.now().secs - START_FRAME.header.stamp.secs > 0:
                raise Exception
        except Exception as e:
            if START_FRAME.child_frame_id:
                START_FRAME.header.stamp = rospy.Time.now()
                publisher.publish(TFMessage([START_FRAME]))

        # publish end frame
        try:
            END_FRAME = tf_buffer.lookup_transform('usb_cam', end_frame, rospy.Time())

            if rospy.Time.now().secs - END_FRAME.header.stamp.secs > 0:
                raise Exception
        except Exception as e:
            if END_FRAME.child_frame_id:
                END_FRAME.header.stamp = rospy.Time.now()
                publisher.publish(TFMessage([END_FRAME]))

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
