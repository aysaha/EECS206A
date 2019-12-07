#!/usr/bin/env python

import numpy as np
import rospy
from turtlesim.msg import Pose
from balebot.msg import State


TARGET = State(7, 7, 90 * np.pi / 180)
STATE = None


def callback(msg):
    global TARGET, STATE

    

    x = msg.x * np.cos(TARGET.theta) - msg.y * np.sin(TARGET.theta) - TARGET.x
    y = msg.x * np.sin(TARGET.theta) + msg.y * np.cos(TARGET.theta) - TARGET.y
    theta = msg.theta - TARGET.theta
    STATE = State(x, y, theta)


def main():
    global STATE

    # initialize ROS node
    rospy.init_node('state_observer')

    # get configuration from parameter server
    try:
        robot_frame = rospy.get_param('/state_observer/robot_frame')
    except Exception as e:
        print(e)
        exit(1)

    # create ROS subscriber
    rospy.Subscriber(robot_frame, Pose, callback)

    # create ROS publisher
    publisher = rospy.Publisher('/state_observer/state', State, queue_size=1)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # publish robot state
        if STATE is not None:
            publisher.publish(STATE)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
