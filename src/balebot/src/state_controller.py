#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from balebot.msg import State


TARGET1 = None
TARGET2 = None
STATE1 = None
STATE2 = None


def polar(source_state, target_state):
    # use the source state as the reference
    x = target_state.x - source_state.x
    y = target_state.y - source_state.y
    
    # convert from Cartesian to polar coordinates
    distance = np.sqrt(np.power(x, 2) + np.power(y, 2))
    angle = np.arctan2(y, x)

    return distance, angle


def target1_callback(msg):
    global TARGET1

    TARGET1 = msg


def target2_callback(msg):
    global TARGET2

    TARGET2 = msg


def state1_callback(msg):
    global STATE1

    STATE1 = msg


def state2_callback(msg):
    global STATE2

    STATE2 = msg


def control(Kv=0.5, Kw=2):
    global STATE1, TARGET1

    command = Twist()

    distance, angle = polar(STATE1, TARGET1)

    if TARGET1.x == 0 and TARGET1.y == 0 and TARGET1.theta == 0:
        command.linear.x = Kv * distance
        command.angular.z = Kw * (angle - STATE1.theta)
    else:
        command.linear.x = 0.5
        command.angular.z = Kw * (angle - STATE1.theta)

    return command


def main():
    global TARGET1, TARGET2, STATE1, STATE2
    
    # initialize ROS node
    rospy.init_node('state_controller')
    
    # load data from parameter server
    try:
        robot1_control = rospy.get_param('/state_controller/robot1_control')
        robot2_control = rospy.get_param('/state_controller/robot2_control')
    except Exception as e:
        print("[state_controller]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/path_planner/target1', State, target1_callback)
    rospy.Subscriber('/path_planner/target2', State, target2_callback)
    rospy.Subscriber('/state_observer/state1', State, state1_callback)
    rospy.Subscriber('/state_observer/state2', State, state2_callback)
    
    # create ROS publishers
    publisher1 = rospy.Publisher(robot1_control, Twist, queue_size=1)
    publisher2 = rospy.Publisher(robot2_control, Twist, queue_size=1)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # generate control input
        if STATE1 is not None and TARGET1 is not None:
            command1 = control()
        else:
            command1 = Twist()
        
        command2 = Twist()

        # publish command
        publisher1.publish(command1)
        publisher2.publish(command2)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
