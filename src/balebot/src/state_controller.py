#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from balebot.msg import State


TARGET = None
STATE = None
I = 0

def polar(source_state, target_state):
    # use the source state as the reference
    x = target_state.x - source_state.x
    y = target_state.y - source_state.y
    
    # convert from Cartesian to polar coordinates
    distance = np.sqrt(np.power(x, 2) + np.power(y, 2))
    angle = np.arctan2(y, x)

    return distance, angle


def target_callback(msg):
    global TARGET

    TARGET = msg


def state_callback(msg):
    global STATE

    STATE = msg


def control(Kv=1, Kw=0.25):
    global STATE, TARGET, I

    command = Twist()

    distance, angle = polar(STATE, TARGET)

    '''
    print('-----------')
    print(STATE)
    print('-----------')
    print(TARGET)
    print('-----------')
    print(angle * 180 / np.pi)
    print('-----------')
    exit(0)
    '''
    if TARGET.x == 0 and TARGET.y == 0 and TARGET.theta == 0:
        print("[state_controller]: moving to last waypoint")
        command.linear.x = Kv * distance
        command.angular.z = 0 
    else:
        command.linear.x = 0.5
        command.angular.z = -Kw * angle
    
    I += 1
    if I % 100 == 0:
        print("[state_controller]: " + str(angle * 180 / np.pi) + "deg")

    return command


def main():
    global TARGET
    global STATE
    
    # initialize ROS node
    rospy.init_node('state_controller')
    
    # create ROS subscriber
    rospy.Subscriber('/path_planner/target', State, target_callback)
    rospy.Subscriber('/state_observer/state', State, state_callback)
    
    # create ROS publisher
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # generate control input
        if STATE is not None and TARGET is not None:
            command = control()
        else:
            command = Twist()

        # publish command
        publisher.publish(command)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
