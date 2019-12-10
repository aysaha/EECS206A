#!/usr/bin/env python

from enum import Enum
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from balebot.msg import State


ROBOT1_STATE = None
ROBOT2_STATE = None
GROUP_STATE = None
GROUP_TARGET = None


'''
class ControlState(Enum):
    Unknown = 0
    Rotate = 1
    Translate = 2
    Adjust = 3
    Follow = 4
    Stop = 5

def polar(source_state, target_state):
    # use the source state as the reference
    x = target_state.x - source_state.x
    y = target_state.y - source_state.y
    
    # convert from Cartesian to polar coordinates
    distance = np.sqrt(np.power(x, 2) + np.power(y, 2))
    angle = np.arctan2(y, x)

    return distance, angle
'''

def control(robot1_config, robot2_config, Kv=1, Kw=2):
    global GROUP_STATE, GROUP_TARGET

    robot1_command = Twist()
    robot2_command = Twist()
 
    '''
    if GROUP_STATE is not None and GROUP_TARGET is not None:
        error_theta = GROUP_TARGET.theta - GROUP_STATE.theta
        W = Kw * error_theta
        
        robot1_command.angular.z = W
        robot1_command.linear.x = 0.25 + W * abs(robot1_config)
        
        robot2_command.angular.z = W
        robot2_command.linear.x = 0.25 + W * abs(robot2_config)
    '''

    return robot1_command, robot2_command



def robot1_state_callback(msg):
    global ROBOT1_STATE

    ROBOT1_STATE = msg


def robot2_state_callback(msg):
    global ROBOT2_STATE

    ROBOT2_STATE = msg


def group_state_callback(msg):
    global GROUP_STATE

    GROUP_STATE = msg


def group_target_callback(msg):
    global GROUP_TARGET

    GROUP_TARGET = msg


def main():
    # initialize ROS node
    rospy.init_node('state_controller')

    # load data from parameter server
    try:
        robot1_config = rospy.get_param('/path_planner/robot1_config')
        robot2_config = rospy.get_param('/path_planner/robot2_config')
        robot1_control = rospy.get_param('/state_controller/robot1_control')
        robot2_control = rospy.get_param('/state_controller/robot2_control')
    except Exception as e:
        print("[state_controller]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/state_observer/robot1_state', State, robot1_state_callback)
    rospy.Subscriber('/state_observer/robot2_state', State, robot2_state_callback)
    rospy.Subscriber('/state_observer/group_state', State, group_state_callback)
    rospy.Subscriber('/path_planner/group_target', State, group_target_callback)
    
    # create ROS publishers
    robot1_publisher = rospy.Publisher(robot1_control, Twist, queue_size=1)
    robot2_publisher = rospy.Publisher(robot2_control, Twist, queue_size=1)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # generate control input
        robot1_command, robot2_command = control(robot1_config, robot2_config)

        # publish command
        robot1_publisher.publish(robot1_command)
        robot2_publisher.publish(robot2_command)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
