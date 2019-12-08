#!/usr/bin/env python

import numpy as np
import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute
from std_srvs.srv import Empty
from balebot.msg import State

TARGET = State(7, 7, -90 * np.pi / 180)
STATE1 = None
STATE2 = None


def transform(robot_frame, target_frame):
    RT = np.array([[np.cos(target_frame.theta), np.sin(target_frame.theta)], [-np.sin(target_frame.theta), np.cos(target_frame.theta)]])
    p = np.array([target_frame.x, target_frame.y])
    q = np.array([robot_frame.x, robot_frame.y]) 
    q = np.matmul(RT, q) - np.matmul(RT, p)
    
    return State(q[0], q[1], robot_frame.theta - target_frame.theta)


def callback1(msg):
    global TARGET, STATE1

    STATE1 = transform(msg, TARGET)


def callback2(msg):
    global TARGET, STATE2

    STATE2 = transform(msg, TARGET)


def main():
    global STATE1, STATE2

    # initialize ROS node
    rospy.init_node('state_observer')
    
    # load data from parameter server
    try:
        simulation = rospy.get_param('/state_observer/simulation')
        robot1_frame = rospy.get_param('/state_observer/robot1_frame')
        robot1_config = [float(val) for val in rospy.get_param('/path_planner/robot1_config').split(',')]
        robot2_frame = rospy.get_param('/state_observer/robot2_frame')
        robot2_config = [float(val) for val in rospy.get_param('/path_planner/robot2_config').split(',')]
    except Exception as e:
        print("[state_observer]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber(robot1_frame, Pose, callback1)
    rospy.Subscriber(robot2_frame, Pose, callback2)

    # create ROS publishers
    publisher1 = rospy.Publisher('/state_observer/state1', State, queue_size=1)
    publisher2 = rospy.Publisher('/state_observer/state2', State, queue_size=1)

    # initialize simulator
    if simulation is True:
        origin = [1, 5]

        clear = rospy.ServiceProxy('clear', Empty)
        spawn = rospy.ServiceProxy('spawn', Spawn)
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        
        rospy.wait_for_service('clear')
        rospy.wait_for_service('spawn')
        rospy.wait_for_service('turtle1/teleport_absolute')

        teleport(robot1_config[0] + origin[0], robot1_config[1] + origin[1], 0)
        spawn(robot2_config[0] + origin[0], robot2_config[1] + origin[1], 0, 'turtle2')
        clear()

    # create a 10Hz timer
    timer = rospy.Rate(10)

    while not rospy.is_shutdown():
        # publish robot states
        if STATE1 is not None:
            publisher1.publish(STATE1)
        
        if STATE2 is not None:
            publisher2.publish(STATE2)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
