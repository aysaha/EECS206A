#!/usr/bin/env python
#This node gets a slave robot to follow a master

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from planning.msg import State


TIMER_FREQ = 100

ROBOT1_STATE = None
ROBOT2_STATE = None

ROBOT1_CONTROL = None
robot2_config = None

integral = 0

current_state = "FOLLOW"

def robot1_state_callback(msg):
    global ROBOT1_STATE
    ROBOT1_STATE = msg


def robot2_state_callback(msg):
    global ROBOT2_STATE
    ROBOT2_STATE = msg

def robot1_control_callback(msg):
    global ROBOT1_CONTROL
    ROBOT1_CONTROL = msg


def controller(move=True):
    global integral, TIMER_FREQ, ROBOT2_STATE, ROBOT1_STATE, ROBOT1_CONTROL, robot2_config
    global current_state

    if ROBOT1_STATE is None or ROBOT2_STATE is None or ROBOT1_CONTROL is None:
        return Twist()

    ############################ PARAMETERS ##################################
    

    """
    #K1: Speed proportionnality constant
    K1 = 0.6

    #K2: Angular velocity proportionnality constant
    Kp = -2
    Ki = -1

    Kd = 0
    cap = .2

    max_rot = 1.57

    #Tolerance values for distance and angle
    eps_d = 0.05
    eps_theta = 0.03

    #Distance in meters before the turtlebot starts slowing down
    slowdown_threshold = 0.15

    #Turtlebot max speed in meters per sec
    max_speed = K1 * slowdown_threshold

    ######### Get next waypoint target ##########
    x = state.x
    y = state.y
    theta = state.theta

    #Integral reset if tqrget has changed
    if target.x != last_target.x:
        integral = 0
        last_target = target
    

    x_w,y_w,theta_w = target.x, target.y, target.theta
    last_waypoint = abs(x_w) < 0.01 and abs(y_w) < 0.01
    #print("Last wp " + str(last_waypoint))

    x = x - x_w
    y = y - y_w
    theta = theta - theta_w
    

    d = np.sqrt(x*x + y*y)
    #theta_r = np.arctan2(-y, -x)


    integral = integral + theta / TIMER_FREQ
    if integral > cap:
        integral = cap
    if integral < -cap:
        integral = - cap

    #Correct the angle singularity
    '''
    if delta_theta > 3.14:
        delta_theta -= 6.28
        print("singularity detected")
    if delta_theta < -3.14:
        delta_theta += 6.28
        print("singularity detected")
	'''
    """

    print(ROBOT1_CONTROL)

    base_command = Twist()
    base_command.angular = ROBOT1_CONTROL.angular    
    base_command.linear.x = ROBOT1_CONTROL.linear.x + ROBOT1_CONTROL.angular.z * robot2_config
    print(base_command)
    return base_command

    next_state = current_state
    
    #print(last_waypoint)
    #print(current_state)
    #print(target)
    #print("theta: " + str(theta * 180 / np.pi))
    #print("distance: " + str(d))
    #print("integral: " + str(integral))
    #print(target.x)
    
    ################# FOLLOW STATE #################
    if current_state == "FOLLOW":
        pass




    ################# REVERSE STATE ##################
    if current_state == "REVERSE":
        command.linear.x = -0.1

        if d > 0.2:
            next_state = "MOVE"


    ################### STOP STATE ####################
    elif current_state == "STOP":
        pass
        #Do nothing


    ################# MOVE STATE #####################
    elif current_state == "MOVE":

        #If you're farther than a certain threshhold value, go at max speed
        if d > slowdown_threshold or (not last_waypoint):
            command.linear.x = K1*d #max_speed
            command.angular.z = Kp * theta + Ki * integral

        #Then slow down as you get closer
        else:
            command.linear.x = K1 * x
            command.angular.z = Kp * theta + Ki * integral

        #Transition happens when closer to goal than tolerance
        if d < eps_d and last_waypoint:
            next_state = "ADJUST"


    ################# ADJUST STATE ##################
    elif current_state == "ADJUST":
        command.linear.x = 0
        command.angular.z = Ki * theta

        if abs(theta) < eps_theta:
            next_state = "STOP"


    ################# PROBLEM STATE ##################
    else:
        print("WRONG STATE: " + current_state)
        exit(1)
    

    #State machine transition
    current_state = next_state
    
    command.linear.x = np.clip(command.linear.x, 0, max_speed)
    command.angular.z = np.clip(command.angular.z, -max_rot, max_rot)

    if move is True:
        robot1_command = command
        robot2_command = Twist()
    else:
        print(command)
        robot1_command = Twist()
        robot2_command = Twist()

    return robot1_command, robot2_command


def main():
    global ROBOT1_STATE, ROBOT1_TARGET, TIMER_FREQ, robot2_config

    # initialize ROS node
    rospy.init_node('follow_controller')

    # load data from parameter server
    try:
        robot2_control = rospy.get_param('/follow_controller/robot2_control')
        robot2_config = rospy.get_param('/follow_controller/robot2_config')
        robot1_control = rospy.get_param('/follow_controller/robot1_control')
    except Exception as e:
        print("[follow_controller]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/state_observer/robot1_state', State, robot1_state_callback)
    rospy.Subscriber('/state_observer/robot2_state', State, robot2_state_callback)
    rospy.Subscriber(robot1_control, Twist, robot1_control_callback)

    # create ROS publisher
    robot2_publisher = rospy.Publisher(robot2_control, Twist, queue_size=1)

    # create a 100Hz timer
    timer = rospy.Rate(TIMER_FREQ)

    while not rospy.is_shutdown():
        # calculate control inputs
        robot2_command = controller()

        # publish control inputs
        robot2_publisher.publish(robot2_command)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
