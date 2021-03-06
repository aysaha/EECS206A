#!/usr/bin/env python

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from balebot.msg import State
from path_planner import polar


TIMER_FREQ = 100
ROBOT1_STATE = None
ROBOT2_STATE = None
ROBOT3_STATE = None
ROBOT1_ERROR = None
ROBOT2_ERROR = None
ROBOT3_ERROR = None
ROBOT1_TARGET = None
last_target = State()
integral = 0
current_state = "MOVE"


def translate(command, config, error, Kx=2, Kz=1):
    local_command = Twist()

    if command is not None:
        # determine theoretical command input
        local_command.linear.x = command.linear.x - config * command.angular.z
        local_command.angular.z = command.angular.z

        # adjust for errors
        if error is not None:
            local_command.linear.x -= Kx * error.x
            local_command.angular.z -= Kz * error.theta

    return local_command


def setup_controller(error, state):
    if error is None:
        return Twist(), state, False

    #K1: Speed proportionnality constant
    K1 = 0.6

    #K2: Angular velocity proportionnality constant
    Kp = -2
    Ki = -1

    #Tolerance values for distance and angle
    eps_d = 0.15
    eps_theta = 0.1

    #Distance in meters before the turtlebot starts slowing down
    slowdown_threshold = 0.15

    #Turtlebot max speed in meters per sec
    max_speed = K1 * slowdown_threshold

    next_state = state
    flag = False
    d, angle = polar(error, State(0, 0, 0))
    theta_r = np.arctan2(-error.y, -error.x)
    command = Twist()

    if state == "MOVE":
        if d > slowdown_threshold:
            command.linear.x = max_speed
            command.angular.z = Kp * (error.theta - theta_r)
        else:
            command.linear.x = K1 * d
            command.angular.z = Kp * (error.theta - theta_r)

        if d < eps_d:
            next_state = "ADJUST"
    elif state == "ADJUST":
        command.linear.x = 0
        command.angular.z = Kp * error.theta

        if abs(error.theta) < eps_theta:
            next_state = "DONE"
    elif state == "DONE":
        flag = True
    else:
        print("[motion_controller] unknown state: " + state)
        exit(1)

    return command, next_state, flag


def controller(state, target, error, move=True):
    global last_target, integral, TIMER_FREQ, current_state
    
    if error is None:
    	return Twist()

    ############################ PARAMETERS ##################################
    
    #K1: Speed proportionnality constant
    K1 = 0.6
    Ky = -14

    #K2: Angular velocity proportionnality constant
    Kp = -2
    Ki = -1

    cap = .2
    max_rot = 1.57

    #Tolerance values for distance and angle
    eps_d = 0.07
    eps_theta = 0.1

    #Distance in meters before the turtlebot starts slowing down
    slowdown_threshold = 0.15

    #Turtlebot max speed in meters per sec
    max_speed = K1 * slowdown_threshold

    ######### Get next waypoint target ##########
    x = state.x
    y = state.y
    theta = state.theta

    #Integral reset if target has changed
    if target.x != last_target.x:
        integral = 0
        last_target = target
    
    x_w,y_w,theta_w = target.x, target.y, target.theta
    last_waypoint = abs(x_w) < 0.01 and abs(y_w) < 0.01

    #x = x - x_w
    #y = y - y_w
    #theta = theta - theta_w
    x = error.x
    y = error.y
    theta = error.theta

    d = np.sqrt(x*x + y*y)

    integral = integral + theta / TIMER_FREQ
    if integral > cap:
        integral = cap
    if integral < -cap:
        integral = - cap

    command = Twist()
    next_state = current_state

    ################### STOP STATE ####################
    if current_state == "STOP":
        #Do nothing
        pass


    ################# MOVE STATE #####################
    elif current_state == "MOVE":
        #print("distance : " + str(d))
        #If you're farther than a certain threshhold value, go at max speed
        if d > slowdown_threshold or (not last_waypoint):
            command.linear.x = K1*d #max_speed
            command.angular.z = Kp * theta + Ki * integral + Ky * y

        #Then slow down as you get closer
        else:
            command.linear.x = K1 * x
            command.angular.z = Kp * theta + Ki * integral + Ky * y

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
        print("[motion_controller] unknown state: " + current_state)
        exit(1)


    #State machine transition
    current_state = next_state
    
    command.linear.x = np.clip(command.linear.x, 0, max_speed)
    command.angular.z = np.clip(command.angular.z, -max_rot, max_rot)

    if move is False:
        command = Twist()

    return command


def robot1_state_callback(msg):
    global ROBOT1_STATE

    ROBOT1_STATE = msg


def robot2_state_callback(msg):
    global ROBOT2_STATE

    ROBOT2_STATE = msg


def robot3_state_callback(msg):
    global ROBOT3_STATE

    ROBOT3_STATE = msg


def robot1_error_callback(msg):
    global ROBOT1_ERROR

    ROBOT1_ERROR = msg


def robot2_error_callback(msg):
    global ROBOT2_ERROR

    ROBOT2_ERROR = msg


def robot3_error_callback(msg):
    global ROBOT3_ERROR

    ROBOT3_ERROR = msg


def robot1_target_callback(msg):
    global ROBOT1_TARGET

    ROBOT1_TARGET = msg


def main():
    global TIMER_FREQ, ROBOT1_STATE, ROBOT2_STATE, ROBOT3_STATE, ROBOT1_ERROR, ROBOT2_ERROR, ROBOT3_ERROR, ROBOT1_TARGET

    # initialize ROS node
    rospy.init_node('motion_controller')

    # load data from parameter server
    try:
        robot2_config = rospy.get_param('/motion_controller/robot2_config')
        robot3_config = rospy.get_param('/motion_controller/robot3_config')
        robot1_control = rospy.get_param('/motion_controller/robot1_control')
        robot2_control = rospy.get_param('/motion_controller/robot2_control')
        robot3_control = rospy.get_param('/motion_controller/robot3_control')
    except Exception as e:
        print("[motion_controller]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/state_observer/robot1_state', State, robot1_state_callback)
    rospy.Subscriber('/state_observer/robot2_state', State, robot2_state_callback)
    rospy.Subscriber('/state_observer/robot3_state', State, robot3_state_callback)
    rospy.Subscriber('/state_observer/robot1_error', State, robot1_error_callback)
    rospy.Subscriber('/state_observer/robot2_error', State, robot2_error_callback)
    rospy.Subscriber('/state_observer/robot3_error', State, robot3_error_callback)
    rospy.Subscriber('/path_planner/robot1_target', State, robot1_target_callback)

    # create ROS publishers
    robot1_publisher = rospy.Publisher(robot1_control, Twist, queue_size=1)
    robot2_publisher = rospy.Publisher(robot2_control, Twist, queue_size=1)
    robot3_publisher = rospy.Publisher(robot3_control, Twist, queue_size=1)

    '''
    robot2_controller = "MOVE"
    robot3_controller = "MOVE"
    robot2_ready = False
    robot3_ready = False
    
    if robot2_ready is True and robot3_ready is True:
        .
        .
        .
    else:
        robot1_command = Twist()
        robot2_command, robot2_controller, robot2_ready = setup_controller(ROBOT2_ERROR, robot2_controller)
        robot3_command, robot3_controller, robot3_ready = setup_controller(ROBOT3_ERROR, robot3_controller)
    '''

    # create a 100Hz timer
    timer = rospy.Rate(TIMER_FREQ)

    while not rospy.is_shutdown():
        # calculate control inputs
        if ROBOT1_STATE is not None and ROBOT1_TARGET is not None:
            robot1_command = controller(ROBOT1_STATE, ROBOT1_TARGET, ROBOT1_ERROR)
            robot2_command = translate(robot1_command, robot2_config, ROBOT2_ERROR)
            robot3_command = translate(robot1_command, robot3_config, ROBOT3_ERROR)
        else:
            robot1_command = Twist()
            robot2_command = Twist()
            robot3_command = Twist()

        # publish control inputs
        robot1_publisher.publish(robot1_command)
        robot2_publisher.publish(robot2_command)
        robot3_publisher.publish(robot3_command)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
