#!/usr/bin/env python

import numpy as np
import rospy
import tf
import tf2_ros
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute
from std_srvs.srv import Empty
from balebot.msg import State


N = 3
GOAL_STATE = None
ROBOT1_STATES = []
ROBOT2_STATES = []
ROBOT3_STATES = []
ROBOT1_ERRORS = []
ROBOT2_ERRORS = []
ROBOT3_ERRORS = []
ROBOT1_TARGET = None


def average(states):
    n = len(states)
    x = 0
    y = 0
    theta = 0

    for state in states:
        x += state.x / n
        y += state.y / n
        theta += state.theta / n

    return State(x, y, theta)


def transform(robot_frame, fixed_frame, tf_buffer=None, tf_attempts=10):
    if tf_buffer is None:
        # transform robot frame to fixed frame via rigid body transform
        RT = np.array([[np.cos(fixed_frame.theta), np.sin(fixed_frame.theta)], [-np.sin(fixed_frame.theta), np.cos(fixed_frame.theta)]])
        p = np.array([fixed_frame.x, fixed_frame.y])
        q = np.array([robot_frame.x, robot_frame.y]) 
        q = np.matmul(RT, q) - np.matmul(RT, p)
        x = q[0]
        y = q[1] 
        theta = robot_frame.theta - fixed_frame.theta
    else:
        # get transformation from robot frame to fixed frame
        for attempt in range(tf_attempts):
            try:
                tf_frame = tf_buffer.lookup_transform(fixed_frame, robot_frame, rospy.Time())
                break
            except:
                tf_frame = None

        # convert transformation to state
        if tf_frame is not None:
            x = tf_frame.transform.translation.x
            y = tf_frame.transform.translation.y
            q = [tf_frame.transform.rotation.x, tf_frame.transform.rotation.y, tf_frame.transform.rotation.z, tf_frame.transform.rotation.w]
            theta = tf.transformations.euler_from_quaternion(q)[-1]
        else:
            x = 0
            y = 0
            theta = 0

    return State(x, y, theta)


def robot1_frame_callback(msg):
    global GOAL_STATE, ROBOT1_STATES, ROBOT1_ERRORS

    ROBOT1_STATES.append(transform(msg, GOAL_STATE))

    while len(ROBOT1_STATES) > N:
        ROBOT1_STATES.pop(0)

    ROBOT1_ERRORS.append(State(0, 0, 0))

    while len(ROBOT1_ERRORS) > N:
        ROBOT1_ERRORS.pop(0)


def robot2_frame_callback(msg):
    global GOAL_STATE, ROBOT2_STATES, ROBOT2_ERRORS

    ROBOT2_STATES.append(transform(msg, GOAL_STATE))

    while len(ROBOT2_STATES) > N:
        ROBOT2_STATES.pop(0)
    
    ROBOT2_ERRORS.append(State(0, 0, 0))

    while len(ROBOT2_ERRORS) > N:
        ROBOT2_ERRORS.pop(0)


def robot3_frame_callback(msg):
    global GOAL_STATE, ROBOT3_STATES, ROBOT3_ERRORS

    ROBOT3_STATES.append(transform(msg, GOAL_STATE))

    while len(ROBOT3_STATES) > N:
        ROBOT3_STATES.pop(0)

    ROBOT3_ERRORS.append(State(0, 0, 0))

    while len(ROBOT3_ERRORS) > N:
        ROBOT3_ERRORS.pop(0)


def robot1_target_callback(msg):
    global ROBOT1_TARGET

    ROBOT1_TARGET = msg


def main():
    global N, GOAL_STATE, ROBOT1_STATES, ROBOT2_STATES, ROBOT3_STATES, ROBOT1_ERRORS, ROBOT2_ERRORS, ROBOT3_ERRORS

    # initialize ROS node
    rospy.init_node('state_observer')

    # load data from parameter server
    try:
        simulation = rospy.get_param('/state_observer/simulation')
        goal_frame = rospy.get_param('/state_observer/goal_frame')
        robot1_frame = rospy.get_param('/state_observer/robot1_frame')
        robot2_frame = rospy.get_param('/state_observer/robot2_frame')
        robot3_frame = rospy.get_param('/state_observer/robot3_frame')
        robot2_static = rospy.get_param('/state_observer/robot2_static')
        robot3_static = rospy.get_param('/state_observer/robot3_static')
        robot2_config = rospy.get_param('/motion_controller/robot2_config')
        robot3_config = rospy.get_param('/motion_controller/robot3_config')
    except Exception as e:
        print("[state_observer]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/path_planner/robot1_target', State, robot1_target_callback)

    # create ROS publishers
    robot1_publisher = rospy.Publisher('/state_observer/robot1_state', State, queue_size=1)
    robot2_publisher = rospy.Publisher('/state_observer/robot2_state', State, queue_size=1)
    robot3_publisher = rospy.Publisher('/state_observer/robot3_state', State, queue_size=1)
    static1_publisher = rospy.Publisher('/state_observer/robot1_error', State, queue_size=1)
    static2_publisher = rospy.Publisher('/state_observer/robot2_error', State, queue_size=1)
    static3_publisher = rospy.Publisher('/state_observer/robot3_error', State, queue_size=1)

    if simulation is True:
        # create ROS subscribers
        rospy.Subscriber(robot1_frame, Pose, robot1_frame_callback)
        rospy.Subscriber(robot2_frame, Pose, robot2_frame_callback)
        rospy.Subscriber(robot3_frame, Pose, robot2_frame_callback)

        # define origin
        origin = State(5, 5, 0)

        # define end goal
        goal_frame = [float(val) for val in goal_frame.split(',')]
        GOAL_STATE = State(goal_frame[0], goal_frame[1], goal_frame[2] * np.pi / 180)

        # initialize simulator services
        clear = rospy.ServiceProxy('clear', Empty)
        spawn = rospy.ServiceProxy('spawn', Spawn)
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

        # wait for services
        rospy.wait_for_service('clear')
        rospy.wait_for_service('spawn')
        rospy.wait_for_service('turtle1/teleport_absolute')

        # initialize robots
        teleport(origin.x, origin.y, origin.theta)
        spawn(origin.x, origin.y + robot2_config, origin.theta, 'turtle2')
        spawn(origin.x, origin.y + robot3_config, origin.theta, 'turtle3')
        clear()
    else:
        # create tf buffer primed with a tf listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    robot1_state = None

    while not rospy.is_shutdown():
        if simulation is False:
            ROBOT1_STATES.append(transform(robot1_frame, goal_frame, tf_buffer=tf_buffer))
            ROBOT2_STATES.append(transform(robot2_frame, goal_frame, tf_buffer=tf_buffer))
            ROBOT3_STATES.append(transform(robot3_frame, goal_frame, tf_buffer=tf_buffer))
            
            if robot1_state is not None and ROBOT1_TARGET is not None:
                ROBOT1_ERRORS.append(transform(robot1_state, ROBOT1_TARGET))
            
            ROBOT2_ERRORS.append(transform(robot2_frame, robot2_static, tf_buffer=tf_buffer))
            ROBOT3_ERRORS.append(transform(robot3_frame, robot3_static, tf_buffer=tf_buffer))

        # determine robot 1 state
        while len(ROBOT1_STATES) > N:
            ROBOT1_STATES.pop(0)

        robot1_state = average(ROBOT1_STATES)

        # determine robot 2 state
        while len(ROBOT2_STATES) > N:
            ROBOT2_STATES.pop(0)
        
        robot2_state = average(ROBOT2_STATES)

        # determine robot 3 state
        while len(ROBOT3_STATES) > N:
            ROBOT3_STATES.pop(0)

        robot3_state = average(ROBOT3_STATES)

        # determine robot 1 error
        while len(ROBOT1_ERRORS) > N:
            ROBOT1_ERRORS.pop(0)
        
        robot1_error = average(ROBOT1_ERRORS)

        # determine robot 2 error
        while len(ROBOT2_ERRORS) > N:
            ROBOT2_ERRORS.pop(0)
        
        robot2_error = average(ROBOT2_ERRORS)

        # determine robot 3 error
        while len(ROBOT3_ERRORS) > N:
            ROBOT3_ERRORS.pop(0)

        robot3_error = average(ROBOT3_ERRORS)

        # publish states
        robot1_publisher.publish(robot1_state)
        robot2_publisher.publish(robot2_state)
        robot3_publisher.publish(robot3_state)
        static1_publisher.publish(robot1_error)
        static2_publisher.publish(robot2_error)
        static3_publisher.publish(robot3_error)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
