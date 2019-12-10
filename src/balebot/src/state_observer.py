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
START_STATE = None
END_STATE = None
ROBOT1_STATES = []
ROBOT2_STATES = []


def random(minimum, maximum):
    return (maximum - minimum) * np.random.random_sample() + minimum


def rotate(x, y, theta):
    x_prime = x * np.cos(theta) - y * np.sin(theta)
    y_prime = x * np.sin(theta) + y * np.cos(theta)

    return x_prime, y_prime


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


def transform(robot_frame, fixed_frame, tf_buffer=None):
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
        while not rospy.is_shutdown():
            try:
                tf_frame = tf_buffer.lookup_transform(fixed_frame, robot_frame, rospy.Time())
                break
            except:
                pass
    
        # convert transformation to state
        y = tf_frame.transform.translation.y
        x = tf_frame.transform.translation.x
        q = [tf_frame.transform.rotation.x, tf_frame.transform.rotation.y, tf_frame.transform.rotation.z, tf_frame.transform.rotation.w]
        theta = tf.transformations.euler_from_quaternion(q)[-1]

    return State(x, y, theta)


def robot1_frame_callback(msg):
    global END_STATE, ROBOT1_STATES

    while len(ROBOT1_STATES) > N:
        ROBOT1_STATES.pop(0)

    ROBOT1_STATES.append(transform(msg, END_STATE, tf_buffer=None))


def robot2_frame_callback(msg):
    global END_STATE, ROBOT2_STATES
    
    while len(ROBOT2_STATES) > N:
        ROBOT2_STATES.pop(0)

    ROBOT2_STATES.append(transform(msg, END_STATE, tf_buffer=None))


def main():
    global N, START_STATE, END_STATE, ROBOT1_STATES, ROBOT2_STATES

    # initialize ROS node
    rospy.init_node('state_observer')
    
    # load data from parameter server
    try:
        simulation = rospy.get_param('/state_observer/simulation')
        start_frame = rospy.get_param('/state_observer/start_frame')
        end_frame = rospy.get_param('/state_observer/end_frame')
        robot1_frame = rospy.get_param('/state_observer/robot1_frame')
        robot2_frame = rospy.get_param('/state_observer/robot2_frame')
        robot1_config = rospy.get_param('/path_planner/robot1_config')
        robot2_config = rospy.get_param('/path_planner/robot2_config')
    except Exception as e:
        print("[state_observer]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS publishers
    start_publisher = rospy.Publisher('/state_observer/start_state', State, queue_size=1)
    robot1_publisher = rospy.Publisher('/state_observer/robot1_state', State, queue_size=1)
    robot2_publisher = rospy.Publisher('/state_observer/robot2_state', State, queue_size=1)
    group_publisher = rospy.Publisher('/state_observer/group_state', State, queue_size=1)

    if simulation is True:
        # create ROS subscribers
        rospy.Subscriber(robot1_frame, Pose, robot1_frame_callback)
        rospy.Subscriber(robot2_frame, Pose, robot2_frame_callback)
        
        # define start state
        start_frame = [float(val) for val in start_frame.split(',')]
        START_STATE = State(start_frame[0], start_frame[1], start_frame[2] * np.pi / 180)
        
        # define end state
        end_frame = [float(val) for val in end_frame.split(',')]
        END_STATE = State(end_frame[0], end_frame[1], end_frame[2] * np.pi / 180)

        # initialize simulator services
        clear = rospy.ServiceProxy('clear', Empty)
        spawn = rospy.ServiceProxy('spawn', Spawn)
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

        # wait for services
        rospy.wait_for_service('clear')
        rospy.wait_for_service('spawn')
        rospy.wait_for_service('turtle1/teleport_absolute')

        # initialize simulator
        #teleport(random(1, 9), random(1, 9), random(-np.pi, np.pi))
        #spawn(random(1, 9), random(1, 9), random(-np.pi, np.pi), 'turtle2')

        # initialize robot 1
        x, y = rotate(0, robot1_config, START_STATE.theta)
        teleport(x + START_STATE.x, y + START_STATE.y, START_STATE.theta)
        
        # initialize robot 2
        x, y = rotate(0, robot2_config, START_STATE.theta)
        spawn(x + START_STATE.x, y + START_STATE.y, START_STATE.theta, 'turtle2')
        
        clear()
    else:
        # create tf buffer primed with a tf listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        if simulation is True:
            start_state = transform(START_STATE, END_STATE, tf_buffer=None)
        else:
            start_state = transform(start_frame, end_frame, tf_buffer=tf_buffer)
            ROBOT1_STATES.append(transform(robot1_frame, end_frame, tf_buffer=tf_buffer))
            ROBOT2_STATES.append(transform(robot2_frame, end_frame, tf_buffer=tf_buffer))

        # determine robot 1 state
        while len(ROBOT1_STATES) > N:
            ROBOT1_STATES.pop(0)

        robot1_state = average(ROBOT1_STATES)
        
        # determine robot 2 state
        while len(ROBOT2_STATES) > N:
            ROBOT2_STATES.pop(0)
        
        robot2_state = average(ROBOT2_STATES)

        # determine group state
        x = (robot1_state.x + robot2_state.x) / 2
        y = (robot1_state.y - robot1_config + robot2_state.y - robot2_config) / 2
        theta = (robot1_state.theta + robot2_state.theta) / 2
        group_state = State(x , y, theta)

        # publish states
        start_publisher.publish(start_state)
        robot1_publisher.publish(robot1_state)
        robot2_publisher.publish(robot2_state)
        group_publisher.publish(group_state)

        # synchronize node
        timer.sleep()


if __name__ == '__main__':
    main()
