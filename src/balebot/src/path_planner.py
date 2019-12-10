#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
from balebot.msg import State
from state_observer import transform

DELTA = 0.15
CURVE = 2
POINTS = 12
START_STATE = None
ROBOT1_STATE = None
ROBOT2_STATE = None
GROUP_STATE = None


def draw(path, label, render=True):
    # plot path
    x = [point.x for point in path]
    y = [point.y for point in path]
    plt.plot(x, y, label=label, linestyle='dashed', marker='o')

    # render graph
    if render is True:
        plt.title('State Trajectory')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.show()


def polar(source_state, target_state):
    # use the source state as the reference
    x = target_state.x - source_state.x
    y = target_state.y - source_state.y
    
    # convert from Cartesian to polar coordinates
    distance = np.sqrt(np.power(x, 2) + np.power(y, 2))
    angle = np.arctan2(y, x)

    return distance, angle


def plan(initial_state, final_state, K=2, N=10):
    path = []
    
    # unpack state vectors
    xy_i = np.array([initial_state.x, initial_state.y])
    theta_i = initial_state.theta
    xy_f = np.array([final_state.x, final_state.y])
    theta_f = final_state.theta
 
    # calculate polynomial coefficients
    alpha = np.array([K * np.cos(theta_f), K * np.sin(theta_f)]) - 3 * xy_f
    beta = np.array([K * np.cos(theta_i), K * np.sin(theta_i)]) + 3 * xy_i

    # create path with N points
    for i in range(N):
        s = float(i) / float(N - 1)
        point = s**3 * xy_f + s**2 * (s - 1) * alpha + s * (s - 1)**2 * beta - (s - 1)**3 * xy_i

        if i == 0:
            heading = theta_i
        elif i == N - 1:
            heading = theta_f
        else:
            distance, angle = polar(path[-1], State(point[0], point[1], 0))
            heading = angle + theta_f

        path.append(State(point[0], point[1], heading))

    return path


def start_state_callback(msg):
    global START_STATE

    START_STATE = msg


def robot1_state_callback(msg):
    global ROBOT1_STATE

    ROBOT1_STATE = msg


def robot2_state_callback(msg):
    global ROBOT2_STATE

    ROBOT2_STATE = msg


def group_state_callback(msg):
    global GROUP_STATE

    GROUP_STATE = msg


def main():
    global DELTA, CURVE, POINTS, ROBOT1_STATE, ROBOT2_STATE

    # initialize ROS node
    rospy.init_node('path_planner')
    
    # load data from parameter server
    try:
        start_frame = rospy.get_param('/state_observer/start_frame')
        end_frame = rospy.get_param('/state_observer/end_frame')
        robot1_config = rospy.get_param('/path_planner/robot1_config')
        robot2_config = rospy.get_param('/path_planner/robot2_config')
    except Exception as e:
        print("[path_planner]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscribers
    rospy.Subscriber('/state_observer/start_state', State, start_state_callback)
    rospy.Subscriber('/state_observer/robot1_state', State, robot1_state_callback)
    rospy.Subscriber('/state_observer/robot2_state', State, robot2_state_callback)
    rospy.Subscriber('/state_observer/group_state', State, group_state_callback)

    # create ROS publisher
    robot1_publisher = rospy.Publisher('/path_planner/robot1_target', State, queue_size=1)
    robot2_publisher = rospy.Publisher('/path_planner/robot2_target', State, queue_size=1)
    group_publisher = rospy.Publisher('/path_planner/group_target', State, queue_size=1)

    # wait for accurate state estimates
    rospy.sleep(1)

    # generate paths to target
    robot1_path = plan(ROBOT1_STATE, State(0, 0, 0), K=CURVE, N=POINTS)
    #group_path = plan(START_STATE, State(0, 0, 0), K=CURVE, N=POINTS)

    # display planned paths
    draw(robot1_path, "robot1")

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        if robot1_path and ROBOT1_STATE is not None:
            distance, angle = polar(ROBOT1_STATE, robot1_path[0])

            if distance < DELTA:
                print("[path_planner]: robot1 reached waypoint " + str(POINTS + 1 - len(robot1_path)))
                robot1_path.pop(0)

        # publish next waypoint
        if robot1_path:
            robot1_publisher.publish(robot1_path[0])

        # synchronize node 
        timer.sleep()


if __name__ == '__main__':
    main()
