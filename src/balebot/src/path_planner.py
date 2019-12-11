#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from balebot.msg import State


K = 2
N = 12
DELTA = 0.15
ROBOT1_STATE = None


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


def plan(initial_state, final_state, K=2, N=12):
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


def robot1_state_callback(msg):
    global ROBOT1_STATE

    ROBOT1_STATE = msg


def main():
    global K, N, DELTA, ROBOT1_STATE

    # initialize ROS node
    rospy.init_node('path_planner')

    # load data from parameter server
    try:
        goal_frame = rospy.get_param('/state_observer/goal_frame')
    except Exception as e:
        print("[path_planner]: could not find " + str(e) + " in parameter server")
        exit(1)

    # create ROS subscriber
    rospy.Subscriber('/state_observer/robot1_state', State, robot1_state_callback)

    # create ROS publishers
    robot1_publisher = rospy.Publisher('/path_planner/robot1_target', State, queue_size=1)
    info_publisher = rospy.Publisher('/path_planner/info', String, queue_size=1)

    # wait for valid state
    while ROBOT1_STATE is None:
        pass

    # wait for accurate state estimate
    rospy.sleep(1)

    # generate path to target
    robot1_path = plan(ROBOT1_STATE, State(0, 0, 0), K=K, N=N)

    # display planned paths
    draw(robot1_path, "robot1")

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        if robot1_path and ROBOT1_STATE is not None:
            distance, angle = polar(ROBOT1_STATE, robot1_path[0])

            if distance < DELTA:
                info_publisher.publish("robot1 reached waypoint " + str(N + 1 - len(robot1_path)))
                robot1_path.pop(0)

        # publish next waypoint
        if robot1_path:
            robot1_publisher.publish(robot1_path[0])

        # synchronize node 
        timer.sleep()


if __name__ == '__main__':
    main()
