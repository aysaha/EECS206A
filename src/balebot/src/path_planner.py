#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
from balebot.msg import State


DELTA = 0.05
STATE = None


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
    for i in range(N - 1):
        s = float(i) / float(N - 1)
        point = s**3 * xy_f + s**2 * (s - 1) * alpha + s * (s - 1)**2 * beta - (s - 1)**3 * xy_i

        if i > 0:
            distance, angle = polar(path[-1], State(point[0], point[1], 0))
            heading = angle
        else:
            heading = theta_i

        path.append(State(point[0], point[1], heading))

    # force trajectory to converge
    path.append(State(0, 0, 0))

    return path


def callback(msg):
    global STATE
    
    STATE = msg


def main():
    global DELTA, STATE

    # initialize ROS node
    rospy.init_node('path_planner')

    # create ROS subscriber
    rospy.Subscriber('/state_observer/state', State, callback)

    # create ROS publisher
    publisher = rospy.Publisher('/path_planner/target', State, queue_size=1)

    # wait for a valid state
    while STATE is None:
        pass

    # generate paths to target
    print('---------------')
    print("x: " + str(STATE.x))
    print("y: " + str(STATE.y))
    print("theta: " + str(STATE.theta * 180 / np.pi))
    print('---------------')
    path = plan(STATE, State(0, 0, 0))

    # display planned paths
    draw(path, "waypoint")
    exit(0)

    # create a 100Hz timer
    timer = rospy.Rate(100)

    while not rospy.is_shutdown():
        # check if robot is near current waypoint
        if path:
            distance, angle = polar(STATE, path[0])

            if distance < DELTA:
                print("[path_planner]: reached waypoint " + str(11 - len(path)))
                path.pop(0)

        # publish next waypoint
        if path:
            publisher.publish(path[0])

        # synchronize node 
        timer.sleep()


if __name__ == '__main__':
    main()
