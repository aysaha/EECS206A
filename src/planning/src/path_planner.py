#!/usr/bin/env python

import sys
import numpy as np
import rospy
from planning.msg import State

'''
def plan(destination, steps=1000, alpha=1):
    waypoints = []
    curr = np.array([0.0, 0.0])
    prev = np.array([-1.0, -0.0])

    print("destination = " + str(destination))
    x = destination[0]
    y = destination[1]

    for i in range(steps):
        # velocity vector
        theta = np.arctan2(-y, -x)
        mag = np.sqrt(x * x + y * y) / steps
        v = np.array([mag * np.cos(theta), mag * np.sin(theta)])

        # momentum vector
        m = curr - prev
        m = m / np.sqrt(m[0] * m[0] + m[1] * m[1]) / steps

        vect = alpha * v + (1 - alpha) * m
        waypoints.append(curr + vect)

        prev = curr
        curr = curr + vect

    for waypoint in waypoints:
        print(str(waypoint[0]) + " " + str(waypoint[1]))

    return waypoints
'''

def planner(state, K=10, N=100):
    # create ROS publisher
    publisher = rospy.Publisher('/path_planner/waypoint', State, queue_size=N)

    # get configuration from parameter server
    if rospy.has_param('~config'):
        config = rospy.get_param('~config')
    else:
        print("Error: could not find configuration in parameter server")
        exit(1)

    # generate global path
    path = []

    x = float(state[0])
    y = float(state[1])
    theta = np.radians(float(state[2]))

    state = np.array([x, y])
    alpha = np.array([K * np.cos(theta) - 3 * x, K * np.sin(theta) - 3 * y])
    beta = np.array([K, 0])

    for i in range(N):
        s = float(i) / float(N)
        waypoint = s**3 * state + s**2 * (s - 1) * alpha + s * (s - 1)**2 * beta
        path.append(State(waypoint[0], waypoint[1], 0))

    '''
    path = zeros(2, steps);
    x = state(1);
    y = state(2);
    theta = state(3);
    
    alpha = [K*cos(theta) - 3*x; K*sin(theta) - 3*y];
    beta = [K; 0];
    
    for i = 1:steps
        s = i / steps;
        path(:, i) = s^3 * [x; y] + s^2 * (s - 1) * alpha + s * (s - 1)^2 * beta;
    end
    ----------------------------------------------------------------------------------
    config = [0; 1];
    steps = 100;
    K = 25;

    path_A = path_planner(target, steps, K);

    path_B = zeros(2, steps);
    d = norm(config);
    theta = atan2(config(2), config(1));

    for i = 1:steps
        if i == 1
            m = path_A(:, i);
        else
            m = path_A(:, i) - path_A(:, i-1);
        end
        
        phi = theta + atan2(m(2), m(1));
        v = d * [cos(phi); sin(phi)];
        
        path_B(:, i) = path_A(:, i) + v;
    end
    '''

    # publish entire path
    for waypoint in path:
        print(waypoint)
        publisher.publish(waypoint)


def main(args):
    # check for correct number of arguments
    if len(args) < 4:
        print("Usage: path_planner x y theta")
        exit(1)

    # initialize ROS node
    rospy.init_node('path_planner', anonymous=True)

    planner((sys.argv[1], sys.argv[2], sys.argv[3]))


if __name__ == '__main__':
    main(sys.argv)