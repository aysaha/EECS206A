#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import tf
import tf2_ros
from planning.msg import State


DELTA = 0.05


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


def transform(source_frame, target_frame, tf_buffer):
    # get transformation from source frame to target frame
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time()).transform
        except Exception as e:
            print(e)

    # unpack transformation
    x = transform.translation.x
    y = transform.translation.y
    q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    theta = tf.transformations.euler_from_quaternion(q)[-1]
    state = State(x, y, theta)

    return state


def translate(current_waypoint, previous_waypoint, local_frame):
    '''
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
    distance, angle = polar(State(0, 0, 0), State(local_frame[0], local_frame[1], 0))


def plan(initial_state, final_state, K=2, N=25):
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


def main():
    global DELTA

    # get configuration from parameter server
    if rospy.has_param('/path_planner/config'):
        config = rospy.get_param('/path_planner/config')
    else:
        print("Error: could not find [config] in parameter server")
        exit(1)

    # get robot frame from parameter server
    if rospy.has_param('/path_planner/robot_frame'):
        robot_frame = rospy.get_param('/path_planner/robot_frame')
    else:
        print("Error: could not find [robot_frame] in parameter server")
        exit(1)

    # get goal frame from parameter server
    if rospy.has_param('/path_planner/goal_frame'):
        goal_frame = rospy.get_param('/path_planner/goal_frame')
    else:
        print("Error: could not find [goal_frame] in parameter server")
        exit(1)

    # initialize ROS node
    rospy.init_node('path_planner', anonymous=True)

    # create ROS publisher
    publisher = rospy.Publisher('/path_planner/waypoint', State, queue_size=1)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # generate paths to target
    state = transform(robot_frame, goal_frame, tf_buffer)
    path_a = plan(state, State(0, 0, 0), K=1)
    path_b = plan(state, State(0, 0, 0), K=2)
    path_c = plan(state, State(0, 0, 0), K=3)

    # display planned paths
    draw(path_a, "a", render=False)
    draw(path_b, "b", render=False)
    draw(path_c, "c", render=True)

    # let user select path
    while not rospy.is_shutdown():
        response = raw_input("select path [a/b/c/n]: ")

        if response == "a":
            path = path_a
            break
        if response == "b":
            path = path_b
            break
        if response == "c":
            path = path_c
            break
        elif response == "n":
            path = []
            break

    # create a 10Hz timer
    timer = rospy.Rate(10)

    while not rospy.is_shutdown():
        # get the current state of the robot
        state = transform(robot_frame, goal_frame, tf_buffer)

        # check if robot is near current waypoint
        if path:
            distance, angle = polar(state, path[0])

            if distance < DELTA:
                path.pop(0)

        # publish next waypoint
        if path:
            publisher.publish(path[0])

        # synchronize node 
        timer.sleep()


if __name__ == '__main__':
    main()
