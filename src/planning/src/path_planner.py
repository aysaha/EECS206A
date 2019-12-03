#!/usr/bin/env python

import numpy as np
import rospy
import tf
import tf2_ros
#from planning.msg import State


PATH = []
DELTA = 0.01


def planner(initial_state, final_state, K=10, N=100):
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
            heading = np.arctan2(point[1] - path[-1].y, point[0] - path[-1].x)
        else:
            heading = theta_i

        path.append(State(point[0], point[1], heading))

    # force trajectory to converge
    path.append(State(0, 0, 0))

    return path


def translate():
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
    pass


def distance(source_state, target_state):
    # calculate Euclidean distance between states
    x = target_state.x - source_state.x
    y = target_state.y - source_state.y
    d = np.sqrt(np.power(x, 2) + np.power(y, 2))

    return d


def transform(source_frame, target_frame, tf_buffer):
    # get transformation from source frame to target frame
    transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time()).transform

    # unpack transformation
    x = transform.translation.x
    y = transform.translation.y
    q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    theta = tf.transformations.euler_from_quaternion(q)[-1]
    state = State(x, y, theta)

    return state


def callback(msg):
    global PATH

    # treat target as the fixed frame
    state_i = State(-msg.x, -msg.y, -msg.theta)
    state_f = State(0, 0, 0)

    # generate path to target
    PATH = planner(state_i, state_f)


def main():
    global PATH
    global DELTA

    # get configuration from parameter server
    if rospy.has_param('~config'):
        config = rospy.get_param('~config')
    else:
        print("Error: could not find 'config' in parameter server")
        exit(1)

    # get robot frame from parameter server
    if rospy.has_param('~robot_frame'):
        robot_frame = rospy.get_param('~robot_frame')
    else:
        print("Error: could not find 'robot_frame' in parameter server")
        exit(1)

    # get goal frame from parameter server
    if rospy.has_param('~goal_frame'):
        goal_frame = rospy.get_param('~goal_frame')
    else:
        print("Error: could not find 'goal_frame' in parameter server")
        exit(1)

    # initialize ROS node
    rospy.init_node('path_planner', anonymous=True)

    # create ROS subscriber
    subcriber = rospy.Subscriber('/path_planner/target', State, callback)

    # create ROS publisher
    publisher = rospy.Publisher('/path_planner/waypoint', State, queue_size=1)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 1Hz timer
    timer = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            # get the current state of the robot
            state = transform(robot_frame, goal_frame, tf_buffer)

            # check if robot is near current waypoint
            if path:
                if distance(state, path[0]) < DELTA:
                    path.pop(0)

            # determine next waypoint
            if path:
                waypoint = path[0]
            else:
                waypoint = state

            publisher.publish(waypoint)
            timer.sleep()
        except:
            pass


if __name__ == '__main__':
    main()
