#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from planning.msg import State
from geometry_msgs.msg import *
from nav_msgs.msg import *
import numpy as np 
from math import *

current_odm_frame = None
last_odm_frame = None

def avg_states(states_list):
    avg_x = 0
    avg y = 0
    avg_theta = 0

    for s in states_list:
        avg_x += s.x/len(states_list)
        avg_y += s.y/len(states_list)
        avg_theta += s.theta/len(states_list)

    return State(avg_x, avg_y, avg_theta)

def rotate(x, y, theta):
    x = x * np.cos(theta) - y * sin(theta)
    y = x * np.sin(theta) + y * cos(theta)

    return x, y 

def odom_callback(msg):
    global current_odm_frame

    secs = msg.header.stamp.secs
    nsecs = msg.header.stamp.nsecs

    pose = msg.pose
    #print("x_odom : ", +str(pose.pose.position.x))
    #print("y_odom : ", +str(pose.pose.position.y))
    x = round(pose.pose.position.x, 2)
    y = round(pose.pose.position.y, 2)
    q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    theta = round(tf.transformations.euler_from_quaternion(q)[-1], 2)
    state = State(x, y, theta)
    current_odm_frame = state


def transform(source_frame, target_frame, tf_buffer):
    # get transformation from source frame to target frame
    try:
        frame = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time.now(), rospy.Duration(1))
    except:
        frame = None

    # convert transformation to State
    if frame is not None:
        y = frame.transform.translation.y
        x = frame.transform.translation.x
        q = [frame.transform.rotation.x, frame.transform.rotation.y, frame.transform.rotation.z, frame.transform.rotation.w]
        theta = tf.transformations.euler_from_quaternion(q)[-1]
        state = State(x, y, theta)
    else:
        state = None

    return state


def main():
    global last_odm_frame, current_odm_frame

    # initialize ROS node
    rospy.init_node('state_observer', anonymous=True)

    #Markers
    robot_frame = "ar_marker_2"
    goal_frame = "ar_marker_8"

    # create ROS publisher
    publisher = rospy.Publisher('/state_observer/state1', State, queue_size=1)

    #Subscribe to the odom frame
    #sub_odom = rospy.Subscriber("/pink/odom", Odometry, odom_callback)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 10Hz timer
    timer = rospy.Rate(10)
    prev_state = None
    odm_vector = None
    cam_vector = None
    
    avg_x = 0
    avg_y = 0 
    avg_theta = 0
    i = 0

    while not rospy.is_shutdown():
        i += 1

        # get the current state of the robot
        state = transform(robot_frame, goal_frame, tf_buffer)

        '''
        if current_odm_frame is not None and state is not None:
            if last_odm_frame is not None:
                x, y = rotate(current_odm_frame.x - last_odm_frame.x,
                              current_odm_frame.y - last_odm_frame.y,
                              state.theta)
                theta = current_odm_frame.theta - last_odm_frame.theta
                odm_vector = State(x, y, theta)

            last_odm_frame = current_odm_frame
        '''

        # publish state
        if state is not None:
            avg_x += state.x
            avg_y += state.y
            avg_theta += state.theta

            
            if i % 10 == 0:
                avg_x /= 10
                avg_y /= 10
                avg_theta /= 10
                publisher.publish(State(avg_x, avg_y, avg_theta * 180 / np.pi))
                avg_x = 0
                avg_y = 0 
                avg_theta = 0

            #if prev_state is not None:
            #    cam_vector = State(state.x - prev_state.x, state.y - prev_state.y, state.theta - prev_state.theta)

            #prev_state = state

        #print('---------------')
        #print("cam_vector:")
        #print(cam_vector)
        #print("odm_vector:")
        #print(odm_vector)

        # synchronize thread
        timer.sleep()


if __name__ == '__main__':
    main()
