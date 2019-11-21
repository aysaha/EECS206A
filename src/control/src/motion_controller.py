#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist


def controller(source_frame, target_frame):
    # create ROS publisher
    turtlebot = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 10Hz timer
    timer = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
            # get the transform from source_frame to target_frame
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            
            K1 = 0.3
            K2 = -1
            eps_d = 0.05
            eps_theta = 0.05
            alpha = 0.2
            beta = 0.2

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            theta = tft.euler_from_quaternion(q)[-1]
            
            transform.transform.rotation.z
            
            d = np.sqrt(x*x + y*y)
            theta_r = np.arctan2(-y, -x)
            delta_theta = theta - theta_r

            command = Twist()

            '''
            if d > eps_d and np.abs(delta_theta) > eps_theta:`
                command.linear.x = 0
                command.angular.z = K2 * delta_theta
            elif d > eps_d:
                command.linear.x = K1 * d
                command.angular.z = K2 * delta_theta
                #command.angular.z = 0
            else:
                command.linear.x = 0
                command.angular.z = K2 * theta
            '''

            def f(x):
                return (np.arctan(alpha * x - beta) + np.pi / 2) / np.pi

            command.linear.x = K1 * d
            command.angular.z = -1 * f(d) * delta_theta

            #print("theta = " + str(theta * 180 / np.pi))
            #print("theta_r = " + str(theta_r * 180 / np.pi))
            #print("delta theta = " + str((theta-theta_r) * 180 / np.pi))
            #print("\n")

            # publish control input
            turtlebot.publish(command)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        timer.sleep()


def main(args):
    # check for correct number of arguments
    if len(args) != 3:
        print("Usage: motion_controller source_frame target_frame")
        exit(1)

    # initialize ROS node
    rospy.init_node('motion_controller', anonymous=True)
    
    # run controller
    controller(sys.argv[1], sys.argv[2])


if __name__ == '__main__':
    main(sys.argv)
