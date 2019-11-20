#!/usr/bin/env python

import sys
import rospy
import tf2_ros
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
            transform = tf_buffer.lookup_transform(target_frame, source_trame, rospy.Time())

            '''
            STATE
            q = (x, y, theta)

            INPUT
            u = (v, omega)

            KINEMATICS
            x_dot = v*cos(theta)
            y_dot = v*sin(theta)
            theta_dot = omega
            
            CONTROL
            v_d = v_r*cos(theta_r - theta) + K1*(x_r - x)
            omega_d = omega_r - K2*(y_r - y)*v_r*sinc(theta_r - theta) - K3*(theta_r - theta)
            '''

            # generate control command
            K1 = 0.3
            K2 = 1
            command = Twist()
            command.x = K1 * trans.transform.translation.x
            command.z = K2 * trans.transform.translation.y

            # publish control input
            turtlebot.publish(command)
        except tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException:
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
    main(sys.args)
