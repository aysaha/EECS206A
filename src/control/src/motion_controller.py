#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist

STATES = ["MOVE", "ADJUST", "STOP", "REVERSE"]

def controller(source_frame, target_frame, move=True):

    global STATES
    current_state = "REVERSE"

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

            eps_d = 0.03

            eps_theta = 0.03

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            
            theta = tft.euler_from_quaternion(q)[-1]
            
            #transform.transform.rotation.z
            
            d = np.sqrt(x*x + y*y)
            theta_r = np.arctan2(-y, -x)

            delta_theta = theta - theta_r

            if delta_theta > 3.14:
                delta_theta -= 6.28
            if delta_theta < -3.14:
                delta_theta += 6.28






            command = Twist()

            next_state = current_state
            print(current_state)





            print("theta_r: " + str(theta_r))
            print("distance: " + str(d))
            print("theta: "+ str(theta))

            
            #rospy.sleep(1)
            #continue


            if current_state == "REVERSE":
                command.linear.x = -0.1

                if d > 0.2:
                    next_state = "MOVE"

            elif current_state == "STOP":
                print("Final state")

                if abs(theta) > eps_theta:
                    next_state = "ADJUST"

            elif current_state == "MOVE":
                command.linear.x = K1 * d
                command.angular.z = K2 * delta_theta

                if d < eps_d:
                    next_state = "ADJUST"


            elif current_state == "ADJUST":
                command.linear.x = 0
                command.angular.z = K2 * theta

                if abs(theta) < eps_theta:
                    next_state = "STOP"

            else:
                print("WRONG STATE")
                sys.exit(1)
            

            current_state = next_state
            # publish control input

            if move == "False":
                print(command)
                rospy.sleep(1)
                continue


            turtlebot.publish(command)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        timer.sleep()


def main(args):
    # check for correct number of arguments
    if len(args) < 3:
        print("Usage: motion_controller source_frame target_frame")
        exit(1)

    # initialize ROS node
    rospy.init_node('motion_controller', anonymous=True)
    
    # run controller
    if len(args) == 4:
        controller(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        controller(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    main(sys.argv)