#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist
from planning.msg import State

STATES = ["MOVE", "ADJUST", "STOP", "REVERSE"]
target = State()

def callback(msg):
    global target
    target = msg


def controller(source_frame, target_frame, move=True):

    global STATES, target
    current_state = "REVERSE"

    # create ROS publisher
    turtlebot = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 10Hz timer
    timer = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        try:

            ############################ PARAMETERS ##################################

            
            
            #K1: Speed proportionnality constant
            K1 = 1

            #K2: Angular velocity proportionnality constant
            K2 = -1

            #Tolerance values for distance and angle
            eps_d = 0.03
            eps_theta = 0.03

            #Distance in meters before the turtlebot starts slowing down
            slowdown_threshold = 0.1

            #Turtlebot max speed in meters per sec
            max_speed = K1 * slowdown_threshold

            ########################### GET TRANSFORMATION ##########################3

            '''
            # get the transform from source_frame to target_frame
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            
            theta = tft.euler_from_quaternion(q)[-1]
            '''

            x,y,theta = target.x, target.y, target.theta
            
            
            d = np.sqrt(x*x + y*y)
            theta_r = np.arctan2(-y, -x)
            delta_theta = theta - theta_r

            #Correct the angle singularity
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

            print(target.x)
            

            ################# REVERSE STATE ##################
            if current_state == "REVERSE":
                command.linear.x = -0.1

                if d > 0.2:
                    next_state = "MOVE"


            ################### STOP STATE ####################
            elif current_state == "STOP":
                #Do nothing

                if abs(theta) > eps_theta:
                    next_state = "ADJUST"



            ################# MOVE STATE #####################
            elif current_state == "MOVE":

                #If you're farther than a certain threshhold value, go at max speed
                if d > slowdown_threshold:
                    command.linear.x = max_speed
                    command.angular.z = K2 * delta_theta

                #Then slow down as you get closer
                else:
                    command.linear.x = min(K1 * d,max_speed)
                    command.angular.z = K2 * delta_theta

                #Transition happens zhen closer to goal than tolerance
                if d < eps_d:
                    next_state = "ADJUST"


            ################# ADJUST STATE ##################
            elif current_state == "ADJUST":
                command.linear.x = 0
                command.angular.z = K2 * theta

                if abs(theta) < eps_theta:
                    next_state = "STOP"


            ################# PROBLEM STATE ##################
            else:
                print("WRONG STATE: " + current_state)
                sys.exit(1)
            

            #State machine transition
            current_state = next_state

            if move == "False":
                print(command)
                rospy.sleep(1)
                continue

            # publish control input
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
    
    rospy.Subscriber('/path_planner/waypoint', State, callback)

    # run controller
    if len(args) == 4:
        controller(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        controller(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    main(sys.argv)