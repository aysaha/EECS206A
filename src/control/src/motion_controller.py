#!/usr/bin/env python

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from planning.msg import State

STATES = ["MOVE", "ADJUST", "STOP", "REVERSE"]
state = None
target = None
last_target = State()
last_theta = None

integral = 0


def callback(msg):
    global target
    target = msg


def state_callback(msg):
    global state
    state = msg

def controller(source_frame, target_frame, move=True):

    global STATES, state, target, last_target, integral, TIMER_FREQ, last_theta
    current_state = "MOVE"

    # create ROS publisher
    turtlebot = rospy.Publisher('/pink/cmd_vel_mux/input/teleop', Twist, queue_size=1)

    # create tf buffer primed with a tf listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create a 100Hz timer
    TIMER_FREQ = 100
    timer = rospy.Rate(TIMER_FREQ)

    while not rospy.is_shutdown():
        try:

            ############################ PARAMETERS ##################################
            
            #K1: Speed proportionnality constant
            K1 = 2

            #K2: Angular velocity proportionnality constant
            Kp = -4
            Ki = -2

            Kd = 0
            cap = .2

            #Tolerance values for distance and angle
            eps_d = 0.1
            eps_theta = 0.03

            #Distance in meters before the turtlebot starts slowing down
            slowdown_threshold = 0.05

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

            ######### Get next waypoint target ##########
            x,y,theta = (0, 0, 0)

            if state is None:
            	#print("state is None")
                continue
            else:
                x = state.x
                y = state.y
                theta = state.theta


          	if target is None:
          		#print("target is None")
          		continue 

            #Integral reset if tqrget has changed
            if target.x != last_target.x:
                print("Reset integral!")
                integral = 0
                last_target = target
            

            x_w,y_w,theta_w = target.x, target.y, target.theta
            last_waypoint = abs(x_w) < 0.01 and abs(y_w) < 0.01
            #print("Last wp " + str(last_waypoint))

            x = x - x_w
            y = y - y_w
            theta = theta - theta_w
            

            d = np.sqrt(x*x + y*y)
            #theta_r = np.arctan2(-y, -x)
        
            #delta_theta = theta - theta_w

            integral = integral + theta / TIMER_FREQ
            if integral > cap:
                integral = cap
            if integral < -cap:
                integral = - cap

            if last_theta != None:
                derivate = (theta - last_theta)*TIMER_FREQ
            else:
                derivate = None
            last_theta = theta


            #Correct the angle singularity
            '''
            if delta_theta > 3.14:
                delta_theta -= 6.28
                print("singularity detected")
            if delta_theta < -3.14:
                delta_theta += 6.28
                print("singularity detected")
			'''


            command = Twist()

            next_state = current_state
            #print(current_state)

            #print(target)

            print("theta: " + str(theta))
            #print("distance: " + str(d))
            #print("integral: " + str(integral))

            #print(target.x)
            

            ################# REVERSE STATE ##################
            if current_state == "REVERSE":
                command.linear.x = -0.1

                if d > 0.2:
                    next_state = "MOVE"


            ################### STOP STATE ####################
            elif current_state == "STOP":
                pass
                #Do nothing


            ################# MOVE STATE #####################
            elif current_state == "MOVE":

                #If you're farther than a certain threshhold value, go at max speed
                if d > slowdown_threshold or (not last_waypoint):
                    command.linear.x = max_speed
                    command.angular.z = Kp * theta + Ki * integral

                #Then slow down as you get closer
                else:
                    command.linear.x = min(K1 * d,max_speed)
                    command.angular.z = Kp * theta #+ Ki * integral

                #Transition happens when closer to goal than tolerance
                if d < eps_d and last_waypoint:
                    next_state = "ADJUST"


            ################# ADJUST STATE ##################
            elif current_state == "ADJUST":
                command.linear.x = 0
                command.angular.z = Ki * theta

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

        except Exception as e:
            print(e)
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
    rospy.Subscriber('/state_observer/state', State, state_callback)

    # run controller
    if len(args) == 4:
        controller(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        controller(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    main(sys.argv)
