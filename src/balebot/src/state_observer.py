#!/usr/bin/env python

import numpy as np
import rospy
import tf
import tf2_ros

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute
from std_srvs.srv import Empty
from balebot.msg import State as _State
from collections import deque

class State(_State):
    def __add__(self, other):
        return State(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __radd__(self, other):  # sum() needs radd to add to 0
        if other == 0:
            return self
        
        return self + other
        
    def __div__(self, N):
        return State(self.x/N, self.y/N, self.theta/N)



def average(states):
    N = len(states)
    if N == 0:
        return State(0, 0, 0)

    return sum(states)/N


def transform(robot_frame, fixed_frame, tf_buffer=None, tf_attempts=10):
    if tf_buffer is None:
        # transform robot frame to fixed frame via rigid body transform
        RT = np.array([[np.cos(fixed_frame.theta), np.sin(fixed_frame.theta)], [-np.sin(fixed_frame.theta), np.cos(fixed_frame.theta)]])
        p = np.array([fixed_frame.x, fixed_frame.y])
        q = np.array([robot_frame.x, robot_frame.y]) 
        q = np.matmul(RT, q) - np.matmul(RT, p)
        x = q[0]
        y = q[1] 
        theta = robot_frame.theta - fixed_frame.theta
    else:
        # get transformation from robot frame to fixed frame
        for attempt in range(tf_attempts):
            try:
                tf_frame = tf_buffer.lookup_transform(fixed_frame, robot_frame, rospy.Time())
                break
            except:
                tf_frame = None

        # convert transformation to state
        if tf_frame is not None:
            x = tf_frame.transform.translation.x
            y = tf_frame.transform.translation.y
            q = [tf_frame.transform.rotation.x, tf_frame.transform.rotation.y, tf_frame.transform.rotation.z, tf_frame.transform.rotation.w]
            theta = tf.transformations.euler_from_quaternion(q)[-1]
        else:
            x = 0
            y = 0
            theta = 0

    return State(x, y, theta)


class StateObserver():
    def __init__(self, robot_count=3):
        self.N = robot_count
        self.ROBOT_RANGE = list(range(1, self.N+1))
        self.goal_state = None
        self.robot_states = {i: deque() for i in self.ROBOT_RANGE}
        self.robot_errors = {i: deque() for i in self.ROBOT_RANGE}
        self.robot1_target = None

    def robot_frame_callback(self, robot_no):
        def callback(self, msg):
            self.robot_states[robot_no].append(transform(msg, self.goal_state))

            while len(self.robot_states[robot_no]) > self.N:
                self.robot_states[robot_no].popleft()
            
            self.robot_errors[robot_no].append(State(0, 0, 0))

            while len(self.robot_errors[2]) > N:
                self.robot_errors[robot_no].popleft()

        return callback

    def robot1_target_callback(self, msg):
        self.robot1_target = msg

    def init_rospy(self):
        # initialize ROS node
        rospy.init_node('state_observer')

        # load data from parameter server
        try:
            self.simulation = rospy.get_param('/state_observer/simulation')
            self.goal_frame = rospy.get_param('/state_observer/goal_frame')
            self.robot_frames = {idx: rospy.get_param('/state_observer/robot{}_frame'.format(idx)) for idx in self.ROBOT_RANGE}
            self.robot_statics = {idx: rospy.get_param('/state_observer/robot{}_frame'.format(idx)) for idx in self.ROBOT_RANGE}
            self.robot_configs = {idx: rospy.get_param('/motion_controller/robot{}_config'.format(idx)) for idx in self.ROBOT_RANGE if idx != 1}
        except Exception as e:
            print("[state_observer]: could not find " + str(e) + " in parameter server")
            exit(1)

        # create ROS subscribers
        rospy.Subscriber('/path_planner/robot1_target', State, self.robot1_target_callback)

        # create ROS publishers
        self.robot_publishers = {idx: rospy.Publisher('/state_observer/robot{}_state'.format(idx), State, queue_size=1) for idx in self.ROBOT_RANGE}
        self.static_publishers = {idx: rospy.Publisher('/state_observer/robot{}_error'.format(idx), State, queue_size=1) for idx in self.ROBOT_RANGE}

        if self.simulation is True:
            # create ROS subscribers
            for idx in self.ROBOT_RANGE:
                rospy.Subscriber(self.robot_frames[idx], Pose, self.robot_frame_callback[idx])

            # define origin
            origin = State(5, 5, 0)

            # define end goal
            self.goal_frame = [float(val) for val in self.goal_frame.split(',')]
            self.goal_state = State(goal_frame[0], goal_frame[1], goal_frame[2] * np.pi / 180)

            # initialize simulator services
            clear = rospy.ServiceProxy('clear', Empty)
            spawn = rospy.ServiceProxy('spawn', Spawn)
            teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

            # wait for services
            rospy.wait_for_service('clear')
            rospy.wait_for_service('spawn')
            rospy.wait_for_service('turtle1/teleport_absolute')

            # initialize robots
            teleport(origin.x, origin.y, origin.theta)
            spawn(origin.x, origin.y + robot2_config, origin.theta, 'turtle2')
            spawn(origin.x, origin.y + robot3_config, origin.theta, 'turtle3')
            clear()
        else:
            # create tf buffer primed with a tf listener
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def run(self):
        # create a 100Hz timer
        timer = rospy.Rate(100)

        current_robot_states = {idx: None for idx in self.ROBOT_RANGE}
        current_robot_errors = {idx: None for idx in self.ROBOT_RANGE}

        while not rospy.is_shutdown():
            if self.simulation is False:
                for idx in self.ROBOT_RANGE:
                    self.robot_states[idx].append(transform(self.robot_frames[idx], self.goal_frame, tf_buffer=self.tf_buffer))
                
                if current_robot_states[1] is not None and self.robot1_target is not None:
                    self.robot_errors[1].append(transform(current_robot_states[1], self.robot1_target))
                
                for idx in self.ROBOT_RANGE:
                    if idx !=1:
                        self.robot_errors[idx].append(transform(self.robot_frames[idx], self.robot_statics[idx], tf_buffer=self.tf_buffer))

            for idx in self.ROBOT_RANGE:
                # determine robot idx state
                while len(self.robot_states[idx]) > self.N:
                    self.robot_states[idx].popleft()

                current_robot_states[idx] = average(self.robot_states[idx])

                # determine robot idx error
                while len(self.robot_errors[1]) > self.N:
                    self.robot_errors[idx].popleft()
                
                current_robot_errors[idx] = average(self.robot_errors[idx])

                # publish states
                self.robot_publishers[idx].publish(current_robot_states[idx])
                self.static_publishers[idx].publish(current_robot_errors[idx])

            # synchronize node
            timer.sleep()


if __name__ == '__main__':
    state_observer = StateObserver()
    state_observer.init_rospy()
    state_observer.run()
