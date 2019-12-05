#!/usr/bin/env python
import sys
import traceback


class GroupFramePublisher():
    def __init__(self, config):
        from maths_helper import norm, diff

        self.config = config
        self.N = len(self.config)
        assert self.N >= 1, "You need to have at least one robot"
        if self.N > 1:
            self.alpha_weights = [norm(diff(self.config[i], self.config[j])) for i in range(self.N) for j in range(i)]
            print(self.alpha_weights)

    def init(self, frames):
        self._frames = frames
        self._translations = {} # Saves the estimated robot positions in real_time
        self._odom = {} # Caches odoms in real time
        self._odom_offset_cache = {}    # Save absolute position - odom farme position at the moments when we loose the AR tag

        self.pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        #Listen for tf updates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.timer = rospy.Rate(10) # create a 10Hz timer


    def compute_frame(self, translations):
        from maths_helper import barycenter, div, mul, diff

        try:
            alpha_sample = [div(diff(self.config[i], self.config[j]), diff(translations[i], translations[j])) for i in range(self.N) for j in range(i)]
        except ZeroDivisionError:
            print("At least 2 robot frames have the same coordinates")
            exit(1)

        alpha_r, alpha_theta = barycenter(alpha_sample, self.alpha_weights) # estimate of scale factor in polar coordinates

        bar_z = barycenter(translations)
        bar_c = barycenter(self.config)
        beta = diff(bar_z, mul(bar_c, alpha_r, alpha_theta))    # beta is the center of the group frame, alpha_theta is it orientation

        return beta, alpha_theta

    def update_translation(self, frame):
        try:
            transform = self.tf_buffer.lookup_transform('usb_cam', frame, rospy.Time())
            odom_transform =self. tf_buffer.lookup_transform('usb_cam', 'odom', rospy.Time())
            odom_translation = point2tuple(odom_transform.transform.translation)

            if transform.header.stamp.secs != rospy.Time.now().secs:
                # AR tag not in sight anymore
                if frame not in self._odom_offset_cache:
                    if frame not in self._translations:
                        print("AR tag {} is not visible at initial time".format(frame))
                        exit(1)
                    self._odom_offset_cache[frame] = diff(self._translations[frame], self._odom.get(frame, odom_translation))    # default value in case the camera never saw the AR tags
                
                self._translations[frame] = add(self._odom_offset_cache[frame], odom_translation)
            else:
                if frame in self._odom_offset_cache:
                    del self._odom_offset_cache[frame]
                self._translations[frame] = point2tuple(transform.transform.translation)

            self._odom = point2tuple(odom_transform.tranform.translation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            traceback.print_exc(file=sys.stdout)


    def publish(self):
        while not rospy.is_shutdown():
            for frame in self._frames:
                self.update_translation(frame)

            transform = TransformStamped()

            if self.N > 1:
                origin, angle = self.compute_frame(self.translations.values())

                transform.transform.translation.x = beta[0]
                transform.transform.translation.y = beta[1]
                transform.transform.translation.z = 0
                transform.transform.rotation = tft.quaternion_from_euler(0, 0, alpha_theta)  # probably in tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
            else:
                transform = self._translations[self._frames[0]]
                cfg = self.config[0]
                transform.transform.translation.x -= cfg[0]
                transform.transform.translation.y -= cfg[1]

            transform.header.stamp = rospy.Time.now()
            tfm = tf2_msgs.msg.TFMessage([transform])
            self.pub.publish(tfm)        

            self.timer.sleep()



def main(args):
    # initialize ROS node
    rospy.init_node('group_frame_publisher', anonymous=True)

    assert rospy.has_param('~config'), "Could not find configuration in parameter server"
    config = eval(rospy.get_param('~config'))
    print("Config: {}".format(config))

    assert rospy.has_param('~robot_frames'), "Error: could not find 'robot_frame' in parameter server"
    robot_frames = rospy.get_param('~robot_frames')
    print("Robot frames: {}".format(robot_frames))

    # check for correct number of arguments
    assert len(robot_frames) == len(config), "Parameter server config supports {0} robots, not {1}".format(len(config), len(robot_frames))

    group_frame_publisher = GroupFramePublisher(config)
    group_frame_publisher.init(robot_frames)
    
    # run controller
    group_frame_publisher.publish()


if __name__ == '__main__':
    import rospy
    import tf2_ros
    import tf.transformations as tft
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import TransformStamped
    import tf2_msgs.msg
    from maths_helper import point2tuple, diff, add

    main(sys.argv)