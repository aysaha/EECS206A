#!/usr/bin/env python
import sys


class GroupFramePublisher():
    def __init__(self, config):
        from maths_helper import norm, diff

        self.config = config
        self.N = len(self.config)
        self.alpha_weights = [norm(diff(self.config[i], self.config[j])) for i in range(self.N) for j in range(i)]

    def compute_frame(self, translations):
        from maths_helper import barycenter, point2tuple, div, mul, diff

        alpha_sample = [div(diff(self.config[i], self.config[j]), diff(translations[i], translations[j])) for i in range(self.N) for j in range(i)]
        alpha_r, alpha_theta = barycenter(alpha_sample, self.alpha_weights) # non-biased estimate of scale factor in polar coordinates

        bar_z = barycenter(translations)
        bar_c = barycenter(self.config)
        beta = diff(bar_z, mul(bar_c, alpha_r, alpha_theta))    # beta is the center of the group frame, alpha_theta is it orientation

        return beta, alpha_theta

    def publish(frames):
        #Create a twist that holds the last updated frame
        last_transform = TransformStamped()
        last_transform.header.stamp.secs = -1

        pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        #Listen for tf updates
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)  # use the ApproximateTimeSynchronizer from package message_filters (lab 6)?

        # create a 10Hz timer
        timer = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            try:
                # get the transform from usb_cam frame to robot frames
                robot_transforms = [tf_buffer.lookup_transform('usb_cam', frame, rospy.Time()) for frame in frames]
                translations = [point2tuple(z) for z in robot_transforms.transform.translation]

                origin, angle = self.compute_frame()

                last_transform.transform.translation.x = beta[0]
                last_transform.transform.translation.y = beta[1]
                last_transform.transform.translation.z = 0
                last_transform.transform.rotation = tft.quaternion_from_euler(0, 0, alpha_theta)

                if rospy.Time.now().secs - last_transform.header.stamp.secs > 0:
                    raise tf2_ros.LookupException

                else:
                    print("Updating tranform")

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

                if last_transform.header.stamp.secs == -1:
                    print("No previous transform to publish!")
                else:
                    print("Camera view is blocked, publishing most revent available transform")
                    last_transform.header.stamp = rospy.Time.now()
                    tfm = tf2_msgs.msg.TFMessage([last_transform])
                    pub.publish(tfm)

            timer.sleep()



def main(args):
    assert rospy.has_param('~config'), "Could not find configuration in parameter server"
    config = rospy.get_param('~config')

    frames = sys.argv[1:]


    # check for correct number of arguments
    assert len(frame) == len(config), "Parameter server supports {0} robots. Usage: group_frame.py frame_1 frame_2 ... frame_{0}".format(len(config))

    # initialize ROS node
    rospy.init_node('group_frame_publisher', anonymous=True)

    group_frame_publisher = GroupFramePublisher(config)
    
    # run controller
    group_frame_publisher.publish(frames)


if __name__ == '__main__':
    import rospy
    import tf2_ros
    import tf.transformations as tft
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import TransformStamped
    import tf2_msgs.msg

    main(sys.argv)