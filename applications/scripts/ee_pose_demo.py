#! /usr/bin/env python

import fetch_api
import rospy
import tf


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()
    argv = rospy.myargv()
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            trans, rot = tf_listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
            rospy.loginfo('{} {}'.format(trans, rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
        rate.sleep()
    


if __name__ == '__main__':
    main()
