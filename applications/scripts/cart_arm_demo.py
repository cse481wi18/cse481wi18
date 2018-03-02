#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
import fetch_api
import rospy
import sys


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('cart_arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.042
    pose1.pose.position.y = 0.384
    pose1.pose.position.z = 1.826
    pose1.pose.orientation.x = 0.173
    pose1.pose.orientation.y = -0.693
    pose1.pose.orientation.z = -0.242
    pose1.pose.orientation.w = 0.657

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.047
    pose2.pose.position.y = 0.545
    pose2.pose.position.z = 1.822
    pose2.pose.orientation.x = -0.274
    pose2.pose.orientation.y = -0.701
    pose2.pose.orientation.z = 0.173
    pose2.pose.orientation.w = 0.635

    gripper_poses = [pose1, pose2]

    arm = fetch_api.Arm()

    def on_shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(on_shutdown)

    while not rospy.is_shutdown():
        for pose_stamped in gripper_poses:
            error = arm.move_to_pose(pose_stamped, replan=True, execution_timeout=2)
            if error is not None:
                rospy.logerr(error)
            rospy.sleep(1)


if __name__ == '__main__':
    main()
