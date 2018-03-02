#! /usr/bin/env python
"""An example of using straight_move_to_pose.
"""

from geometry_msgs.msg import PoseStamped
import fetch_api
import moveit_commander
import rospy
import sys


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('straight_gripper_motion_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)

    # Set the torso height before running this demo.
    #torso = fetch_api.Torso()
    #torso.set_height(0.4)

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.502
    pose1.pose.position.y = -0.510
    pose1.pose.position.z = 1.084
    pose1.pose.orientation.x = -0.498
    pose1.pose.orientation.y = 0.508
    pose1.pose.orientation.z = 0.475
    pose1.pose.orientation.w = 0.519

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.502
    pose2.pose.position.y = 0.510
    pose2.pose.position.z = 1.084
    pose2.pose.orientation.x = 0.522
    pose2.pose.orientation.y = -0.484
    pose2.pose.orientation.z = -0.499
    pose2.pose.orientation.w = -0.494

    gripper_poses = [pose1, pose2]

    arm = fetch_api.Arm()
    while not rospy.is_shutdown():
        for pose_stamped in gripper_poses:
            error = arm.straight_move_to_pose(group, pose_stamped, jump_threshold=2.0)
            if error is not None:
                rospy.logerr(error)
            rospy.sleep(0.25)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
