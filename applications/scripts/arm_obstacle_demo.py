#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
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
    rospy.init_node('arm_obstacle_demo')
    wait_for_time()
    argv = rospy.myargv()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeAttachedObject('tray')

    # Create table obstacle
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)

    # Create divider obstacle
    planning_scene.removeCollisionObject('divider')
    size_x = 0.5
    size_y = 0.01
    size_z = 0.05
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = 0
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.5
    pose1.pose.position.y = -0.3
    pose1.pose.position.z = 0.75
    pose1.pose.orientation.w = 1

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.5
    pose2.pose.position.y = 0.3
    pose2.pose.position.z = 0.9
    pose2.pose.orientation.z = 0.2588
    pose2.pose.orientation.w = 0.9659

    arm = fetch_api.Arm()

    def shutdown():
        arm.cancel_all_goals()
        planning_scene.clear()

    rospy.on_shutdown(shutdown)

    kwargs = {
        'allowed_planning_time': 5,
        'execution_timeout': 30,
        'group_name': 'arm',
        'num_planning_attempts': 10,
        'replan': True,
        'replan_attempts': 5,
        'tolerance': 0.001
    }
    error = arm.move_to_pose(pose1, **kwargs)
    if error is not None:
        arm.cancel_all_goals()
        rospy.logerr('Pose 1 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 1 succeeded')
        frame_attached_to = 'gripper_link'
        frames_okay_to_collide_with = [
            'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
        ]
        planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                                 frame_attached_to,
                                 frames_okay_to_collide_with)
        planning_scene.setColor('tray', 1, 0, 1)
        planning_scene.sendColors()

    rospy.sleep(2)

    kwargs = {
        'allowed_planning_time': 120,
        'execution_timeout': 120,
        'group_name': 'arm',
        'num_planning_attempts': 1,
        'replan': True,
        'replan_attempts': 10,
        'tolerance': 0.01
    }
    oc = OrientationConstraint()
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_roll_link'
    oc.orientation.w = 1
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 1.0
    kwargs['orientation_constraint'] = oc
    error = arm.move_to_pose(pose2, **kwargs)
    if error is not None:
        arm.cancel_all_goals()
        rospy.logerr('Pose 2 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 2 succeeded')


if __name__ == '__main__':
    main()
