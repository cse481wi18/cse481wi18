#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import math
import rospy

from .arm_joints import ArmJoints

ACTION_SERVER = 'arm_controller/follow_joint_trajectory'
TIME_FROM_START = 5


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))

    def move_to_joints(self, joint_state):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(ArmJoints.names())
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.extend(joint_state.values())
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))
