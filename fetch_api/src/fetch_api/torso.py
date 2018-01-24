#!/usr/bin/env python

import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg

ACTION_SERVER = 'torso_controller/follow_joint_trajectory'
TORSO_JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        height = min(height, 0.4)
        height = max(height, 0.0)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(TORSO_JOINT_NAME)
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(height)
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))
