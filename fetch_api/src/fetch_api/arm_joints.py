#!/usr/bin/env python

import math
import rospy

DEGS_TO_RADS = math.pi / 180


class ArmJoints(object):
    """ArmJoints holds the positions of the Fetch's arm joints.

    When setting values, it also enforces joint limits.

    js = ArmJoints()
    js.set_shoulder_pan(1.5)
    js.set_shoulder_lift(-0.6)
    js.set_upperarm_roll(3.0)
    js.set_elbow_flex(1.0)
    js.set_forearm_roll(3.0)
    js.set_wrist_flex(1.0)
    js.set_wrist_roll(3.0)
    """
    JOINT_LIMITS = {
        'shoulder_pan': (-92 * DEGS_TO_RADS, 92 * DEGS_TO_RADS),
        'shoulder_lift': (-70 * DEGS_TO_RADS, 87 * DEGS_TO_RADS),
        'elbow_flex': (-129 * DEGS_TO_RADS, 129 * DEGS_TO_RADS),
        'wrist_flex': (-125 * DEGS_TO_RADS, 125 * DEGS_TO_RADS)
    }

    def __init__(self):
        self.shoulder_pan = 0
        self.shoulder_lift = 0
        self.upperarm_roll = 0
        self.elbow_flex = 0
        self.forearm_roll = 0
        self.wrist_flex = 0
        self.wrist_roll = 0

    @staticmethod
    def from_list(vals):
        if len(vals) != 7:
            rospy.logerr('Need 7 values to create ArmJoints (got {})'.format(
                len(vals)))
            return None
        j = ArmJoints()
        j.set_shoulder_pan(vals[0])
        j.set_shoulder_lift(vals[1])
        j.set_upperarm_roll(vals[2])
        j.set_elbow_flex(vals[3])
        j.set_forearm_roll(vals[4])
        j.set_wrist_flex(vals[5])
        j.set_wrist_roll(vals[6])
        return j

    @staticmethod
    def names():
        return [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
            'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint',
            'wrist_roll_joint'
        ]

    def values(self):
        return [
            self.shoulder_pan, self.shoulder_lift, self.upperarm_roll,
            self.elbow_flex, self.forearm_roll, self.wrist_flex,
            self.wrist_roll
        ]

    def _clamp_val(self, val, joint_name):
        if joint_name in ArmJoints.JOINT_LIMITS:
            limits = ArmJoints.JOINT_LIMITS[joint_name]
            min_val, max_val = limits[0], limits[1]
            final_val = min(max(val, min_val), max_val)
            if val != final_val:
                rospy.logwarn('{} not in [{}, {}] for {} joint.'.format(
                    val, min_val, max_val, joint_name))
            return final_val
        else:
            return val

    def set_shoulder_pan(self, val):
        val = self._clamp_val(val, 'shoulder_pan')
        self.shoulder_pan = val

    def set_shoulder_lift(self, val):
        val = self._clamp_val(val, 'shoulder_lift')
        self.shoulder_lift = val

    def set_upperarm_roll(self, val):
        val = self._clamp_val(val, 'upperarm_roll')
        self.upperarm_roll = val

    def set_elbow_flex(self, val):
        val = self._clamp_val(val, 'elbow_flex')
        self.elbow_flex = val

    def set_forearm_roll(self, val):
        val = self._clamp_val(val, 'forearm_roll')
        self.forearm_roll = val

    def set_wrist_flex(self, val):
        val = self._clamp_val(val, 'wrist_flex')
        self.wrist_flex = val

    def set_wrist_roll(self, val):
        val = self._clamp_val(val, 'wrist_roll')
        self.wrist_roll = val
