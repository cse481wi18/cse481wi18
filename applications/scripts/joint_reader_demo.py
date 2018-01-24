#! /usr/bin/env python

import fetch_api
from joint_state_reader import JointStateReader
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_reader_demo')
    wait_for_time()
    argv = rospy.myargv()
    reader = JointStateReader()
    rospy.sleep(0.5)
    names = fetch_api.ArmJoints.names()
    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{}'.format(k, v)

    other_joints = ['torso_lift_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
    for j in other_joints:
        val = reader.get_joint(j)
        print '{}\t{}'.format(j, val)
    

if __name__ == '__main__':
    main()
