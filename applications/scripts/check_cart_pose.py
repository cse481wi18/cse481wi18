#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications check_cart_pose.py plan X Y Z'
    print '       rosrun applications check_cart_pose.py ik X Y Z'


def main():
    rospy.init_node('check_cart_pose')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 5:
        print_usage()
        return
    command, x, y, z = argv[1], float(argv[2]), float(argv[3]), float(argv[4])

    arm = fetch_api.Arm()
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1

    if command == 'plan':
        error = arm.check_pose(ps, allowed_planning_time=1.0)
        if error is None:
            rospy.loginfo('Found plan!')
        else:
            rospy.loginfo('No plan found.')
        arm.cancel_all_goals()
    elif command == 'ik':
        if arm.compute_ik(ps):
            rospy.loginfo('Found IK!')
        else:
            rospy.loginfo('No IK found.')
    else:
        print_usage()


if __name__ == '__main__':
    main()
