#! /usr/bin/env python

import math
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications base_demo.py move 0.1'
    print '       rosrun applications base_demo.py rotate 30'


def main():
    rospy.init_node('base_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 3:
        print_usage()
        return
    command = argv[1]
    value = float(argv[2])

    base = fetch_api.Base()
    if command == 'move':
        base.go_forward(value)
    elif command == 'rotate':
        base.turn(value * math.pi / 180)
    else:
        print_usage()


if __name__ == '__main__':
    main()
