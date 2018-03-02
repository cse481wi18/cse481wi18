#! /usr/bin/env python

import fetch_api
import rospy


def print_usage():
    print 'Moves the torso to a certain height between [0.0, 0.4]'
    print 'Usage: rosrun applications torso_demo.py 0.4'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('torso_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    height = float(argv[1])

    torso = fetch_api.Torso()
    torso.set_height(height)


if __name__ == '__main__':
    main()
