#! /usr/bin/env python

import geometry_msgs.msg
import rospy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._publisher = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=5)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._publisher.publish(twist)

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0, 0)
