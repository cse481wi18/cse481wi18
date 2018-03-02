#!/usr/bin/env python

import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

import tf.transformations as tft
import fetch_api
import math
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def _linear_distance(pos1, pos2):
    dx = pos1.x - pos2.x
    dy = pos1.y - pos2.y
    dz = pos1.z - pos2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _yaw_from_quaternion(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    return math.atan2(m[1, 0], m[0, 0])


class Driver(object):
    def __init__(self, base):
        self._base = base
        self._goal = None
        self._updated_goal = False

    def start(self):
        current_goal = copy.deepcopy(self._goal)
        state = 'turn'
        rate = rospy.Rate(25)

        linear_speed = 0
        angular_speed = 0
        while True:
            if self._updated_goal:
                current_goal = copy.deepcopy(self._goal)
                state = 'turn'
                start_pos = copy.deepcopy(self._base.odom.position)
                self._updated_goal = False
            if current_goal is None:
                rate.sleep()
                continue

            current_pose = self._base.odom

            dy = current_goal.y - current_pose.position.y
            dx = current_goal.x - current_pose.position.x
            desired_theta = math.atan2(dy, dx)
            current_theta = _yaw_from_quaternion(current_pose.orientation)
            angle = (desired_theta - current_theta) % (2 * math.pi)
            if 2 * math.pi - angle < angle:
                angle -= 2 * math.pi

            if math.fabs(angle) > 0.01:
                if state == 'turn':
                    linear_speed = 0
                state = 'turn'
            else:
                state = 'move'

            if state == 'turn':
                speed = max(0.25, min(1, math.fabs(angle)))
                direction = 1 if angle > 0 else -1
                angular_speed = direction * speed
            else:
                state = 'move'

            distance_remaining = _linear_distance(current_pose.position, current_goal)
            if state == 'move':
                if math.fabs(distance_remaining) > 0.005:
                    speed = max(0.02, min(0.5, math.fabs(distance_remaining)))
                    direction = 1 if distance_remaining > 0 else -1
                    linear_speed = direction * speed
                    angular_speed = 0
                else:
                    current_goal = None
                    self._goal = None

            self._base.move(linear_speed, angular_speed)
            rate.sleep()

    def set_goal(self, goal):
        self._goal = goal
        self._updated_goal = True


class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        self._server = server
        self._x = x
        self._y = y
        self._name = name
        self._driver = driver

    def start(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'odom'
        int_marker.name = self._name
        int_marker.description = self._name
        int_marker.pose.position.x = self._x
        int_marker.pose.position.y = self._y
        int_marker.pose.orientation.w = 1

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(box_marker)
        int_marker.controls.append(button_control)

        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

    def _callback(self, msg):
        if (msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            interactive_marker = self._server.get(msg.marker_name)
            position = interactive_marker.pose.position
            rospy.loginfo('User clicked {} at {}, {}, {}'.format(
                msg.marker_name, position.x, position.y, position.z))
            self._driver.set_goal(position)


def main():
    rospy.init_node('drive_demo')
    wait_for_time()

    base = fetch_api.Base()
    driver = Driver(base)

    server = InteractiveMarkerServer('simple_marker')
    marker1 = DestinationMarker(server, 2, 2, 'dest1', driver)
    marker2 = DestinationMarker(server, 1, 0, 'dest2', driver)
    marker3 = DestinationMarker(server, 3, -1, 'dest3', driver)
    marker1.start()
    marker2.start()
    marker3.start()

    driver.start()
    rospy.spin()


if __name__ == '__main__':
    main()
