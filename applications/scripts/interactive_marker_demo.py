#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class DestinationMarker(object):
    def __init__(self, server, x, y, name, goal_pub):
        self._server = server
        self._x = x
        self._y = y
        self._name = name
        self._goal_pub = goal_pub

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
            rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))
            self._goal_pub.publish(position)


def main():
    rospy.init_node('marker_demo')
    wait_for_time()
    pub = rospy.Publisher('odometry_goal', Point, queue_size=5)
    rospy.sleep(0.5)
    server = InteractiveMarkerServer('simple_marker')
    marker1 = DestinationMarker(server, 2, 2, 'dest1', pub)
    marker2 = DestinationMarker(server, 1, 0, 'dest2', pub)
    marker3 = DestinationMarker(server, 3, -1, 'dest3', pub)
    marker1.start()
    marker2.start()
    marker3.start()

    rospy.spin()


if __name__ == '__main__':
    main()
