#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def cosd(degs):
    return math.cos(degs * math.pi / 180)


def sind(degs):
    return math.sin(degs * math.pi / 180)


def axis_marker(pose_stamped):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'axes'
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    marker.scale.x = 0.1

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0.8, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(0.8, 0, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0.8, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 0.8, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 0.8, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 0.8, 1))

    return marker


def transform_to_pose(matrix):
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    x, y, z, w = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose


def pose_to_transform(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix


def arrow_marker(point):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'arrow'
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.header.frame_id = 'frame_a'
    marker.points.append(Point(0, 0, 0))
    marker.points.append(point)
    marker.scale.x = 0.1
    marker.scale.y = 0.15
    marker.color.r = 1
    marker.color.g = 1
    marker.color.a = 1
    return marker


def main():
    rospy.init_node('transformation_demo')
    wait_for_time()
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)
    b_in_a = np.array([
        [cosd(45), -sind(45), 0, 0],
        [sind(45), cosd(45), 0, 0],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1]
    ])
    ps = PoseStamped()
    ps.header.frame_id = 'frame_a'
    ps.pose = transform_to_pose(b_in_a)
    viz_pub.publish(axis_marker(ps))

    point_in_b = np.array([1, 0, 0, 1])
    point_in_a = np.dot(b_in_a, point_in_b)
    rospy.loginfo(point_in_b)
    rospy.loginfo(point_in_a)
    point = Point(point_in_a[0], point_in_a[1], point_in_a[2])
    viz_pub.publish(arrow_marker(point))


def main2():
    rospy.init_node('transformation_demo')
    wait_for_time()
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)

    obj_pose_in_base = Pose(Point(0.6, -0.1, 0.7), Quaternion(0, 0, 0.38268343, 0.92387953))
    obj_mat_in_base = pose_to_transform(obj_pose_in_base)

    pregrasp_pose_in_obj = Pose()
    pregrasp_pose_in_obj.position.x = -0.1
    pregrasp_pose_in_obj.orientation.w = 1
    pregrasp_mat_in_obj = pose_to_transform(pregrasp_pose_in_obj)

    pregrasp_mat_in_base = np.dot(obj_mat_in_base, pregrasp_mat_in_obj)
    pregrasp_pose = transform_to_pose(pregrasp_mat_in_base)

    ps1 = PoseStamped()
    ps1.header.frame_id = 'frame_a'
    ps1.pose = obj_pose_in_base

    ps2 = PoseStamped()
    ps2.header.frame_id = 'frame_a'
    ps2.pose = pregrasp_pose

    viz_pub.publish(axis_marker(ps1))
    marker2 = axis_marker(ps2)
    marker2.id = 1
    viz_pub.publish(marker2)


if __name__ == '__main__':
    main2()
