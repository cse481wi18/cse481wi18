#! /usr/bin/env python

import copy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MenuEntry
import fetch_api
import numpy as np
import rospy
import tf.transformations as tft


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def make_6dof_controls():
    """Returns a list of 6 InteractiveMarkerControls
    """
    controls = []
    # Add 6 DOF controls
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.name = 'rotate_x'
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.name = 'move_x'
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.name = 'rotate_z'
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.name = 'move_z'
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.name = 'rotate_y'
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.name = 'move_y'
    controls.append(copy.deepcopy(control))

    return controls


def color_gripper(gripper_im, r, g, b, a):
    for marker in gripper_im.controls[0].markers:
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a


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


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # Get base interactive marker
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = 0.5
        ps.pose.position.y = 0
        ps.pose.position.z = 0.5
        ps.pose.orientation.w = 1
        gripper_im = fetch_api.gripper_interactive_marker(ps, 0.1)
        gripper_im.name = 'gripper'

        # Add menu
        gripper_im.controls[0].interaction_mode = InteractiveMarkerControl.MENU
        menu_entry = MenuEntry()
        menu_entry.command_type = MenuEntry.FEEDBACK

        menu_entry.id = 1
        menu_entry.title = 'Move gripper here'
        gripper_im.menu_entries.append(copy.deepcopy(menu_entry))

        menu_entry.id = 2
        menu_entry.title = 'Open gripper'
        gripper_im.menu_entries.append(copy.deepcopy(menu_entry))

        menu_entry.id = 3
        menu_entry.title = 'Close gripper'
        gripper_im.menu_entries.append(copy.deepcopy(menu_entry))

        # Add 6 DOF controls
        gripper_im.controls.extend(make_6dof_controls())

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

        self._check_ik(ps)

    def handle_feedback(self, feedback):
        if feedback.marker_name != 'gripper':
            return
        gripper_im = self._im_server.get('gripper')
        ps = PoseStamped()
        ps.header = gripper_im.header
        ps.pose = gripper_im.pose

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                error = self._arm.move_to_pose(ps)
                if error is not None:
                    rospy.logerr(error)
            elif feedback.menu_entry_id == 2:
                self._gripper.open()
            elif feedback.menu_entry_id == 3:
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._check_ik(ps)

    def _check_ik(self, pose_stamped):
        gripper_im = self._im_server.get('gripper')
        joints = self._arm.compute_ik(pose_stamped)
        if joints == False:
            color_gripper(gripper_im, 1, 0, 0, 1)
        else:
            color_gripper(gripper_im, 0, 1, 0, 1)
        self._im_server.insert(gripper_im)
        self._im_server.applyChanges()


class PickPlaceTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_marker = Marker()
        obj_marker.type = Marker.CUBE
        obj_marker.pose.orientation.w = 1
        obj_marker.scale.x = 0.05
        obj_marker.scale.y = 0.05
        obj_marker.scale.z = 0.05
        obj_marker.color.r = 1
        obj_marker.color.g = 1
        obj_marker.color.a = 0.7

        obj_control = InteractiveMarkerControl()
        obj_control.interaction_mode = InteractiveMarkerControl.MENU
        obj_control.always_visible = True
        obj_control.markers.append(obj_marker)

        menu_entry1 = MenuEntry()
        menu_entry1.id = 1
        menu_entry1.title = 'Pick from front'
        menu_entry1.command_type = MenuEntry.FEEDBACK
        menu_entry2 = MenuEntry()
        menu_entry2.id = 2
        menu_entry2.title = 'Open gripper'
        menu_entry2.command_type = MenuEntry.FEEDBACK

        obj_im = InteractiveMarker()
        obj_im.name = 'object'
        obj_im.header.frame_id = 'base_link'
        obj_im.pose.position.x = 0.7
        obj_im.pose.position.z = 0.5
        obj_im.pose.orientation.w = 1
        obj_im.scale = 0.25
        obj_im.menu_entries.append(menu_entry1)
        obj_im.menu_entries.append(menu_entry2)
        obj_im.controls.append(obj_control)
        obj_im.controls.extend(make_6dof_controls())

        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        self._update_grippers()

    def handle_feedback(self, feedback):
        if feedback.marker_name != 'object':
            return
        obj_im = self._im_server.get('object')
        ps = PoseStamped()
        ps.header = obj_im.header
        ps.pose = obj_im.pose

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                pregrasp_im = self._im_server.get('pregrasp')
                pregrasp_ps = PoseStamped()
                pregrasp_ps.header = pregrasp_im.header
                pregrasp_ps.pose = pregrasp_im.pose

                grasp_im = self._im_server.get('grasp')
                grasp_ps = PoseStamped()
                grasp_ps.header = grasp_im.header
                grasp_ps.pose = grasp_im.pose

                lift_im = self._im_server.get('lift')
                lift_ps = PoseStamped()
                lift_ps.header = lift_im.header
                lift_ps.pose = lift_im.pose

                error = self._arm.move_to_pose(pregrasp_ps)
                if error is not None:
                    rospy.logerr(error)
                    return
                error = self._arm.move_to_pose(grasp_ps)
                if error is not None:
                    rospy.logerr(error)
                    return
                self._gripper.close()
                error = self._arm.move_to_pose(lift_ps)
                if error is not None:
                    rospy.logerr(error)
                    return
            elif feedback.menu_entry_id == 2:
                self._gripper.open()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._update_grippers()

    def _compute_grasp_offset(self, obj_ps, grasp_tf):
        """Computes a pose offset from the object.

        Args:
            obj_ps: PoseStamped, the pose of the object.
            grasp_tf: np.array, 4x4 matrix that specifies the transform from the object

        Returns: PoseStamped, the resulting pose.
        """
        obj_tf = pose_to_transform(obj_ps.pose)
        final_grasp_tf = np.dot(obj_tf, grasp_tf)
        result = PoseStamped()
        result.header = obj_ps.header
        result.pose = transform_to_pose(final_grasp_tf)
        return result

    def _update_grippers(self):
        obj_im = self._im_server.get('object')
        obj_ps = PoseStamped()
        obj_ps.header = obj_im.header
        obj_ps.pose = obj_im.pose

        # yapf: disable
        pregrasp_in_obj = np.array([
            [1, 0, 0, -0.266],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        grasp_in_obj = np.array([
            [1, 0, 0, -0.166],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        lift_in_obj = np.array([
            [1, 0, 0, -0.166],
            [0, 1, 0, 0],
            [0, 0, 1, 0.2],
            [0, 0, 0, 1],
        ])
        # yapf: enable

        pregrasp_pose = self._compute_grasp_offset(obj_ps, pregrasp_in_obj)
        pregrasp_im = fetch_api.gripper_interactive_marker(pregrasp_pose, 0.1)
        pregrasp_im.name = 'pregrasp'
        joints = self._arm.compute_ik(pregrasp_pose)
        if joints == False:
            color_gripper(pregrasp_im, 1, 0, 0, 0.5)
        else:
            color_gripper(pregrasp_im, 0, 1, 0, 0.5)

        grasp_pose = self._compute_grasp_offset(obj_ps, grasp_in_obj)
        grasp_im = fetch_api.gripper_interactive_marker(grasp_pose, 0.1)
        grasp_im.name = 'grasp'
        joints = self._arm.compute_ik(grasp_pose)
        if joints == False:
            color_gripper(grasp_im, 1, 0, 0, 0.5)
        else:
            color_gripper(grasp_im, 0, 1, 0, 0.5)

        lift_pose = self._compute_grasp_offset(obj_ps, lift_in_obj)
        lift_im = fetch_api.gripper_interactive_marker(lift_pose, 0.1)
        lift_im.name = 'lift'
        joints = self._arm.compute_ik(lift_pose)
        if joints == False:
            color_gripper(lift_im, 1, 0, 0, 0.5)
        else:
            color_gripper(lift_im, 0, 1, 0, 0.5)

        self._im_server.insert(pregrasp_im)
        self._im_server.insert(grasp_im)
        self._im_server.insert(lift_im)
        self._im_server.applyChanges()


def main():
    rospy.init_node('gripper_teleop')
    wait_for_time()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    teleop.start()

    pp_im_server = InteractiveMarkerServer('pick_place_im_server', q_size=2)
    pick_place = PickPlaceTeleop(arm, gripper, pp_im_server)
    pick_place.start()

    rospy.spin()


if __name__ == '__main__':
    main()
