from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

MIN_DISTANCE = 0.0131 # Distance between fingers when gripper is closed

def gripper_interactive_marker(pose_stamped, finger_distance):
    gripper_marker = Marker()
    gripper_marker.pose.position.x = 0.166
    gripper_marker.pose.orientation.w = 1
    gripper_marker.type = Marker.MESH_RESOURCE
    gripper_marker.mesh_resource = GRIPPER_MESH
    gripper_marker.mesh_use_embedded_materials = True

    finger_distance = max(0, min(finger_distance, 0.1))

    l_marker = Marker()
    l_marker.pose.position.x = 0.166
    l_marker.pose.position.y = finger_distance / 2.0 - 0.1
    l_marker.pose.orientation.w = 1
    l_marker.type = Marker.MESH_RESOURCE
    l_marker.mesh_resource = L_FINGER_MESH
    l_marker.mesh_use_embedded_materials = True

    r_marker = Marker()
    r_marker.pose.position.x = 0.166
    r_marker.pose.position.y = -finger_distance / 2.0 + 0.1
    r_marker.pose.orientation.w = 1
    r_marker.type = Marker.MESH_RESOURCE
    r_marker.mesh_resource = R_FINGER_MESH
    r_marker.mesh_use_embedded_materials = True

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    control.markers.append(gripper_marker)
    control.markers.append(l_marker)
    control.markers.append(r_marker)

    interactive_marker = InteractiveMarker()
    interactive_marker.header = pose_stamped.header
    interactive_marker.pose = pose_stamped.pose
    interactive_marker.controls.append(control)
    interactive_marker.scale = 0.25

    return interactive_marker
