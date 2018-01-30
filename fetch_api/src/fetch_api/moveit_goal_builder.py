from geometry_msgs.msg import *
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                             OrientationConstraint, BoundingVolume)
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from shape_msgs.msg import SolidPrimitive
from tf.listener import TransformListener
import actionlib
import copy
import moveit_msgs.msg
import rospy
import tf


class MoveItGoalBuilder(object):
    """Builds a MoveGroupGoal.

    Example:
        # To do a reachability check from the current robot pose.
        builder = MoveItGoalBuilder()
        builder.set_pose_goal(pose_stamped)
        builder.allowed_planning_time = 5
        builder.plan_only = True
        goal = builder.build()

        # To move to a current robot pose with a few options changed.
        builder = MoveItGoalBuilder()
        builder.set_pose_goal(pose_stamped)
        builder.replan = True
        builder.replan_attempts = 10
        goal = builder.build()

    Here are the most common class attributes you might set before calling
    build(), and their default values:

    allowed_planning_time: float=10. How much time to allow the planner,
        in seconds.
    group_name: string='arm'. Either 'arm' or 'arm_with_torso'.
    num_planning_attempts: int=1. How many times to compute the same plan (most
        planners are randomized). The shortest plan will be used.
    plan_only: bool=False. Whether to only compute the plan but not execute it.
    replan: bool=False. Whether to come up with a new plan if there was an
        error executing the original plan.
    replan_attempts: int=5. How many times to do replanning.
    replay_delay: float=1. How long to wait in between replanning, in seconds.
    tolerance: float=0.01. The tolerance, in meters for the goal pose.
    """

    def __init__(self):
        self.allowed_planning_time = 10.0
        self.fixed_frame = 'base_link'
        self.gripper_frame = 'wrist_roll_link'
        self.group_name = 'arm'
        self.planning_scene_diff = moveit_msgs.msg.PlanningScene()
        self.planning_scene_diff.is_diff = True
        self.planning_scene_diff.robot_state.is_diff = True
        self.look_around = False
        self.max_acceleration_scaling_factor = 0
        self.max_velocity_scaling_factor = 0
        self.num_planning_attempts = 1
        self.plan_only = False
        self.planner_id = 'RRTConnectkConfigDefault'
        self.replan = False
        self.replan_attempts = 5
        self.replan_delay = 1
        self.start_state = moveit_msgs.msg.RobotState()
        self.start_state.is_diff = True
        self.tolerance = 0.01
        self._orientation_constraints = []
        self._pose_goal = None
        self._joint_names = None
        self._joint_positions = None
        self._tf_listener = TransformListener()

    def set_pose_goal(self, pose_stamped):
        """Sets a pose goal.

        Pose and joint goals are mutually exclusive. The most recently set goal
        wins.

        Args:
            pose_stamped: A geometry_msgs/PoseStamped.
        """
        self._pose_goal = pose_stamped
        self._joint_names = None
        self._joint_positions = None

    def set_joint_goal(self, joint_names, joint_positions):
        """Set a joint-space planning goal.

        Args:
            joint_names: A list of strings. The names of the joints in the goal.
            joint_positions: A list of floats. The joint angles to achieve.
        """
        self._joint_names = joint_names
        self._joint_positions = joint_positions
        self._pose_goal = None

    def add_path_orientation_constraint(self, o_constraint):
        """Adds an orientation constraint to the path.

        Args:
            o_constraint: A moveit_msgs/OrientationConstraint.
        """
        self._orientation_constraints.append(copy.deepcopy(o_constraint))
        self.planner_id = 'RRTConnectkConfigDefault'

    def build(self, tf_timeout=rospy.Duration(5.0)):
        """Builds the goal message.

        To set a pose or joint goal, call set_pose_goal or set_joint_goal
        before calling build. To add a path orientation constraint, call
        add_path_orientation_constraint first.

        Args:
            tf_timeout: rospy.Duration. How long to wait for a TF message. Only
                used with pose goals.

        Returns: moveit_msgs/MoveGroupGoal
        """
        goal = MoveGroupGoal()

        # Set start state
        goal.request.start_state = copy.deepcopy(self.start_state)

        # Set goal constraint
        if self._pose_goal is not None:
            self._tf_listener.waitForTransform(self._pose_goal.header.frame_id,
                                               self.fixed_frame,
                                               rospy.Time.now(), tf_timeout)
            try:
                pose_transformed = self._tf_listener.transformPose(
                    self.fixed_frame, self._pose_goal)
            except (tf.LookupException, tf.ConnectivityException):
                return None
            c1 = Constraints()
            c1.position_constraints.append(PositionConstraint())
            c1.position_constraints[0].header.frame_id = self.fixed_frame
            c1.position_constraints[0].link_name = self.gripper_frame
            b = BoundingVolume()
            s = SolidPrimitive()
            s.dimensions = [self.tolerance * self.tolerance]
            s.type = s.SPHERE
            b.primitives.append(s)
            b.primitive_poses.append(self._pose_goal.pose)
            c1.position_constraints[0].constraint_region = b
            c1.position_constraints[0].weight = 1.0

            c1.orientation_constraints.append(OrientationConstraint())
            c1.orientation_constraints[0].header.frame_id = self.fixed_frame
            c1.orientation_constraints[
                0].orientation = pose_transformed.pose.orientation
            c1.orientation_constraints[0].link_name = self.gripper_frame
            c1.orientation_constraints[
                0].absolute_x_axis_tolerance = self.tolerance
            c1.orientation_constraints[
                0].absolute_y_axis_tolerance = self.tolerance
            c1.orientation_constraints[
                0].absolute_z_axis_tolerance = self.tolerance
            c1.orientation_constraints[0].weight = 1.0

            goal.request.goal_constraints.append(c1)
        elif self._joint_names is not None:
            c1 = Constraints()
            for i in range(len(self._joint_names)):
                c1.joint_constraints.append(JointConstraint())
                c1.joint_constraints[i].joint_name = self._joint_names[i]
                c1.joint_constraints[i].position = self._joint_positions[i]
                c1.joint_constraints[i].tolerance_above = self.tolerance
                c1.joint_constraints[i].tolerance_below = self.tolerance
                c1.joint_constraints[i].weight = 1.0
            goal.request.goal_constraints.append(c1)

        # Set path constraints
        goal.request.path_constraints.orientation_constraints = self._orientation_constraints 

        # Set trajectory constraints

        # Set planner ID (name of motion planner to use)
        goal.request.planner_id = self.planner_id

        # Set group name
        goal.request.group_name = self.group_name

        # Set planning attempts
        goal.request.num_planning_attempts = self.num_planning_attempts

        # Set planning time
        goal.request.allowed_planning_time = self.allowed_planning_time

        # Set scaling factors
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling_factor

        # Set planning scene diff
        goal.planning_options.planning_scene_diff = copy.deepcopy(
            self.planning_scene_diff)

        # Set is plan only
        goal.planning_options.plan_only = self.plan_only

        # Set look around
        goal.planning_options.look_around = self.look_around

        # Set replanning options
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay

        return goal
