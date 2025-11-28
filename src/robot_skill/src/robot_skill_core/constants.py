"""Centralized constants for robot_skill components."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped

# Action topics
SKILL_ACTION_TOPIC = '/robot_skill/action/skill_sequence'
GRASP_RECORD_ACTION_TOPIC = '/robot_skill/action/grasp_record'

# Joint configuration
JOINT_NAME_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Supported step types
SUPPORTED_STEP_TYPES = frozenset({'safety_pose', 'cartesian_move', 'gripper', 'joint_move'})

# Timing and speed defaults
DEFAULT_ACTION_TIMEOUT_SEC = 30.0  # seconds
DEFAULT_SPEED_SCALE = 1.0  # ratio applied when per-step speed not provided

# Grasp planning defaults
DEFAULT_APPROACH_DISTANCE = 0.08  # meters (8cm above grasp point)
DEFAULT_GRIPPER_WIDTH_MARGIN = 0.02  # meters (2cm margin for gripper width)
DEFAULT_MAX_GRIPPER_WIDTH = 0.094  # meters (94mm max opening)


def get_observe_pose() -> PoseStamped:
    """
    Get the fixed observation pose for object inspection.

    This pose positions the end-effector in front of the camera for object scanning.
    Values are from move_record_observe in general_skill.yaml.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = 0.03586991293867343
    pose.pose.position.y = 0.3111905019299687
    pose.pose.position.z = 0.4428649697650363
    pose.pose.orientation.x = -0.5268686399645887
    pose.pose.orientation.y = -0.15210254136154186
    pose.pose.orientation.z = -0.20206340031608086
    pose.pose.orientation.w = 0.8114460150779765
    return pose


# Pre-computed observe pose for convenience
OBSERVE_POSE = get_observe_pose()


__all__ = [
    'SKILL_ACTION_TOPIC',
    'GRASP_RECORD_ACTION_TOPIC',
    'JOINT_NAME_ORDER',
    'SUPPORTED_STEP_TYPES',
    'DEFAULT_ACTION_TIMEOUT_SEC',
    'DEFAULT_SPEED_SCALE',
    'DEFAULT_APPROACH_DISTANCE',
    'DEFAULT_GRIPPER_WIDTH_MARGIN',
    'DEFAULT_MAX_GRIPPER_WIDTH',
    'OBSERVE_POSE',
    'get_observe_pose',
]
