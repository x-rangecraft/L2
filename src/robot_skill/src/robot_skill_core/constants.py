"""Centralized constants for robot_skill components."""

from __future__ import annotations

SKILL_ACTION_TOPIC = '/robot_skill/action/skill_sequence'
JOINT_NAME_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
SUPPORTED_STEP_TYPES = frozenset({'safety_pose', 'cartesian_move', 'gripper', 'joint_move'})

DEFAULT_ACTION_TIMEOUT_SEC = 30.0  # seconds
DEFAULT_SPEED_SCALE = 1.0  # ratio applied when per-step speed not provided


__all__ = [
    'SKILL_ACTION_TOPIC',
    'JOINT_NAME_ORDER',
    'SUPPORTED_STEP_TYPES',
    'DEFAULT_ACTION_TIMEOUT_SEC',
    'DEFAULT_SPEED_SCALE',
]
