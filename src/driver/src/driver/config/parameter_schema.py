"""Parameter parsing helpers for robot_driver."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from rclpy.node import Node


@dataclass
class SafePoseFallback:
    joint_names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    ready: bool = False


@dataclass
class JointCommandConfig:
    mode: str
    rate_limit: float


@dataclass
class DriverParameters:
    can_channel: str
    joint_state_rate: float
    diagnostics_rate: float
    command_timeout_s: float
    zero_gravity_default: bool
    xyz_only_mode: bool
    log_dir: str
    safe_pose_file: str
    robot_description_file: str
    publish_tf: bool
    tf_base_frame: str
    tf_tool_frame: str
    diag_hardware_id: str
    diagnostic_latch_sec: float
    cartesian_command_rate_limit: float
    joint_state_topic: str
    diagnostics_topic: str
    follow_joint_trajectory_action: str
    zero_gravity_service: str
    can_reset_script: str
    joint_command: JointCommandConfig
    enable_joint_velocity_fuse: bool
    safe_pose_fallback: SafePoseFallback


_JOINT_MODES = {"position", "velocity", "effort"}


def declare_and_get_parameters(node: Node) -> DriverParameters:
    """Declare ROS params (with defaults) and bundle them into ``DriverParameters``."""

    def _declare(name: str, default):
        return node.declare_parameter(name, default).value

    joint_cfg = JointCommandConfig(
        mode=_declare('joint_command_mode', 'position'),
        rate_limit=float(_declare('joint_command_rate_limit', 0.5)),
    )
    joint_cfg.mode = joint_cfg.mode.lower()
    if joint_cfg.mode not in _JOINT_MODES:
        raise ValueError(f"joint_command_mode must be one of {_JOINT_MODES}, got {joint_cfg.mode}")

    DEFAULT_SAFE_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    DEFAULT_SAFE_POS = [0.0, -0.3, 0.6, -1.2, 0.8, 0.0]

    # Nested parameters use dot notation in ROS 2 parameter files. Declaring
    # the dotted names ensures YAML snippets like ``safe_pose_fallback: { ... }``
    # map to the expected lists and avoids declaring dictionaries (which ROS 2
    # parameters do not support).
    joint_names = _declare('safe_pose_fallback.joint_names', DEFAULT_SAFE_JOINTS)
    positions = _declare('safe_pose_fallback.positions', DEFAULT_SAFE_POS)
    ready_flag = bool(_declare('safe_pose_fallback.ready', False))

    safe_pose_fallback = SafePoseFallback(
        joint_names=list(joint_names),
        positions=[float(x) for x in positions],
        ready=ready_flag,
    )

    return DriverParameters(
        can_channel=_declare('can_channel', 'can0'),
        joint_state_rate=float(_declare('joint_state_rate', 30.0)),
        diagnostics_rate=float(_declare('diagnostics_rate', 1.0)),
        command_timeout_s=float(_declare('command_timeout_s', 600.0)),
        zero_gravity_default=bool(_declare('zero_gravity_default', False)),
        xyz_only_mode=bool(_declare('xyz_only_mode', False)),
        log_dir=_declare('log_dir', 'log/robot_driver'),
        safe_pose_file=_declare('safe_pose_file', 'config/safe_pose_default.yaml'),
        robot_description_file=_declare('robot_description_file', 'robot_description.yaml'),
        publish_tf=bool(_declare('publish_tf', True)),
        tf_base_frame=_declare('tf_base_frame', 'base_link'),
        tf_tool_frame=_declare('tf_tool_frame', 'tool0'),
        diag_hardware_id=_declare('diag_hardware_id', 'robot_driver_node'),
        diagnostic_latch_sec=float(_declare('diagnostic_latch_sec', 5.0)),
        cartesian_command_rate_limit=float(_declare('cartesian_command_rate_limit', 0.15)),
        joint_state_topic=_declare('joint_state_topic', '/joint_states'),
        diagnostics_topic=_declare('diagnostics_topic', '/robot_driver/diagnostics'),
        follow_joint_trajectory_action=_declare(
            'follow_joint_trajectory_action', '/robot_driver/action/follow_joint_trajectory'
        ),
        zero_gravity_service=_declare('zero_gravity_service', '/robot_driver/service/zero_gravity'),
        can_reset_script=_declare('can_reset_script', ''),
        joint_command=joint_cfg,
        enable_joint_velocity_fuse=bool(_declare('enable_joint_velocity_fuse', False)),
        safe_pose_fallback=safe_pose_fallback,
    )
