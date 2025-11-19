"""Compatibility shim; routed to reorganized modules."""
from importlib import import_module

_alias_map = {
    'robot_driver_node': 'driver.app.robot_driver_node',
    'hardware_commander': 'driver.hardware.hardware_commander',
    'can_interface_manager': 'driver.hardware.can_interface_manager',
    'robot_description_loader': 'driver.hardware.robot_description_loader',
    'safe_pose_loader': 'driver.config.safe_pose_loader',
    'command_watchdog': 'driver.control.command_watchdog',
    'motion_controller': 'driver.control.motion_controller',
    'joint_controler': 'driver.control.joint_controler',
    'state_publisher': 'driver.control.state_publisher',
    'parameter_schema': 'driver.config.parameter_schema',
    'safe_pose_executor': 'driver.safety.safe_pose_executor',
    'path_utils': 'driver.utils.path_utils',
    'logging_utils': 'driver.utils.logging_utils',
}

_target = _alias_map[__name__.split('.')[-1]]
module = import_module(_target)

globals().update(module.__dict__)
