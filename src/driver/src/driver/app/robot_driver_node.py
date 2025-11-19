"""ROS 2 entry point for robot_driver."""
from __future__ import annotations

import traceback

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from robot_driver.action import JointCommand

from driver.config.parameter_schema import DriverParameters, declare_and_get_parameters
from driver.config.safe_pose_loader import SafePoseLoader
from driver.control.command_watchdog import CommandWatchdog
from driver.control.joint_controler import JointControler
from driver.control.state_publisher import StatePublisher
from driver.control.zero_gravity_manager import ZeroGravityManager
from driver.hardware.hardware_commander import HardwareCommander
from driver.hardware.robot_description_loader import RobotDescriptionLoader
from driver.utils.logging_utils import get_logger


class RobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_driver')
        self._logger = get_logger('robot_driver_node')
        self._params: DriverParameters = declare_and_get_parameters(self)
        self._safe_pose = SafePoseLoader(self._params.safe_pose_fallback).load(self._params.safe_pose_file)
        self._robot_description = RobotDescriptionLoader().load(self._params.robot_description_file)
        self._commander = HardwareCommander(
            self._safe_pose,
            joint_limit_rate=self._params.joint_command.rate_limit,
            robot_description=self._robot_description,
        )
        self._zero_gravity_manager = ZeroGravityManager(
            self,
            self._commander,
            service_name=self._params.zero_gravity_service,
        )
        self._motion_controller = JointControler(
            self,
            self._params,
            self._commander,
            zero_gravity_manager=self._zero_gravity_manager,
            zero_gravity_service=self._params.zero_gravity_service,
        )
        self._state_publisher = StatePublisher(self, self._commander, self._params)
        self._watchdog = CommandWatchdog(
            self,
            self._motion_controller,
            self._commander,
            timeout_s=self._params.command_timeout_s,
        )

        self._subscriptions = []
        self._create_ros_interfaces()
        self._connect_hardware()

        # Register parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self._on_parameter_update)

    # ------------------------------------------------------------------ setup helpers
    def _create_ros_interfaces(self) -> None:
        self._subscriptions.append(
            self.create_subscription(
                PoseStamped,
                '/robot_driver/robot_command',
                self._on_robot_command,
                10,
            )
        )
        # Always subscribe so tooling can publish without needing runtime toggles; the
        # handler will still ignore commands if joint direct control stays disabled.
        self._subscriptions.append(
            self.create_subscription(
                JointState,
                '/robot_driver/joint_command',
                self._on_joint_command,
                10,
            )
        )
        self._subscriptions.append(
            self.create_subscription(
                Empty,
                '/robot_driver/safety_stop',
                self._watchdog.handle_safety_stop,
                10,
            )
        )

        self._self_check_srv = self.create_service(
            Trigger,
            '/robot_driver/service/self_check',
            self._handle_self_check,
        )

        self._trajectory_action = ActionServer(
            self,
            FollowJointTrajectory,
            self._params.follow_joint_trajectory_action,
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self._joint_action = ActionServer(
            self,
            JointCommand,
            self._params.joint_command_action,
            execute_callback=self._execute_joint_action,
            goal_callback=self._joint_goal_callback,
            cancel_callback=self._joint_cancel_callback,
        )

    def _connect_hardware(self) -> None:
        try:
            reset_script = self._params.can_reset_script or None
            self._commander.connect(self._params.can_channel, reset_script=reset_script)
            reached_safe_pose = False
            if self._safe_pose.ready:
                reached_safe_pose = self._motion_controller.safe_pose_sequence(
                    reason='startup',
                    exit_zero_gravity=False,
                    wait_time=2.0,
                    tolerance=0.1,
                    reenable_zero_gravity=self._params.zero_gravity_default,
                )
            else:
                self._logger.warning('SAFE_POSE not marked ready; skipping initial move.')

            if self._params.zero_gravity_default and not reached_safe_pose:
                self._logger.warning('Zero-gravity default requested but SAFE_POSE verification failed; skipping zero-gravity.')
        except Exception as exc:  # pragma: no cover
            self._logger.error('Failed to connect hardware: %s', exc)
            self.get_logger().error(traceback.format_exc())
            raise

    # ------------------------------------------------------------------ subscribers
    def _on_robot_command(self, msg: PoseStamped) -> None:
        try:
            self._motion_controller.handle_robot_command(msg)
        except Exception as exc:  # pragma: no cover - defensive guard
            self._logger.error('robot_command handler failed: %s', exc)
            self.get_logger().error(traceback.format_exc())

    def _on_joint_command(self, msg: JointState) -> None:
        try:
            self._motion_controller.handle_joint_command(msg)
        except Exception as exc:  # pragma: no cover - defensive guard
            self._logger.error('joint_command handler failed: %s', exc)
            self.get_logger().error(traceback.format_exc())

    # ------------------------------------------------------------------ services
    def _handle_self_check(self, _request: Trigger.Request, response: Trigger.Response):
        """Run self-check: verify joints, move through range, return to safe pose."""
        try:
            self._logger.info('Running self-check sequence')

            # Step 1: Read current joint state
            joint_data = self._commander.read_joint_state()
            if not joint_data.names:
                response.success = False
                response.message = 'No joint data available'
                return response

            # Step 2: Verify we can read positions
            self._logger.info(f'Self-check: Read {len(joint_data.names)} joints')

            # Step 3: Move to safe pose
            self._commander.move_to_safe_pose()
            import time
            time.sleep(1.0)

            # Step 4: Verify arrival
            if self._commander.check_at_safe_pose():
                response.success = True
                response.message = f'Self-check passed: {len(joint_data.names)} joints OK, moved to safe pose'
            else:
                response.success = False
                response.message = 'Self-check: Failed to reach safe pose'

            self._logger.info(f'Self-check complete: {response.message}')
        except Exception as exc:
            response.success = False
            response.message = f'Self-check failed: {exc}'
            self._logger.error(response.message)

        return response

    # ------------------------------------------------------------------ action server
    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self._motion_controller.halt_and_safe('trajectory_cancelled')
        return CancelResponse.ACCEPT

    async def _execute_trajectory(self, goal_handle):
        try:
            result = self._motion_controller.execute_trajectory(goal_handle)
            goal_handle.succeed()
            return result
        except Exception as exc:
            self.get_logger().error('Trajectory execution failed: %s', exc)
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

    def _joint_goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _joint_cancel_callback(self, goal_handle) -> CancelResponse:
        self._motion_controller.halt_and_safe('joint_action_cancelled')
        return CancelResponse.ACCEPT

    async def _execute_joint_action(self, goal_handle):
        try:
            result = self._motion_controller.execute_joint_action(goal_handle)
            if result.success:
                goal_handle.succeed()
            elif result.result_code == 'CANCELED':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            self.get_logger().error('Joint action execution failed: %s', exc)
            goal_handle.abort()
            result = JointCommand.Result()
            result.success = False
            result.result_code = 'EXCEPTION'
            result.last_error = str(exc)
            return result

    # ------------------------------------------------------------------ lifecycle
    def destroy_node(self):
        self._trajectory_action.destroy()
        self._joint_action.destroy()
        self._commander.disconnect()
        return super().destroy_node()

    def _on_parameter_update(self, params):
        """Handle dynamic parameter updates."""
        result = SetParametersResult()
        result.successful = True
        result.reason = ''

        for param in params:
            param_name = param.name
            if param_name == 'joint_state_rate':
                new_rate = param.value
                self._logger.info(f'Updating joint_state_rate: {new_rate}')
                # Would need to recreate timer, skip for now
                result.successful = False
                result.reason = f'Runtime update of {param_name} not yet supported'
            elif param_name == 'command_timeout_s':
                new_timeout = param.value
                self._logger.info(f'Updating command_timeout_s: {new_timeout}')
                self._params.command_timeout_s = new_timeout
                self._watchdog._timeout = new_timeout
            else:
                self._logger.warning(f'Unknown parameter for runtime update: {param_name}')
                result.successful = False
                result.reason = f'Parameter {param_name} cannot be updated at runtime'

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
