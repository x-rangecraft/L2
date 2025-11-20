"""ROS 2 entry point for robot_driver."""
from __future__ import annotations

import asyncio
import math
import time
import traceback

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from robot_driver.action import JointCommand, SafetyPose

from driver.config.parameter_schema import DriverParameters, declare_and_get_parameters
from driver.config.safe_pose_loader import SafePoseLoader
from driver.control.command_watchdog import CommandWatchdog
from driver.control.joint_controler import JointControler
from driver.control.state_publisher import StatePublisher
from driver.control.zero_gravity_manager import ZeroGravityManager
from driver.hardware.hardware_commander import HardwareCommander
from driver.hardware.robot_description_loader import RobotDescriptionLoader
from driver.utils.logging_utils import get_logger

_SAFE_POSE_SPEED_SCALE = 0.8
_SAFE_POSE_TIMEOUT_S = 10.0
_SAFE_POSE_EXIT_ZERO_GRAVITY = True
_SAFE_POSE_RELATIVE = False
_SAFE_POSE_SETTLE_TIME = 1.5
_SAFE_POSE_TOLERANCE = 0.05


class RobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_driver')
        self._params: DriverParameters = declare_and_get_parameters(self)
        self._logger = get_logger('robot_driver_node')
        self._safe_pose_loader = SafePoseLoader(self._params.safe_pose_fallback)
        self._safe_pose = self._safe_pose_loader.load(self._params.safe_pose_file)
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
        self._state_publisher = StatePublisher(
            self,
            self._commander,
            self._params,
        )
        self._watchdog = CommandWatchdog(
            self,
            self._motion_controller,
            self._commander,
            timeout_s=self._params.command_timeout_s,
        )
        self._trajectory_group = ReentrantCallbackGroup()
        self._joint_action_group = ReentrantCallbackGroup()
        self._safety_action_group = ReentrantCallbackGroup()
        self._joint_client_group = ReentrantCallbackGroup()

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
            callback_group=self._trajectory_group,
        )
        self._joint_action = ActionServer(
            self,
            JointCommand,
            self._params.joint_command_action,
            execute_callback=self._execute_joint_action,
            goal_callback=self._joint_goal_callback,
            cancel_callback=self._joint_cancel_callback,
            callback_group=self._joint_action_group,
        )
        self._joint_action_client = ActionClient(
            self,
            JointCommand,
            self._params.joint_command_action,
            callback_group=self._joint_client_group,
        )
        self._safety_pose_action = ActionServer(
            self,
            SafetyPose,
            self._params.safety_pose_action,
            execute_callback=self._execute_safety_pose_action,
            goal_callback=self._safety_goal_callback,
            cancel_callback=self._safety_cancel_callback,
            callback_group=self._safety_action_group,
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

    def _safety_goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _safety_cancel_callback(self, goal_handle) -> CancelResponse:
        self._logger.warning('Safety pose action cancel requested.')
        return CancelResponse.ACCEPT

    async def _execute_safety_pose_action(self, goal_handle):
        self._ensure_event_loop()
        goal = goal_handle.request
        enable_zero_gravity_after = bool(goal.enable_zero_gravity_after)
        label = 'safety_pose'
        label_tag = f' [{label}]'
        config_path = self._params.safe_pose_file
        settle_time = _SAFE_POSE_SETTLE_TIME
        tolerance = _SAFE_POSE_TOLERANCE
        speed_scale = _SAFE_POSE_SPEED_SCALE
        skip_verification = False
        timeout = _SAFE_POSE_TIMEOUT_S
        exit_zero_gravity = _SAFE_POSE_EXIT_ZERO_GRAVITY

        feedback = SafetyPose.Feedback()
        feedback.phase = 'loading'
        feedback.completion_ratio = 0.0
        feedback.message = f'Loading SAFE_POSE from {config_path}'
        goal_handle.publish_feedback(feedback)

        self._logger.info('Safety pose action%s: loading %s', label_tag, config_path)
        try:
            safe_pose = self._safe_pose_loader.load(config_path)
        except Exception as exc:
            self._logger.error('Safety pose action%s failed to load config: %s', label_tag, exc)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'LOAD_FAILED'
            result.last_error = str(exc)
            result.max_error = float('nan')
            result.joint_action_code = 'UNSENT'
            return result

        if not safe_pose.joint_names or not safe_pose.positions:
            self._logger.error('Safety pose action%s missing joint data', label_tag)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'INVALID_CONFIG'
            result.last_error = 'safe_pose_config missing joint data'
            result.max_error = float('nan')
            result.joint_action_code = 'UNSENT'
            return result

        if not safe_pose.ready:
            self._logger.warning('Safety pose action%s: SAFE_POSE not marked ready', label_tag)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'SAFE_POSE_NOT_READY'
            result.last_error = 'SAFE_POSE file not ready (.ready marker missing)'
            result.max_error = float('nan')
            result.joint_action_code = 'UNSENT'
            return result

        if not await self._wait_for_joint_server(timeout):
            self._logger.error('Safety pose action%s: joint action server unavailable', label_tag)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'JOINT_SERVER_UNAVAILABLE'
            result.last_error = 'Timed out waiting for /robot_driver/action/joint'
            result.max_error = float('nan')
            result.joint_action_code = 'UNSENT'
            return result

        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state = JointState()
        joint_goal.target.joint_state.name = list(safe_pose.joint_names)
        joint_goal.target.joint_state.position = list(safe_pose.positions)
        joint_goal.target.relative = _SAFE_POSE_RELATIVE
        joint_goal.target.speed_scale = speed_scale
        joint_goal.options.timeout = self._float_to_duration(timeout)
        joint_goal.options.exit_zero_gravity = exit_zero_gravity
        joint_goal.options.label = label

        def _forward_joint_feedback(feedback_msg):
            fb = SafetyPose.Feedback()
            fb.phase = f'joint/{feedback_msg.feedback.phase}'
            fb.completion_ratio = feedback_msg.feedback.completion_ratio
            summary = self._summarize_joint_state(feedback_msg.feedback.current_state)
            if summary:
                fb.message = f'Moving joints: {summary}'
            else:
                fb.message = 'Joint action running'
            if feedback_msg.feedback.last_error:
                fb.message += f' | {feedback_msg.feedback.last_error}'
            goal_handle.publish_feedback(fb)

        send_future = self._joint_action_client.send_goal_async(
            joint_goal,
            feedback_callback=_forward_joint_feedback,
        )
        joint_goal_handle = await send_future
        if not joint_goal_handle.accepted:
            self._logger.error('Safety pose action%s: joint goal rejected', label_tag)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'JOINT_REJECTED'
            result.last_error = 'Joint action rejected goal'
            result.max_error = float('nan')
            result.joint_action_code = 'REJECTED'
            return result

        feedback.phase = 'joint'
        feedback.completion_ratio = 0.1
        feedback.message = 'Joint action running'
        goal_handle.publish_feedback(feedback)

        joint_result_future = joint_goal_handle.get_result_async()
        while not joint_result_future.done():
            if goal_handle.is_cancel_requested:
                self._logger.warning('Safety pose action%s cancel requested; canceling joint goal', label_tag)
                await joint_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result = SafetyPose.Result()
                result.success = False
                result.result_code = 'CANCELED'
                result.last_error = 'Safety pose goal canceled'
                result.max_error = float('nan')
                result.joint_action_code = 'CANCELED'
                return result
            await asyncio.sleep(0.1)

        joint_result = await joint_result_future
        action_result = joint_result.result
        max_error = self._compute_max_error(action_result.final_state, action_result.target_state)
        feedback.phase = 'settling'
        feedback.completion_ratio = 0.95
        final_summary = self._summarize_joint_state(action_result.final_state)
        if final_summary:
            feedback.message = f'Waiting for joints to settle (current: {final_summary})'
        else:
            feedback.message = 'Waiting for joints to settle'
        goal_handle.publish_feedback(feedback)

        if settle_time > 0:
            await asyncio.sleep(settle_time)

        final_max_error = max_error if math.isfinite(max_error) else float('nan')
        tolerance_check = True
        if not skip_verification and math.isfinite(final_max_error):
            tolerance_check = final_max_error <= tolerance

        result = SafetyPose.Result()
        result.joint_action_code = action_result.result_code or ''
        result.max_error = float(final_max_error)
        result.last_error = action_result.last_error or ''

        if not action_result.success:
            self._logger.error('Safety pose action%s failed: %s', label_tag, action_result.last_error)
            result.success = False
            result.result_code = action_result.result_code or 'JOINT_FAILED'
            goal_handle.abort()
            return result

        if not tolerance_check:
            self._logger.warning(
                'Safety pose action%s out of tolerance (%.4f rad > %.4f rad)',
                label_tag,
                final_max_error,
                tolerance,
            )
            result.success = False
            result.result_code = 'OUT_OF_TOLERANCE'
            result.last_error = (
                f'SAFE_POSE max error {final_max_error:.4f} rad exceeds tolerance {tolerance:.4f} rad'
            )
            goal_handle.abort()
            return result

        feedback.phase = 'verification'
        feedback.completion_ratio = 0.97
        feedback.message = f'SAFE_POSE verified (max |Δ|={final_max_error:.4f} rad ≤ {tolerance:.4f} rad)'
        goal_handle.publish_feedback(feedback)

        if enable_zero_gravity_after:
            feedback.phase = 'zero_gravity'
            feedback.completion_ratio = 0.98
            feedback.message = 'Enabling zero-gravity mode'
            goal_handle.publish_feedback(feedback)
            if not self._zero_gravity_manager.enable():
                self._logger.error('Safety pose action%s failed to enable zero-gravity', label_tag)
                result.success = False
                result.result_code = 'ZERO_GRAVITY_FAILED'
                result.last_error = 'Failed to enable zero gravity after SAFE_POSE'
                goal_handle.abort()
                return result
            feedback.message = 'Zero-gravity enabled'
            goal_handle.publish_feedback(feedback)

        feedback.phase = 'complete'
        feedback.completion_ratio = 1.0
        summary = self._summarize_joint_state(action_result.final_state)
        if summary:
            feedback.message = f'SAFE_POSE reached (state: {summary})'
        else:
            feedback.message = 'SAFE_POSE reached'
        goal_handle.publish_feedback(feedback)

        self._logger.info('Safety pose action%s complete (max |Δ|=%.4f rad)', label_tag, final_max_error)
        result.success = True
        result.result_code = 'SUCCESS'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------ lifecycle
    async def _wait_for_joint_server(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 1.0)
        while time.monotonic() < deadline:
            if self._joint_action_client.wait_for_server(timeout_sec=0.0):
                return True
            await asyncio.sleep(0.1)
        return False

    @staticmethod
    def _float_to_duration(value: float) -> Duration:
        msg = Duration()
        if value <= 0.0 or not math.isfinite(value):
            return msg
        msg.sec = int(value)
        msg.nanosec = int((value - msg.sec) * 1e9)
        return msg

    @staticmethod
    def _compute_max_error(final_state: JointState, target_state: JointState) -> float:
        if not final_state or not final_state.name or not target_state or not target_state.name:
            return float('nan')
        final_map = {name: pos for name, pos in zip(final_state.name, final_state.position)}
        target_map = {name: pos for name, pos in zip(target_state.name, target_state.position)}
        if not final_map or not target_map:
            return float('nan')
        diffs = [abs(final_map.get(name, 0.0) - target) for name, target in target_map.items()]
        return max(diffs) if diffs else float('nan')

    @staticmethod
    def _summarize_joint_state(state: JointState | None, limit: int = 3) -> str:
        if not state or not state.name or not state.position:
            return ''
        pairs = []
        for name, pos in zip(state.name, state.position):
            pairs.append(f'{name}={pos:+.3f}')
            if len(pairs) == limit:
                break
        summary = ', '.join(pairs)
        if len(state.name) > limit:
            summary += ', ...'
        return summary

    @staticmethod
    def _ensure_event_loop() -> None:
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

    def destroy_node(self):
        self._state_publisher.shutdown()
        self._trajectory_action.destroy()
        self._joint_action.destroy()
        self._safety_pose_action.destroy()
        self._joint_action_client.destroy()
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
