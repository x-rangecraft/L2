"""ROS 2 entry point for robot_driver."""
from __future__ import annotations

import math
import threading
import time
import traceback

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from robot_driver.action import JointCommand, RobotCommand, SafetyPose

from driver.config.parameter_schema import DriverParameters, declare_and_get_parameters
from driver.config.safe_pose_loader import SafePoseLoader
from driver.control.joint_controler import JointControler
from driver.control.state_publisher import StatePublisher
from driver.control.end_effector_pose_publisher import EndEffectorPosePublisher
from driver.control.zero_gravity_manager import ZeroGravityManager
from driver.hardware.hardware_commander import HardwareCommander
from driver.hardware.kinematics_solver import KinematicsSolver
from driver.hardware.robot_description_loader import RobotDescriptionLoader
from driver.safety.safe_pose_manager import SafePoseManager
from driver.utils.logging_utils import get_action_robot_logger, get_logger
from driver.app.gripper_action import GripperAction

_SAFE_POSE_SPEED_SCALE = 1.0
_SAFE_POSE_TIMEOUT_S = 10.0
_SAFE_POSE_EXIT_ZERO_GRAVITY = True
_SAFE_POSE_RELATIVE = False
_SAFE_POSE_SETTLE_TIME = 1.5
_SAFE_POSE_TOLERANCE = 0.05
_ROBOT_ACTION_DEFAULT_SPEED = 0.9
_ROBOT_ACTION_DEFAULT_TIMEOUT = 15.0


class RobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_driver')
        self._params: DriverParameters = declare_and_get_parameters(self)
        self._logger = get_logger('robot_driver_node')
        self._robot_description = RobotDescriptionLoader().load(self._params.robot_description_file)
        self._commander = HardwareCommander(
            joint_limit_rate=self._params.joint_command.rate_limit,
            robot_description=self._robot_description,
        )
        self._zero_gravity_manager = ZeroGravityManager(
            self,
            self._commander,
            service_name=self._params.zero_gravity_service,
        )
        # SAFE_POSE management is handled by a dedicated manager so that
        # HardwareCommander remains a thin hardware abstraction.
        self._safe_pose_manager = SafePoseManager(
            self._commander,
            SafePoseLoader(self._params.safe_pose_fallback),
            self._params.safe_pose_file,
        )
        self._motion_controller = JointControler(
            self,
            self._params,
            self._commander,
            zero_gravity_manager=self._zero_gravity_manager,
            zero_gravity_service=self._params.zero_gravity_service,
            safe_pose_manager=self._safe_pose_manager,
        )
        self._robot_action_logger = get_action_robot_logger(
            name='action_robot',
            log_dir='log/action_robot',
            filename='action_robot.log',
        )
        self._kinematics_solver = KinematicsSolver(
            self._commander,
            base_frame=self._params.tf_base_frame,
        )
        self._state_publisher = StatePublisher(
            self,
            self._commander,
            self._params,
        )
        self._ee_pose_publisher = EndEffectorPosePublisher(
            self,
            solver=self._kinematics_solver,
            joint_state_topic=self._params.joint_state_topic,
        )
        self._trajectory_group = ReentrantCallbackGroup()
        self._joint_action_group = ReentrantCallbackGroup()
        self._safety_action_group = ReentrantCallbackGroup()
        self._robot_action_group = ReentrantCallbackGroup()
        # Dedicated callback groups for action clients so topic wrappers can
        # safely forward commands to actions without blocking other callbacks.
        self._joint_client_group = ReentrantCallbackGroup()
        self._robot_client_group = ReentrantCallbackGroup()
        self._safety_client_group = ReentrantCallbackGroup()
        self._robot_goal_lock = threading.Lock()
        self._robot_joint_goals: dict[int, ActionClient.GoalHandle] = {}

        self._subscriptions = []
        self._create_ros_interfaces()
        self._connect_hardware()

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
                self._on_safety_stop,
                10,
            )
        )

        # Keep the self-check service name for API compatibility, but implement it
        # as a lightweight stub to avoid duplicating SAFE_POSE logic here.
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
        self._robot_action = ActionServer(
            self,
            RobotCommand,
            self._params.robot_command_action,
            execute_callback=self._execute_robot_action,
            goal_callback=self._robot_goal_callback,
            cancel_callback=self._robot_cancel_callback,
            callback_group=self._robot_action_group,
        )
        self._joint_action_client = ActionClient(
            self,
            JointCommand,
            self._params.joint_command_action,
            callback_group=self._joint_client_group,
        )
        self._robot_action_client = ActionClient(
            self,
            RobotCommand,
            self._params.robot_command_action,
            callback_group=self._robot_client_group,
        )
        self._safety_pose_client = ActionClient(
            self,
            SafetyPose,
            self._params.safety_pose_action,
            callback_group=self._safety_client_group,
        )
        # Dedicated gripper action wrapper; exposes /robot_driver/action/gripper
        # while internally reusing the JointCommand action.
        self._gripper_action = GripperAction(
            node=self,
            joint_client=self._joint_action_client,
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
            # 启动阶段不再自动回 SAFE_POSE；是否执行 SAFE_POSE 由外部脚本
            # （例如 start_robot_driver.sh 调用 /robot_driver/action/safety_pose）
            # 进行编排，便于开发阶段频繁重启和调试。
        except Exception as exc:  # pragma: no cover
            self._logger.error('Failed to connect hardware: %s', exc)
            self.get_logger().error(traceback.format_exc())
            raise

    # ------------------------------------------------------------------ subscribers
    def _on_robot_command(self, msg: PoseStamped) -> None:
        """Thin wrapper: forward /robot_driver/robot_command to RobotCommand action.

        This keeps the Topic API as a compatibility layer while ensuring all
        motion logic goes through /robot_driver/action/robot.
        """
        try:
            if not self._robot_action_client.wait_for_server(timeout_sec=0.0):
                self._logger.error(
                    'robot_command topic: RobotCommand action server %s unavailable',
                    self._params.robot_command_action,
                )
                return

            goal = RobotCommand.Goal()
            # PoseStamped carries both frame_id and pose; reuse it directly.
            goal.target = msg
            # Let action use its default speed/timeout handling; only tag source.
            goal.label = 'topic_robot_command'

            def _goal_response_cb(future) -> None:
                try:
                    goal_handle = future.result()
                except Exception as exc:  # pragma: no cover - defensive guard
                    self._logger.error('robot_command topic: failed to send RobotCommand goal: %s', exc)
                    return
                if goal_handle is None or not getattr(goal_handle, 'accepted', False):
                    self._logger.error('robot_command topic: RobotCommand goal rejected by action server')

            send_future = self._robot_action_client.send_goal_async(goal)
            send_future.add_done_callback(_goal_response_cb)
        except Exception as exc:  # pragma: no cover - defensive guard
            self._logger.error('robot_command handler failed: %s', exc)
            self.get_logger().error(traceback.format_exc())

    def _on_joint_command(self, msg: JointState) -> None:
        """Thin wrapper: forward /robot_driver/joint_command to JointCommand action.

        Topic interface stays always-on for tooling, but execution is unified
        through /robot_driver/action/joint.
        """
        try:
            if not self._joint_action_client.wait_for_server(timeout_sec=0.0):
                self._logger.error(
                    'joint_command topic: JointCommand action server %s unavailable',
                    self._params.joint_command_action,
                )
                return

            goal = JointCommand.Goal()
            goal.target.joint_state = msg
            # /robot_driver/joint_command historically uses absolute targets.
            goal.target.relative = False
            # Use full configured rate; callers needing slower motion should
            # switch to the action API and set speed_scale explicitly.
            goal.target.speed_scale = 1.0
            goal.options.label = 'topic_joint_command'

            def _goal_response_cb(future) -> None:
                try:
                    goal_handle = future.result()
                except Exception as exc:  # pragma: no cover - defensive guard
                    self._logger.error('joint_command topic: failed to send JointCommand goal: %s', exc)
                    return
                if goal_handle is None or not getattr(goal_handle, 'accepted', False):
                    self._logger.error('joint_command topic: JointCommand goal rejected by action server')

            send_future = self._joint_action_client.send_goal_async(goal)
            send_future.add_done_callback(_goal_response_cb)
        except Exception as exc:  # pragma: no cover - defensive guard
            self._logger.error('joint_command handler failed: %s', exc)
            self.get_logger().error(traceback.format_exc())

    def _on_safety_stop(self, _msg: Empty) -> None:
        """Handle /robot_driver/safety_stop by triggering SafetyPose action.

        This mirrors the SAFE_POSE sequence but routes it through the unified
        /robot_driver/action/safety_pose interface so that all SAFE_POSE logic
        lives in a single place.
        """
        try:
            self._logger.warning('Safety stop requested; sending SafetyPose action goal')
            if not self._safety_pose_client.wait_for_server(timeout_sec=0.0):
                self._logger.error(
                    'Safety stop: SafetyPose action server %s unavailable',
                    self._params.safety_pose_action,
                )
                return

            goal = SafetyPose.Goal()
            # After safety stop we expect zero-gravity to be enabled again.
            goal.enable_zero_gravity_after = True

            def _goal_response_cb(future) -> None:
                try:
                    goal_handle = future.result()
                except Exception as exc:  # pragma: no cover - defensive guard
                    self._logger.error('Safety stop: failed to send SafetyPose goal: %s', exc)
                    return
                if goal_handle is None or not getattr(goal_handle, 'accepted', False):
                    self._logger.error('Safety stop: SafetyPose goal rejected by action server')
                    return

                result_future = goal_handle.get_result_async()

                def _result_cb(result_future) -> None:
                    try:
                        action_result = result_future.result()
                    except Exception as exc:  # pragma: no cover - defensive guard
                        self._logger.error('Safety stop: exception waiting for SafetyPose result: %s', exc)
                        return
                    result_msg = getattr(action_result, 'result', None)
                    if result_msg is None:
                        self._logger.error('Safety stop: SafetyPose result message missing')
                        return
                    self._logger.warning(
                        'Safety stop completed: success=%s result_code=%s max_error=%.4f last_error=%s',
                        bool(getattr(result_msg, 'success', False)),
                        getattr(result_msg, 'result_code', '') or '',
                        float(getattr(result_msg, 'max_error', float('nan'))),
                        getattr(result_msg, 'last_error', '') or '',
                    )

                result_future.add_done_callback(_result_cb)

            send_future = self._safety_pose_client.send_goal_async(goal)
            send_future.add_done_callback(_goal_response_cb)
        except Exception as exc:  # pragma: no cover - defensive guard
            self._logger.error('Safety stop handler failed: %s', exc)
            self.get_logger().error(traceback.format_exc())

    # ------------------------------------------------------------------ services
    def _handle_self_check(self, _request: Trigger.Request, response: Trigger.Response):
        """Lightweight stub for /robot_driver/service/self_check.

        The previous self-check sequence duplicated SAFE_POSE handling; it is now
        removed to keep the node simpler. The service is left in place so that
        existing tools do not crash, but it always reports that self-check is
        disabled.
        """
        response.success = False
        response.message = 'Self-check is disabled; use external diagnostics instead.'
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

    def _robot_goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _robot_cancel_callback(self, goal_handle) -> CancelResponse:
        self._logger.warning('Robot action cancel requested.')
        with self._robot_goal_lock:
            joint_handle = self._robot_joint_goals.get(id(goal_handle))
        if joint_handle is not None:
            try:
                joint_handle.cancel_goal_async()
            except Exception as exc:
                self._logger.error('Failed to forward cancel to JointCommand: %s', exc)
        return CancelResponse.ACCEPT

    async def _execute_robot_action(self, goal_handle):
        goal = goal_handle.request
        label = goal.label or 'robot'
        label_tag = f' [{label}]' if label else ''
        pose_stamped = goal.target
        frame_id = pose_stamped.header.frame_id or self._params.tf_base_frame
        if frame_id and frame_id != self._params.tf_base_frame:
            msg = (
                f'Robot action target frame "{frame_id}" does not match base frame '
                f'"{self._params.tf_base_frame}"'
            )
            self._logger.error(msg + label_tag)
            return self._robot_action_abort(goal_handle, msg, 'UNSUPPORTED_FRAME')

        if not self._motion_controller.validate_robot_pose(pose_stamped):
            msg = 'Robot action goal failed validation'
            self._logger.error(msg + label_tag)
            return self._robot_action_abort(goal_handle, msg, 'INVALID_GOAL')

        speed_scale = self._sanitize_speed_scale(goal.speed_scale)
        timeout_msg, timeout_s = self._resolve_timeout(goal.timeout)
        start_time = time.monotonic()
        self._robot_action_logger.info(
            'Goal%s frame=%s pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f) '
            'speed_scale=%.2f timeout=%.2fs',
            label_tag,
            frame_id,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
            speed_scale,
            timeout_s,
        )

        ik_ok, joint_target, ik_error = self._kinematics_solver.solve_ik(pose_stamped.pose)
        if not ik_ok or joint_target is None:
            msg = ik_error or 'Failed to solve IK for target pose'
            self._logger.error('Robot action%s: %s', label_tag, msg)
            self._robot_action_logger.error('Goal%s failed IK: %s', label_tag, msg)
            return self._robot_action_abort(goal_handle, msg, 'IK_FAILED')

        feedback = RobotCommand.Feedback()
        feedback.phase = 'planning'
        feedback.completion_ratio = 0.0
        feedback.remaining_time = timeout_msg
        feedback.current_state = self._read_joint_state()
        feedback.target_state = joint_target
        feedback.contact_state = []
        pose_msg, _ = self._kinematics_solver.compute_fk_pose(
            stamp=self.get_clock().now().to_msg(),
            joint_state=self._commander.read_joint_state(),
        )
        feedback.current_pose = pose_msg
        feedback.last_error = ''
        goal_handle.publish_feedback(feedback)

        if not self._joint_action_client.wait_for_server(timeout_sec=2.0):
            msg = 'JointCommand action server unavailable for robot action'
            self._logger.error(msg + label_tag)
            self._robot_action_logger.error('Goal%s aborted: %s', label_tag, msg)
            return self._robot_action_abort(goal_handle, msg, 'JOINT_SERVER_UNAVAILABLE')

        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state = joint_target
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = timeout_msg
        joint_goal.options.label = label or 'robot'

        def _forward_joint_feedback(feedback_msg):
            jc_fb = feedback_msg.feedback
            fb = RobotCommand.Feedback()
            fb.phase = f'joint/{jc_fb.phase}'
            fb.completion_ratio = jc_fb.completion_ratio
            fb.remaining_time = jc_fb.remaining_time
            fb.current_state = jc_fb.current_state
            fb.target_state = jc_fb.target_state
            fb.contact_state = jc_fb.contact_state
            pose_from_fb, _ = self._kinematics_solver.compute_fk_pose(
                stamp=self.get_clock().now().to_msg(),
                joint_msg=jc_fb.current_state,
            )
            fb.current_pose = pose_from_fb
            fb.last_error = jc_fb.last_error
            goal_handle.publish_feedback(fb)

        send_future = self._joint_action_client.send_goal_async(
            joint_goal,
            feedback_callback=_forward_joint_feedback,
        )
        joint_goal_handle = await send_future
        if not joint_goal_handle.accepted:
            msg = 'JointCommand rejected robot action goal'
            self._logger.error(msg + label_tag)
            self._robot_action_logger.error('Goal%s rejected by JointCommand', label_tag)
            return self._robot_action_abort(goal_handle, msg, 'JOINT_REJECTED', target_state=joint_target)

        key = id(goal_handle)
        with self._robot_goal_lock:
            self._robot_joint_goals[key] = joint_goal_handle

        feedback.phase = 'waiting_joint'
        feedback.completion_ratio = 0.05
        feedback.remaining_time = timeout_msg
        goal_handle.publish_feedback(feedback)

        try:
            joint_result = await joint_goal_handle.get_result_async()
        finally:
            with self._robot_goal_lock:
                self._robot_joint_goals.pop(key, None)
        jc_result: JointCommand.Result = joint_result.result
        final_pose, _ = self._kinematics_solver.compute_fk_pose(
            stamp=self.get_clock().now().to_msg(),
            joint_msg=jc_result.final_state,
        )

        result = RobotCommand.Result()
        result.success = jc_result.success
        result.result_code = jc_result.result_code or ''
        result.last_error = jc_result.last_error or ''
        result.final_state = jc_result.final_state
        result.target_state = joint_target
        result.contact_state = list(jc_result.contact_state)
        result.final_pose = final_pose

        duration = time.monotonic() - start_time
        self._robot_action_logger.info(
            'Goal%s completed result=%s success=%s duration=%.2fs error=%s',
            label_tag,
            result.result_code,
            result.success,
            duration,
            result.last_error,
        )

        final_feedback = RobotCommand.Feedback()
        final_feedback.phase = 'complete'
        final_feedback.completion_ratio = 1.0 if jc_result.success else 0.0
        final_feedback.remaining_time = self._float_to_duration(0.0)
        final_feedback.current_state = jc_result.final_state
        final_feedback.target_state = joint_target
        final_feedback.contact_state = list(jc_result.contact_state)
        final_feedback.current_pose = final_pose
        final_feedback.last_error = jc_result.last_error or ''
        goal_handle.publish_feedback(final_feedback)

        if jc_result.success:
            goal_handle.succeed()
        elif jc_result.result_code == 'CANCELED':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _safety_goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _safety_cancel_callback(self, goal_handle) -> CancelResponse:
        self._logger.warning('Safety pose action cancel requested.')
        return CancelResponse.ACCEPT

    async def _execute_safety_pose_action(self, goal_handle):
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
            safe_pose = self._safe_pose_manager.reload(config_path)
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

        # 如果已经在安全位姿附近，则无需重复发送关节动作，只根据参数选择是否打开零重力。
        status = self._safe_pose_manager.check_at_safe_pose(tolerance=tolerance)
        if status.in_pose:
            self._logger.info(
                'Safety pose action%s: already at SAFE_POSE (max |Δ|=%.4f rad)',
                label_tag,
                status.max_error,
            )
            result = SafetyPose.Result()
            result.max_error = float(status.max_error)
            result.last_error = ''
            result.joint_action_code = 'NOOP'

            if enable_zero_gravity_after:
                if not self._zero_gravity_manager.enable():
                    self._logger.error('Safety pose action%s failed to enable zero-gravity', label_tag)
                    result.success = False
                    result.result_code = 'ZERO_GRAVITY_FAILED'
                    result.last_error = 'Failed to enable zero gravity after SAFE_POSE'
                    goal_handle.abort()
                    return result

            result.success = True
            result.result_code = 'SUCCESS'
            feedback.phase = 'complete'
            feedback.completion_ratio = 1.0
            feedback.message = 'SAFE_POSE already reached'
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return result

        if not self._joint_action_client.wait_for_server(timeout_sec=timeout):
            self._logger.error('Safety pose action%s: joint action server unavailable', label_tag)
            goal_handle.abort()
            result = SafetyPose.Result()
            result.success = False
            result.result_code = 'JOINT_SERVER_UNAVAILABLE'
            result.last_error = 'Timed out waiting for /robot_driver/action/joint'
            result.max_error = float('nan')
            result.joint_action_code = 'UNSENT'
            return result

        if exit_zero_gravity:
            self._logger.info('Safety pose action%s: disabling zero-gravity before joint motion', label_tag)
            if not self._zero_gravity_manager.disable():
                self._logger.warning('Safety pose action%s: failed to disable zero-gravity', label_tag)

        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state = JointState()
        joint_goal.target.joint_state.name = list(safe_pose.joint_names)
        joint_goal.target.joint_state.position = list(safe_pose.positions)
        joint_goal.target.relative = _SAFE_POSE_RELATIVE
        joint_goal.target.speed_scale = speed_scale
        joint_goal.options.timeout = self._float_to_duration(timeout)
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

        joint_result = await joint_goal_handle.get_result_async()
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
            time.sleep(settle_time)

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

    def _robot_action_abort(
        self,
        goal_handle,
        message: str,
        code: str,
        *,
        target_state: JointState | None = None,
    ) -> RobotCommand.Result:
        self._robot_action_logger.error('Robot action aborted: code=%s error=%s', code, message)
        result = RobotCommand.Result()
        result.success = False
        result.result_code = code
        result.last_error = message
        result.final_state = self._read_joint_state()
        result.target_state = target_state or JointState()
        result.contact_state = []
        pose_msg, _ = self._kinematics_solver.compute_fk_pose(
            stamp=self.get_clock().now().to_msg(),
            joint_state=self._commander.read_joint_state(),
        )
        result.final_pose = pose_msg
        goal_handle.abort()
        return result

    def _sanitize_speed_scale(self, value: float) -> float:
        if not math.isfinite(value) or value <= 0.0:
            return _ROBOT_ACTION_DEFAULT_SPEED
        return max(0.05, min(1.0, value))

    def _resolve_timeout(self, duration_msg: Duration) -> tuple[Duration, float]:
        total = self._duration_to_float(duration_msg)
        if total > 0.0:
            return duration_msg, total
        fallback = self._float_to_duration(_ROBOT_ACTION_DEFAULT_TIMEOUT)
        return fallback, _ROBOT_ACTION_DEFAULT_TIMEOUT

    def _read_joint_state(self) -> JointState:
        data = self._commander.read_joint_state()
        msg = JointState()
        msg.name = list(data.names)
        msg.position = list(data.positions)
        msg.velocity = list(data.velocities)
        msg.effort = list(data.efforts)
        return msg

    @staticmethod
    def _duration_to_float(duration: Duration) -> float:
        if duration is None:
            return 0.0
        return float(duration.sec) + float(duration.nanosec) / 1e9

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

    def destroy_node(self):
        self._state_publisher.shutdown()
        self._trajectory_action.destroy()
        self._joint_action.destroy()
        self._safety_pose_action.destroy()
        self._robot_action.destroy()
        self._gripper_action.destroy()
        self._joint_action_client.destroy()
        self._ee_pose_publisher.shutdown()
        self._commander.disconnect()
        return super().destroy_node()


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
