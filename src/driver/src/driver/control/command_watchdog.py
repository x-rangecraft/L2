"""Safety watchdog for robot_driver."""
from __future__ import annotations

import threading
import time
from typing import Optional

from std_msgs.msg import Empty

from driver.control.motion_controller import MotionController
from driver.hardware.hardware_commander import HardwareCommander
from driver.utils.logging_utils import get_logger


class CommandWatchdog:
    def __init__(
        self,
        node,
        motion_controller: MotionController,
        commander: HardwareCommander,
        *,
        timeout_s: float,
    ):
        self._node = node
        self._motion_controller = motion_controller
        self._commander = commander
        self._timeout = timeout_s
        self._logger = get_logger(__name__)
        self._last_trigger = 0.0
        self._safe_pose_wait_time = 2.0  # Wait time to reach SAFE_POSE
        period = min(1.0, timeout_s / 3.0)
        self._timer = node.create_timer(period, self._tick)
        self._last_max_error = 0.0
        self._worker_lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None

    def _safe_pose_callback(self, success: bool, max_error: float) -> None:
        self._last_max_error = max_error
        if success:
            self._logger.info('SAFE_POSE sequence complete (max |Δ|=%.3f rad)', max_error)
        else:
            self._logger.warning('SAFE_POSE sequence failed; last max |Δ|=%.3f rad', max_error)

    # ------------------------------------------------------------------ timers
    def _tick(self):
        age = self._motion_controller.seconds_since_last_command()
        if age > self._timeout:
            self._logger.warning('Command watchdog timeout (%.1fs > %.1fs)', age, self._timeout)
            self.engage('command_timeout')

    # ------------------------------------------------------------------ public
    def handle_safety_stop(self, _msg: Empty | None = None) -> None:
        """Handle safety stop with the same SAFE_POSE+zero-gravity sequence used for timeouts."""
        self.engage('safety_stop')

    def engage(self, reason: str) -> None:
        """Execute safety sequence without blocking the rclpy executor."""
        now = time.monotonic()
        if now - self._last_trigger < 1.0:
            return
        self._last_trigger = now

        with self._worker_lock:
            if self._worker_thread and self._worker_thread.is_alive():
                self._logger.info('Safety sequence already running; ignoring trigger %s', reason)
                return
            self._worker_thread = threading.Thread(
                target=self._run_safety_sequence,
                args=(reason,),
                daemon=True,
            )
            self._worker_thread.start()

    # ------------------------------------------------------------------ helpers
    def _run_safety_sequence(self, reason: str) -> None:
        try:
            self._logger.warning('Safety engage triggered: %s', reason)
            ok = self._motion_controller.safe_pose_sequence(
                reason=reason,
                exit_zero_gravity=True,
                wait_time=self._safe_pose_wait_time,
                tolerance=0.1,
                reenable_zero_gravity=True,
                on_complete=self._safe_pose_callback,
            )
            if not ok:
                self._logger.warning('Safety sequence incomplete; zero_gravity may remain disabled')
        finally:
            with self._worker_lock:
                self._worker_thread = None
