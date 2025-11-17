"""Safety watchdog for robot_driver."""
from __future__ import annotations

import time

from std_msgs.msg import Empty

from .hardware_commander import HardwareCommander
from .logging_utils import get_logger
from .motion_controller import MotionController


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
        """Execute safety sequence: exit zero-gravity if active, move to safe pose, wait, then re-enter zero-gravity."""
        now = time.monotonic()
        if now - self._last_trigger < 1.0:
            return
        self._last_trigger = now

        self._logger.warning('Safety engage triggered: %s', reason)

        # Step 1: If in zero-gravity, exit it first
        was_in_zero_g = self._commander.get_zero_gravity()
        if was_in_zero_g:
            self._logger.info('Exiting zero-gravity before safe pose movement')
            self._commander.set_zero_gravity(False)
            time.sleep(0.2)  # Brief pause

        # Step 2: Stop current motion and move to SAFE_POSE
        moved = self._motion_controller.halt_and_safe(reason)
        if not moved:
            self._logger.warning('SAFE_POSE move skipped; watchdog sequence degraded.')
            return

        # Step 3: Wait and verify arrival at SAFE_POSE
        self._logger.info('Waiting %.1fs for SAFE_POSE arrival', self._safe_pose_wait_time)
        time.sleep(self._safe_pose_wait_time)

        # Step 4: Verify position; only re-enter zero-gravity when SAFE_POSE is confirmed
        if self._commander.check_at_safe_pose(tolerance=0.1):
            self._logger.info('Confirmed arrival at SAFE_POSE, entering zero-gravity mode')
            self._commander.set_zero_gravity(True)
            self._logger.info('Safety sequence complete, zero_gravity enabled')
        else:
            self._logger.warning('Robot may not be at SAFE_POSE; zero-gravity re-enable skipped')
