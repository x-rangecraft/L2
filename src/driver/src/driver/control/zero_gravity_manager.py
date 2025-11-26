"""Utility class for managing zero-gravity state and ROS service exposure."""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node
from std_srvs.srv import SetBool

from driver.hardware.hardware_commander import HardwareCommander
from driver.utils.logging_utils import get_logger


class ZeroGravityManager:
    """Encapsulates zero-gravity toggling and state tracking.

    This class owns the `/robot_driver/service/zero_gravity` ROS service and
    proxies requests to :class:`HardwareCommander`, while caching the current
    state to make it easy for higher level modules to query.
    """

    def __init__(
        self,
        node: Node,
        commander: HardwareCommander,
        *,
        service_name: str,
        is_ready_callback: Optional[Callable[[], tuple[bool, str]]] = None,
    ) -> None:
        self._commander = commander
        self._logger = get_logger('zero_gravity_manager')
        self._current_state = commander.get_zero_gravity()
        self._is_ready_callback = is_ready_callback
        self._service = node.create_service(SetBool, service_name, self._handle_service_request)

    @property
    def current_state(self) -> bool:
        """Return the last known zero-gravity state."""
        return self._current_state

    def set_state(self, enabled: bool) -> bool:
        """Toggle zero-gravity mode via the HardwareCommander."""
        try:
            success = self._commander.set_zero_gravity(enabled)
        except Exception as exc:  # pragma: no cover - defensive, commander handles errors
            self._logger.error('Exception toggling zero_gravity: %s', exc)
            return False

        if success:
            self._current_state = enabled
            self._logger.info('zero_gravity -> %s', enabled)
        else:
            self._logger.error('Hardware commander rejected zero_gravity -> %s', enabled)
        return success

    def enable(self) -> bool:
        """Convenience helper to enable zero-gravity."""
        return self.set_state(True)

    def disable(self) -> bool:
        """Convenience helper to disable zero-gravity."""
        return self.set_state(False)

    def refresh_state(self) -> bool:
        """Sync cached state from HardwareCommander."""
        self._current_state = self._commander.get_zero_gravity()
        return self._current_state

    # ------------------------------------------------------------------ ROS callbacks
    def _handle_service_request(self, request: SetBool.Request, response: SetBool.Response):
        # 检查是否可以接受请求
        if self._is_ready_callback is not None:
            ready, reason = self._is_ready_callback()
            if not ready:
                self._logger.warning('zero_gravity 服务请求被拒绝: %s', reason)
                response.success = False
                response.message = reason
                return response

        success = self.set_state(request.data)
        response.success = success
        if success:
            response.message = f'zero_gravity -> {request.data}'
        else:
            response.message = 'Failed to toggle zero_gravity (see logs)'
        return response

    # These attributes are primarily for testing/introspection.
    @property
    def service(self):
        """Return the ROS service handle (primarily for tests)."""
        return self._service
