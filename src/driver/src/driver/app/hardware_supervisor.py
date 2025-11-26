"""Hardware watchdog + CAN maintenance helpers for robot_driver."""
from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from driver.config.parameter_schema import DriverParameters
from driver.hardware.hardware_commander import HardwareCommander
from driver.utils.logging_utils import get_logger
from driver.utils.path_utils import resolve_relative_path


_DEFAULT_BACKOFF_S = 5.0


@dataclass
class _MaintenanceConfig:
    cleanup_script: Optional[Path]
    reset_script: Optional[Path]
    force_flag: bool
    cleanup_on_recover: bool


class _CanMaintenance:
    """Wrapper around the legacy shell cleanup/reset helpers."""

    def __init__(self, config: _MaintenanceConfig) -> None:
        self._cfg = config
        self._logger = get_logger('can_maintenance')

    def prepare(self, can_channel: str, *, first_attempt: bool) -> None:
        if not can_channel:
            raise ValueError('can_channel is required for CAN maintenance')

        if self._cfg.cleanup_script and (first_attempt or self._cfg.cleanup_on_recover):
            self._run_cleanup(can_channel)

        if self._cfg.reset_script:
            self._run_script(self._cfg.reset_script, label='reset_all_can')

    # ------------------------------------------------------------------ helpers
    def _run_cleanup(self, can_channel: str) -> None:
        script = self._cfg.cleanup_script
        if not script:
            self._logger.info('cleanup_can helper skipped：未配置脚本路径')
            return
        if not script.exists():
            self._logger.warning('cleanup_can helper 未找到：%s', script)
            return
        args = [str(script), can_channel]
        if self._cfg.force_flag:
            args.append('--force')
        self._run_script(args, label='cleanup_can')

    def _run_script(self, command, *, label: str) -> None:
        if isinstance(command, (str, Path)):
            cmd = [str(command)]
        else:
            cmd = [str(part) for part in command]
        if not Path(cmd[0]).exists():
            self._logger.warning('%s helper 路径不存在：%s，跳过执行', label, cmd[0])
            return
        self._logger.info('Running %s helper: %s', label, ' '.join(cmd))
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self._logger.error('%s helper failed (%s): %s', label, result.returncode, result.stderr.strip())
            raise RuntimeError(f'{label} helper failed (exit {result.returncode})')
        if result.stdout:
            self._logger.debug('%s output: %s', label, result.stdout.strip())


class HardwareSupervisor:
    """Manage CAN maintenance and hardware connect/disconnect.

    Note: 安全位姿复位由 RobotDriverNode 通过 SafetyPose action 异步执行，
    本类只负责硬件连接层面的管理。
    """

    def __init__(
        self,
        *,
        commander: HardwareCommander,
        params: DriverParameters,
        on_recovery_needed: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._commander = commander
        self._params = params
        self._logger = get_logger('hardware_supervisor')
        self._stop_event = threading.Event()
        self._connected = False
        self._first_attempt = True
        self._thread_hook_installed = False
        self._orig_thread_hook = None
        self._on_recovery_needed = on_recovery_needed
        cleanup_script = self._resolve_optional_path(params.can_cleanup_script)
        reset_script = self._resolve_optional_path(params.can_reset_script)
        maintenance_cfg = _MaintenanceConfig(
            cleanup_script=cleanup_script,
            reset_script=reset_script,
            force_flag=bool(params.can_cleanup_force),
            cleanup_on_recover=bool(params.can_cleanup_on_recover),
        )
        self._maintenance = _CanMaintenance(maintenance_cfg)

    @property
    def connected(self) -> bool:
        """是否已连接硬件。"""
        return self._connected

    # ------------------------------------------------------------------ lifecycle
    def connect(self) -> None:
        """执行 CAN 准备和硬件连接。

        Raises:
            RuntimeError: 如果连接失败
        """
        self._install_thread_hook()
        self._logger.info('[robot_start] CAN 检查并清理中 (CAN=%s)', self._params.can_channel)
        self._maintenance.prepare(self._params.can_channel, first_attempt=self._first_attempt)
        self._commander.connect(
            self._params.can_channel,
            reset_script=self._resolve_optional_path(self._params.can_reset_script),
        )
        self._first_attempt = False
        self._connected = True
        self._logger.info('[robot_start] 机械臂连接成功')

    def disconnect(self) -> None:
        """断开硬件连接。"""
        self._stop_event.set()
        self._remove_thread_hook()
        try:
            self._commander.disconnect()
        except Exception as exc:
            self._logger.warning('Error disconnecting commander: %s', exc)
        self._connected = False
        self._logger.info('Hardware disconnected')

    def connect_with_retry(self, max_attempts: int = 0, backoff: float = 0.0) -> bool:
        """带重试的连接。

        Args:
            max_attempts: 最大重试次数，0 表示无限重试
            backoff: 重试间隔秒数，0 使用默认值

        Returns:
            是否连接成功
        """
        if backoff <= 0:
            backoff = max(float(self._params.hardware_reconnect_backoff_s or _DEFAULT_BACKOFF_S), 0.5)

        attempt = 0
        while not self._stop_event.is_set():
            attempt += 1
            try:
                self.connect()
                return True
            except Exception as exc:
                self._logger.error('Hardware connect attempt %d failed: %s', attempt, exc)
                if max_attempts and attempt >= max_attempts:
                    self._logger.error('Reached hardware connect attempt limit (%d); giving up', max_attempts)
                    return False
                time.sleep(backoff)
        return False

    # ------------------------------------------------------------------ recovery handling
    def _install_thread_hook(self) -> None:
        if getattr(threading, 'excepthook', None) is None or self._thread_hook_installed:
            return
        self._orig_thread_hook = threading.excepthook
        supervisor = self

        def _hook(args):  # type: ignore[override]
            supervisor._on_thread_exception(args)
            if supervisor._orig_thread_hook:
                supervisor._orig_thread_hook(args)

        threading.excepthook = _hook
        self._thread_hook_installed = True
        self._logger.debug('Installed threading.excepthook for hardware watchdog')

    def _remove_thread_hook(self) -> None:
        if self._thread_hook_installed and getattr(threading, 'excepthook', None) is not None:
            threading.excepthook = self._orig_thread_hook  # type: ignore[assignment]
        self._thread_hook_installed = False

    def _on_thread_exception(self, args) -> None:
        if self._stop_event.is_set():
            return
        message = str(args.exc_value).lower()
        thread_name = getattr(args.thread, 'name', '<unknown>')
        self._logger.error('Background thread %s crashed: %s', thread_name, args.exc_value)
        if self._should_attempt_recovery(message):
            self._connected = False
            if self._on_recovery_needed:
                self._on_recovery_needed(message or 'thread_exception')

    def _should_attempt_recovery(self, message: str) -> bool:
        if not message:
            return False
        keywords = ('motorchainrobot', 'dmchain', 'loss communication', 'can0', 'i2rt')
        return any(key in message for key in keywords)

    # ------------------------------------------------------------------ misc
    @staticmethod
    def _resolve_optional_path(path_str: str) -> Optional[Path]:
        if not path_str:
            return None
        try:
            return resolve_relative_path(path_str, must_exist=True)
        except FileNotFoundError:
            get_logger('hardware_supervisor').warning('Optional path %s not found; skipping', path_str)
            return None


__all__ = ['HardwareSupervisor']
