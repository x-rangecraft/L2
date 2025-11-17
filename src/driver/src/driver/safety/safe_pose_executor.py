"""Shared SAFE_POSE execution helper.

Provides a single function that encapsulates the full sequence of
"exit zero‑gravity → 停止/落位 → 等待 → 校验 → 可选恢复零重力" 并可选地
向调用方回调执行结果，便于在 driver 内外一致调用。
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Protocol


class _Logger(Protocol):  # minimal logger Protocol to avoid tight coupling
    def info(self, msg: str, *args, **kwargs) -> None: ...
    def warning(self, msg: str, *args, **kwargs) -> None: ...


class _Commander(Protocol):
    def get_zero_gravity(self) -> bool: ...
    def set_zero_gravity(self, enabled: bool) -> bool: ...
    def check_at_safe_pose(self, tolerance: float = 0.05) -> bool: ...
    def safe_pose_max_error(self) -> float: ...


def run_safe_pose_sequence(
    *,
    commander: _Commander,
    halt_and_safe: Callable[[str], bool],
    logger: _Logger,
    reason: str = '',
    exit_zero_gravity: bool = False,
    wait_time: float = 2.0,
    tolerance: float = 0.1,
    reenable_zero_gravity: bool = False,
    on_complete: Optional[Callable[[bool, float], None]] = None,
) -> bool:
    """Run SAFE_POSE sequence and optionally invoke a callback with result.

    Returns True only if SAFE_POSE 校验通过且（若要求）零重力已恢复。
    The callback, if provided, receives (success, max_error_rad).
    """

    if exit_zero_gravity and commander.get_zero_gravity():
        logger.info('Exiting zero-gravity before safe pose movement')
        commander.set_zero_gravity(False)
        time.sleep(0.2)

    moved = halt_and_safe(reason)
    if not moved:
        if on_complete:
            on_complete(False, float('inf'))
        return False

    if wait_time > 0:
        logger.info('Waiting %.1fs for SAFE_POSE arrival', wait_time)
        time.sleep(wait_time)

    in_pose = commander.check_at_safe_pose(tolerance=tolerance)
    max_err = commander.safe_pose_max_error()
    if not in_pose:
        logger.warning(
            'Robot may not be at SAFE_POSE; max |Δ|=%.3f rad (tolerance %.3f).',
            max_err,
            tolerance,
        )
        if on_complete:
            on_complete(False, max_err)
        return False

    if reenable_zero_gravity:
        logger.info('Confirmed arrival at SAFE_POSE, entering zero-gravity mode')
        commander.set_zero_gravity(True)
        logger.info('Safety sequence complete, zero_gravity enabled')

    if on_complete:
        on_complete(True, max_err)
    return True


__all__ = ['run_safe_pose_sequence']
