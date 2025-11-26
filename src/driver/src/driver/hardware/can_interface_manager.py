"""Helper utilities for validating gs_usb CAN adapters."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from driver.utils.logging_utils import get_logger
from driver.utils.path_utils import resolve_relative_path

LOGGER = get_logger(__name__)


def ensure_ready(interface: str, *, reset_script: Optional[str] = None, bitrate: int = 1_000_000) -> None:
    """Ensure the CAN ``interface`` exists and is in the UP state."""
    if not interface:
        raise ValueError('interface is required')

    if _check_interface(interface):
        LOGGER.info('CAN interface %s already up', interface)
        return

    LOGGER.warning('CAN interface %s not up, attempting recovery', interface)

    if reset_script:
        _run_reset_script(reset_script)
        time.sleep(0.5)

    _bring_up_interface(interface, bitrate)

    if not _check_interface(interface):
        raise RuntimeError(f'Failed to bring {interface} up even after recovery attempts')

    LOGGER.info('CAN interface %s ready', interface)


def _run_reset_script(path_str: str) -> None:
    path = resolve_relative_path(path_str, must_exist=True)
    LOGGER.info('Running CAN reset script: %s', path)
    result = subprocess.run([str(path)], capture_output=True, text=True)
    if result.returncode != 0:
        LOGGER.error('Reset script failed (%s): %s', result.returncode, result.stderr.strip())
        raise RuntimeError('reset script failed')


def _bring_up_interface(interface: str, bitrate: int) -> None:
    """将 CAN 接口设置为 UP 状态。需要 root 权限或 sudo。"""
    # 如果不是 root 用户，使用 sudo
    use_sudo = os.geteuid() != 0
    sudo_prefix = ['sudo'] if use_sudo else []

    commands = [
        sudo_prefix + ['ip', 'link', 'set', interface, 'down'],
        sudo_prefix + ['ip', 'link', 'set', interface, 'type', 'can', 'bitrate', str(bitrate)],
        sudo_prefix + ['ip', 'link', 'set', interface, 'up'],
    ]
    for cmd in commands:
        LOGGER.info('Executing: %s', ' '.join(cmd))
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            LOGGER.error('Command %s failed: %s', cmd, result.stderr.strip())
            raise RuntimeError(f'Failed to configure {interface}')


def _check_interface(interface: str) -> bool:
    """检查 CAN 接口是否处于 UP 状态。"""
    result = subprocess.run(['ip', '-details', 'link', 'show', interface], capture_output=True, text=True)
    if result.returncode != 0:
        LOGGER.warning('Failed to query interface %s: %s', interface, result.stderr.strip())
        return False
    output = result.stdout.lower()
    # 必须同时满足：是 CAN 接口 且 状态为 UP
    is_can = 'link/can' in output or 'type can' in output
    is_up = 'state up' in output
    if not is_can:
        LOGGER.warning('Interface %s is not a CAN interface', interface)
        return False
    if not is_up:
        LOGGER.warning('CAN interface %s exists but state is not UP', interface)
        return False
    return True


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='gs_usb CAN interface manager')
    parser.add_argument('--ensure', action='store_true', help='Ensure the interface is ready')
    parser.add_argument('--interface', required=True, help='Interface name, e.g. can0')
    parser.add_argument('--reset-script', help='Optional reset script path')
    parser.add_argument('--bitrate', type=int, default=1_000_000)
    return parser.parse_args(argv)


def _main(argv: list[str]) -> int:
    args = _parse_args(argv)
    if args.ensure:
        ensure_ready(args.interface, reset_script=args.reset_script, bitrate=args.bitrate)
    else:
        LOGGER.info('No action requested')
    return 0


if __name__ == '__main__':
    raise SystemExit(_main(sys.argv[1:]))
