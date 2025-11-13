"""Helper utilities for validating gs_usb CAN adapters."""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from .logging_utils import get_logger
from .path_utils import resolve_relative_path

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
    commands = [
        ['ip', 'link', 'set', interface, 'down'],
        ['ip', 'link', 'set', interface, 'type', 'can', 'bitrate', str(bitrate)],
        ['ip', 'link', 'set', interface, 'up'],
    ]
    for cmd in commands:
        LOGGER.info('Executing: %s', ' '.join(cmd))
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            LOGGER.error('Command %s failed: %s', cmd, result.stderr.strip())
            raise RuntimeError(f'Failed to configure {interface}')


def _check_interface(interface: str) -> bool:
    result = subprocess.run(['ip', '-details', 'link', 'show', interface], capture_output=True, text=True)
    if result.returncode != 0:
        return False
    output = result.stdout.lower()
    return 'state up' in output or 'can' in output


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
