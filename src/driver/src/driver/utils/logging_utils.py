"""Logging helpers for robot_driver."""
from __future__ import annotations

import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

from driver.utils.path_utils import resolve_relative_path

_LOGGER: Optional[logging.Logger] = None
_GRIPPER_LOGGER: Optional[logging.Logger] = None


def get_logger(name: str = 'robot_driver', log_dir: str = 'log/robot_driver') -> logging.Logger:
    """Return a module-level logger writing both stdout and a log file."""
    global _LOGGER
    if _LOGGER:
        return _LOGGER

    log_path = _resolve_log_path(name, log_dir)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s')

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    file_handler = logging.FileHandler(log_path)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.propagate = False
    logger.info('Logger initialized. Output file: %s', log_path)

    _LOGGER = logger
    return logger


def _resolve_log_path(node_name: str, log_dir: str) -> Path:
    env_path = os.environ.get('ROBOT_DRIVER_LOG_FILE')
    if env_path:
        return Path(env_path).expanduser().resolve()

    target_dir = resolve_relative_path(log_dir, must_exist=False)
    timestamp = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
    return target_dir / f'{node_name}_{timestamp}.log'


def emit_event(logger: logging.Logger, message: str, level: int = logging.INFO) -> None:
    """Shortcut for structured log lines."""
    logger.log(level, '%s', message)


def get_gripper_logger(
    name: str = 'gripper_action',
    log_dir: str = 'log',
    filename: str = 'gripper_action.log',
) -> logging.Logger:
    """Return a dedicated logger for gripper action debugging.

    This logger writes only to a file under ``log_dir`` (default ``L2/log``)
    and does not emit anything to stdout/stderr,以避免刷屏。每次创建时会先清理
    旧文件，方便做单次动作的离线分析。
    """
    global _GRIPPER_LOGGER
    if _GRIPPER_LOGGER:
        return _GRIPPER_LOGGER

    base_dir = resolve_relative_path(log_dir, must_exist=False)
    base_dir.mkdir(parents=True, exist_ok=True)
    log_path = base_dir / filename

    # 覆盖旧文件，避免无限增长。
    if log_path.exists():
        try:
            log_path.unlink()
        except OSError:
            # 若删除失败，继续以追加方式写入。
            pass

    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s')
    file_handler = logging.FileHandler(log_path)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.propagate = False
    logger.info('Gripper log initialized. Output file: %s', log_path)

    _GRIPPER_LOGGER = logger
    return logger
