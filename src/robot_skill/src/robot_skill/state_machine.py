from __future__ import annotations

from enum import Enum, auto
from typing import Optional


class SkillState(Enum):
    IDLE = auto()
    RUNNING = auto()
    SUCCESS = auto()
    FAILED = auto()


class SkillStateMachine:
    """Placeholder state machine for future skill orchestration."""

    def __init__(self, node) -> None:
        self._node = node
        self._state = SkillState.IDLE
        self._last_error: Optional[str] = None

    @property
    def state(self) -> SkillState:
        return self._state

    def reset(self) -> None:
        self._node.get_logger().debug('Resetting skill state machine to IDLE')
        self._state = SkillState.IDLE
        self._last_error = None

    def mark_running(self) -> None:
        self._node.get_logger().debug('Skill state -> RUNNING')
        self._state = SkillState.RUNNING

    def mark_success(self) -> None:
        self._node.get_logger().debug('Skill state -> SUCCESS')
        self._state = SkillState.SUCCESS

    def mark_failed(self, error: str) -> None:
        self._node.get_logger().error(f'Skill state -> FAILED: {error}')
        self._state = SkillState.FAILED
        self._last_error = error

    def last_error(self) -> Optional[str]:
        return self._last_error


__all__ = ['SkillStateMachine', 'SkillState']
