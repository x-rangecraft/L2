"""SAFE_POSE configuration and state helper.

This module centralises all SAFE_POSE‑related configuration and state
inspection logic so that lower layers like ``HardwareCommander`` do not need
to know about SAFE_POSE semantics.

Responsibilities:
* load SAFE_POSE configuration (via :class:`SafePoseLoader`);
* cache the current SAFE_POSE description;
* compute the max joint error between the current state and SAFE_POSE;
* answer "are we at SAFE_POSE within tolerance?" style queries;
* provide a simple helper for moving *towards* SAFE_POSE using
  :class:`HardwareCommander` primitives.

It deliberately does *not* own any ROS entities (no subscriptions, actions
or services) so it can be reused from different layers (actions, watchdogs,
etc.) without introducing tight coupling.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from driver.config.safe_pose_loader import SafePose, SafePoseLoader
from driver.hardware.hardware_commander import HardwareCommander, JointStateData
from driver.utils.logging_utils import get_logger


@dataclass
class SafePoseStatus:
    """Result of a SAFE_POSE check."""

    in_pose: bool
    max_error: float


class SafePoseManager:
    """Manage SAFE_POSE configuration and comparison against current state."""

    def __init__(
        self,
        commander: HardwareCommander,
        loader: SafePoseLoader,
        safe_pose_file: str,
    ) -> None:
        self._logger = get_logger('safe_pose_manager')
        self._commander = commander
        self._loader = loader
        self._safe_pose_file = safe_pose_file
        self._safe_pose: SafePose = self._loader.load(safe_pose_file)

    # ------------------------------------------------------------------ config
    @property
    def safe_pose(self) -> SafePose:
        """Return the currently cached SAFE_POSE description."""
        return self._safe_pose

    def reload(self, path: Optional[str] = None) -> SafePose:
        """Reload SAFE_POSE from the given path (or the original file)."""
        if path is not None:
            self._safe_pose_file = path
        self._safe_pose = self._loader.load(self._safe_pose_file)
        return self._safe_pose

    def safe_pose_available(self) -> bool:
        """Return True if SAFE_POSE has data and is marked ready."""
        sp = self._safe_pose
        return bool(sp.ready) and bool(sp.joint_names) and bool(sp.positions)

    # ------------------------------------------------------------------ checks
    def safe_pose_max_error(
        self,
        state: Optional[JointStateData] = None,
    ) -> float:
        """Return max |error| in radians between current joints and SAFE_POSE.

        If SAFE_POSE is unavailable or the joint mapping is incomplete,
        returns ``math.inf`` so callers can log a meaningful failure reason.
        """
        if not self.safe_pose_available():
            return math.inf

        # Refresh state from hardware if caller did not supply one.
        if state is None:
            state = self._commander.read_joint_state()

        names = list(state.names)
        positions = list(state.positions)
        if not names or not positions:
            return math.inf

        index = {name: idx for idx, name in enumerate(names)}

        max_err = 0.0
        for i, name in enumerate(self._safe_pose.joint_names):
            idx = index.get(name)
            if idx is None or idx >= len(positions):
                return math.inf
            err = abs(positions[idx] - self._safe_pose.positions[i])
            if err > max_err:
                max_err = err
        return max_err

    def check_at_safe_pose(
        self,
        tolerance: float = 0.05,
        state: Optional[JointStateData] = None,
    ) -> SafePoseStatus:
        """Check whether the robot is at SAFE_POSE within the given tolerance.

        Returns a :class:`SafePoseStatus` with both the boolean result and the
        max joint error, so callers do not need to recompute it.
        """
        max_err = self.safe_pose_max_error(state=state)
        if math.isinf(max_err):
            return SafePoseStatus(in_pose=False, max_error=max_err)
        return SafePoseStatus(in_pose=max_err <= tolerance, max_error=max_err)

    # ------------------------------------------------------------------ commands
    def move_to_safe_pose(self) -> bool:
        """Best-effort move towards SAFE_POSE using joint commands.

        This helper intentionally uses only the public ``HardwareCommander``
        primitives instead of accessing its private SDK internals. The exact
        ramp behaviour is delegated to :meth:`command_joint_positions`.
        """
        if not self.safe_pose_available():
            self._logger.warning(
                'SAFE_POSE unavailable (ready flag missing or empty); '
                'skipping move_to_safe_pose.',
            )
            return False

        try:
            self._commander.command_joint_positions(
                self._safe_pose.joint_names,
                self._safe_pose.positions,
            )
            self._logger.info(
                'Commanded SAFE_POSE using %s',
                self._safe_pose.source or '<unknown>',
            )
            return True
        except Exception as exc:  # pragma: no cover - defensive
            self._logger.error('Failed to command SAFE_POSE: %s', exc)
            return False

    # ------------------------------------------------------------------ zero-gravity passthrough
    def get_zero_gravity(self) -> bool:
        """Expose zero-gravity state for helpers that expect a commander-like API."""
        return self._commander.get_zero_gravity()

    def set_zero_gravity(self, enabled: bool) -> bool:
        """Toggle zero-gravity via the underlying commander."""
        return self._commander.set_zero_gravity(enabled)


__all__ = ['SafePoseManager', 'SafePoseStatus']

