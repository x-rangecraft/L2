"""Safe pose loading and validation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

import yaml

from .logging_utils import get_logger
from .path_utils import resolve_relative_path
from .parameter_schema import SafePoseFallback


@dataclass
class SafePose:
    joint_names: List[str]
    positions: List[float]
    frame_id: str = 'base_link'


class SafePoseLoader:
    def __init__(self, fallback: Optional[SafePoseFallback] = None):
        self._fallback = fallback or SafePoseFallback()
        self._logger = get_logger()

    def load(self, path_str: str) -> SafePose:
        if not path_str:
            self._logger.warning('safe_pose_file not provided, using fallback configuration')
            return self._fallback_pose()

        path = resolve_relative_path(path_str, must_exist=False)
        if not path.exists():
            self._logger.warning('Safe pose file %s missing, using fallback', path)
            return self._fallback_pose()

        data = yaml.safe_load(path.read_text()) or {}
        if 'safe_pose' in data:
            data = data['safe_pose']

        joint_names = list(data.get('joint_names', []))
        positions = [float(x) for x in data.get('positions', [])]
        frame_id = data.get('metadata', {}).get('frame_id', 'base_link')

        if not joint_names or not positions:
            self._logger.warning('Safe pose file %s lacks joint data, using fallback', path)
            return self._fallback_pose()

        if len(joint_names) != len(positions):
            raise ValueError(f'Safe pose file {path} has mismatched joint_names/positions lengths')

        self._logger.info('Loaded SAFE_POSE from %s (%d joints)', path, len(joint_names))
        return SafePose(joint_names=joint_names, positions=positions, frame_id=frame_id)

    def _fallback_pose(self) -> SafePose:
        if not self._fallback.joint_names or not self._fallback.positions:
            raise ValueError('Fallback SAFE_POSE data missing; update config safe_pose_fallback')
        return SafePose(
            joint_names=self._fallback.joint_names,
            positions=self._fallback.positions,
        )
