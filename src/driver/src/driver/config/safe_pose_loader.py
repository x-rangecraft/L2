"""Safe pose loading and validation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import yaml

from driver.config.parameter_schema import SafePoseFallback
from driver.utils.logging_utils import get_logger
from driver.utils.path_utils import resolve_relative_path


@dataclass
class SafePose:
    joint_names: List[str]
    positions: List[float]
    frame_id: str = 'base_link'
    ready: bool = False
    source: str = ''


class SafePoseLoader:
    def __init__(self, fallback: Optional[SafePoseFallback] = None):
        self._fallback = fallback or SafePoseFallback()
        self._logger = get_logger()

    def load(self, path_str: str) -> SafePose:
        if not path_str:
            self._logger.warning('safe_pose_file not provided, using fallback configuration')
            return self._fallback_pose()

        # 强制把相对路径限定到 driver/config，避免把 SAFE_POSE 散落在工作区根目录
        normalized = Path(path_str)
        if not normalized.is_absolute():
            if not normalized.parts or normalized.parts[0] != 'config':
                normalized = Path('config') / normalized
            path_str = str(normalized)

        try:
            path = resolve_relative_path(path_str, must_exist=True)
        except FileNotFoundError:
            self._logger.warning('Safe pose file %s missing, using fallback', path_str)
            return self._fallback_pose()

        data = yaml.safe_load(path.read_text()) or {}
        if 'safe_pose' in data:
            data = data['safe_pose']

        joint_names, positions = self._extract_joint_data(data)
        metadata = data.get('metadata', {})
        frame_id = metadata.get('frame_id', 'base_link')
        ready_flag = bool(metadata.get('ready', False))
        ready_marker = Path(f'{path}.ready')
        ready = ready_flag or ready_marker.exists()

        if not joint_names or not positions:
            self._logger.warning('Safe pose file %s lacks joint data, using fallback', path)
            return self._fallback_pose()

        if len(joint_names) != len(positions):
            raise ValueError(f'Safe pose file {path} has mismatched joint_names/positions lengths')

        if not ready:
            self._logger.warning(
                'SAFE_POSE file %s 未标记为可用（缺少 .ready 标记或 metadata.ready=false）；相关操作将报错',
                path,
            )

        self._logger.info('Loaded SAFE_POSE from %s (%d joints, ready=%s)', path, len(joint_names), ready)
        return SafePose(joint_names=joint_names, positions=positions, frame_id=frame_id, ready=ready, source=str(path))

    def _fallback_pose(self) -> SafePose:
        if not self._fallback.joint_names or not self._fallback.positions:
            raise ValueError('Fallback SAFE_POSE data missing; update config safe_pose_fallback')
        if not self._fallback.ready:
            self._logger.warning('SAFE_POSE fallback 未标记为可用，将阻止 move_to_safe_pose')
        return SafePose(
            joint_names=self._fallback.joint_names,
            positions=self._fallback.positions,
            ready=self._fallback.ready,
            source='parameter:safe_pose_fallback',
        )

    def _extract_joint_data(self, data) -> tuple[list[str], list[float]]:
        """Extract joint data supporting legacy (names+positions) and new map formats."""
        joint_names = list(data.get('joint_names', []))
        positions = [float(x) for x in data.get('positions', [])]
        if joint_names and positions:
            return joint_names, positions

        joints_section = data.get('joints')
        if isinstance(joints_section, dict):
            ordered_names = list(joints_section.keys())
            ordered_positions = [float(joints_section[name]) for name in ordered_names]
            return ordered_names, ordered_positions

        entries = data.get('joint_entries')
        if isinstance(entries, list):
            names: list[str] = []
            values: list[float] = []
            for entry in entries:
                name = str(entry.get('name', '')).strip()
                if not name:
                    continue
                names.append(name)
                values.append(float(entry.get('position', 0.0)))
            if names and values:
                return names, values

        return joint_names, positions
