"""Robot description loader for joint limit metadata."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, Optional, Tuple

import yaml

from driver.utils.logging_utils import get_logger
from driver.utils.path_utils import resolve_relative_path


@dataclass
class JointLimit:
    name: str
    min_position: Optional[float] = None
    max_position: Optional[float] = None
    max_velocity: Optional[float] = None
    max_effort: Optional[float] = None


class RobotDescription:
    """Structured robot description data parsed from YAML."""

    def __init__(self, *, name: str = '', joint_limits: Optional[Dict[str, JointLimit]] = None, metadata: Optional[dict] = None):
        self.name = name
        self._joint_limits = joint_limits or {}
        self.metadata = metadata or {}

    def joint_limit(self, joint_name: str) -> Optional[JointLimit]:
        return self._joint_limits.get(_normalize_joint_name(joint_name))

    @property
    def joint_limits(self) -> Dict[str, JointLimit]:
        return dict(self._joint_limits)

    def joint_names(self) -> Iterable[str]:
        return list(self._joint_limits.keys())

    def iter_joint_limits(self) -> Iterator[JointLimit]:
        return iter(self._joint_limits.values())

    def __bool__(self) -> bool:
        return bool(self._joint_limits)


class RobotDescriptionLoader:
    def __init__(self) -> None:
        self._logger = get_logger(__name__)

    def load(self, path_str: str) -> RobotDescription:
        if not path_str:
            self._logger.warning('robot_description_file 未配置，返回空描述。')
            return RobotDescription()

        # robot_description_file 是驱动的基础配置，这里采用 fail-fast 策略：
        #  - 使用 resolve_relative_path(must_exist=True) 在候选根目录中查找文件；
        #  - 找不到时直接抛出异常，而不是静默返回空描述，避免后续关节校验全部失败却难以定位。
        try:
            path = resolve_relative_path(path_str, must_exist=True)
        except FileNotFoundError:
            self._logger.error(
                'Robot description file %s not found; please check the robot_description_file parameter.',
                path_str,
            )
            raise

        try:
            data = yaml.safe_load(path.read_text()) or {}
        except Exception as exc:  # pragma: no cover - YAML errors
            self._logger.error('Failed to parse robot description %s: %s', path, exc)
            return RobotDescription()

        robot_meta = data.get('robot', {}) or {}
        joint_section = data.get('joints', {}) or {}
        limits = self._parse_joint_limits(joint_section)
        if limits:
            self._logger.info('Loaded robot_description (%d joints) from %s', len(limits), path)
        else:
            self._logger.warning('robot_description %s does not define any joints', path)

        return RobotDescription(name=robot_meta.get('name', ''), joint_limits=limits, metadata=robot_meta)

    def _parse_joint_limits(self, joint_section) -> Dict[str, JointLimit]:
        entries = []
        if isinstance(joint_section, dict):
            for key, value in joint_section.items():
                if isinstance(value, dict):
                    if 'name' not in value:
                        value = dict(value)
                        value['name'] = key
                    entries.append(value)
        elif isinstance(joint_section, list):
            entries = [entry for entry in joint_section if isinstance(entry, dict)]

        limits: Dict[str, JointLimit] = {}
        for entry in entries:
            raw_name = entry.get('name')
            if not raw_name:
                continue
            name = _normalize_joint_name(str(raw_name))
            min_pos, max_pos = _extract_range(entry.get('position_limits_rad'))
            velocity = _to_float(entry.get('velocity_limit_rad_per_s'))
            effort = _to_float(entry.get('effort_limit_nm'))
            limits[name] = JointLimit(name=name, min_position=min_pos, max_position=max_pos, max_velocity=velocity, max_effort=effort)
        return limits


def _normalize_joint_name(name: str) -> str:
    """Normalize joint labels to the `joint#` form used throughout the driver."""
    cleaned = name.strip().lower()
    if not cleaned:
        return cleaned

    if cleaned.startswith('joint'):
        suffix = cleaned[5:]
        if suffix.startswith('_'):
            suffix = suffix[1:]
        if suffix.isdigit():
            return f'joint{suffix}'
    return cleaned


def _extract_range(value) -> Tuple[Optional[float], Optional[float]]:
    if value is None:
        return None, None
    if isinstance(value, (list, tuple)):
        if len(value) >= 2:
            return _to_float(value[0]), _to_float(value[1])
        if len(value) == 1:
            return _to_float(value[0]), None
    if isinstance(value, dict):
        lower = value.get('min')
        if lower is None:
            lower = value.get('lower') or value.get('low')
        upper = value.get('max')
        if upper is None:
            upper = value.get('upper') or value.get('high')
        return _to_float(lower), _to_float(upper)
    return None, None


def _to_float(value) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):  # pragma: no cover - defensive
        return None
