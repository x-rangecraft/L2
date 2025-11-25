from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml


@dataclass
class SkillStep:
    id: str
    type: str
    desc: str
    params: Dict[str, Any]


@dataclass
class SkillDefinition:
    skill_id: str
    description: str
    steps: List[SkillStep]


class SkillLibrary:
    """Loads general actions and per-skill sequences from YAML files."""

    def __init__(self, skill_sets_dir: str | Path) -> None:
        self._base_path = Path(skill_sets_dir)
        self._common_actions: Dict[str, SkillStep] = {}
        self.reload_common_actions()

    def reload_common_actions(self) -> None:
        general_path = self._base_path / 'general_skill.yaml'
        if not general_path.exists():
            raise FileNotFoundError(f"general_skill.yaml not found under {self._base_path}")
        data = yaml.safe_load(general_path.read_text()) or {}
        actions: Dict[str, SkillStep] = {}
        for action_id, spec in (data.get('common_actions') or {}).items():
            action_type = spec.get('type')
            if not action_type:
                raise ValueError(f"common action '{action_id}' missing 'type'")
            desc = spec.get('desc', action_id)
            params = deepcopy(spec.get('params', {}))
            actions[action_id] = SkillStep(
                id=action_id,
                type=action_type,
                desc=desc,
                params=params,
            )
        if not actions:
            raise ValueError('common_actions is empty in general_skill.yaml')
        self._common_actions = actions

    def has_skill(self, skill_id: str) -> bool:
        return (self._base_path / f'{skill_id}.yaml').is_file()

    def list_skills(self) -> List[str]:
        return [
            path.stem
            for path in self._base_path.glob('*.yaml')
            if path.name != 'general_skill.yaml'
        ]

    def load_skill(self, skill_id: str) -> SkillDefinition:
        skill_path = self._base_path / f'{skill_id}.yaml'
        if not skill_path.exists():
            raise FileNotFoundError(
                f"Skill file '{skill_id}.yaml' not found under {self._base_path}"
            )
        data = yaml.safe_load(skill_path.read_text()) or {}
        description = data.get('description', '')
        steps_data = data.get('steps') or []
        if not steps_data:
            raise ValueError(f"Skill '{skill_id}' has no steps defined")
        steps: List[SkillStep] = []
        for index, item in enumerate(steps_data):
            steps.append(self._resolve_step(item, index))
        return SkillDefinition(skill_id=skill_id, description=description, steps=steps)

    def _resolve_step(self, item: Dict[str, Any], index: int) -> SkillStep:
        action_ref = item.get('action') or item.get('action_id')
        if action_ref:
            if action_ref not in self._common_actions:
                raise KeyError(f"Unknown action reference '{action_ref}' in step {index}")
            base = deepcopy(self._common_actions[action_ref])
            step_id = item.get('id', base.id)
            desc = item.get('desc', base.desc)
            params_override = item.get('params') or item.get('params_override')
            if params_override:
                params = self._merge_params(base.params, params_override)
            else:
                params = deepcopy(base.params)
            return SkillStep(id=step_id, type=base.type, desc=desc, params=params)

        step_type = item.get('type')
        if not step_type:
            raise ValueError(f"step {index} missing 'type' or 'action' field")
        step_id = item.get('id', f'{step_type}_{index}')
        desc = item.get('desc', step_id)
        params = deepcopy(item.get('params', {}))
        return SkillStep(id=step_id, type=step_type, desc=desc, params=params)

    def _merge_params(self, base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        result = deepcopy(base)
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(result.get(key), dict):
                result[key] = self._merge_params(result[key], value)
            else:
                result[key] = value
        return result


__all__ = ['SkillLibrary', 'SkillDefinition', 'SkillStep']
