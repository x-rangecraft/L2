from pathlib import Path

from robot_skill_core.skill_loader import SkillLibrary


def test_record_object_skill_steps():
    base_dir = Path(__file__).resolve().parents[1] / 'skill_sets'
    lib = SkillLibrary(base_dir)
    skill = lib.load_skill('record_object')

    assert skill.skill_id == 'record_object'
    assert len(skill.steps) == 10
    assert skill.steps[0].type == 'safety_pose'
    assert skill.steps[1].type == 'cartesian_move'
    assert skill.steps[1].params['target_pose']['position']['x'] == 0.44570661670496153
    assert skill.steps[2].type == 'gripper'
    assert skill.steps[6].type == 'joint_move'
    assert skill.steps[6].params['positions'] == [-2.14, 2.14, -2.14]
