"""Unit tests for HardwareCommander and SAFE_POSE helpers."""
import unittest

from driver.hardware.hardware_commander import HardwareCommander, JointStateData, CartesianTarget
from driver.hardware.robot_description_loader import JointLimit, RobotDescription
from driver.config.safe_pose_loader import SafePose, SafePoseFallback, SafePoseLoader
from driver.safety.safe_pose_manager import SafePoseManager


class TestHardwareCommander(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures."""
        joint_limits = {
            'joint1': JointLimit('joint1', min_position=-1.0, max_position=1.0),
            'joint2': JointLimit('joint2', min_position=-0.5, max_position=1.5),
            'joint3': JointLimit('joint3', min_position=0.0, max_position=2.0),
        }
        self.robot_description = RobotDescription(joint_limits=joint_limits)
        self.commander = HardwareCommander(
            joint_limit_rate=10.0,
            robot_description=self.robot_description,
        )

    def test_initialization(self):
        """Test commander initializes correctly."""
        self.assertIsNotNone(self.commander)
        self.assertFalse(self.commander.get_zero_gravity())

    def test_zero_gravity_toggle(self):
        """Test zero gravity mode can be toggled."""
        self.assertFalse(self.commander.get_zero_gravity())
        self.commander.set_zero_gravity(True)
        self.assertTrue(self.commander.get_zero_gravity())
        self.commander.set_zero_gravity(False)
        self.assertFalse(self.commander.get_zero_gravity())

    def test_read_joint_state(self):
        """Test reading joint state returns valid data."""
        state = self.commander.read_joint_state()
        self.assertIsInstance(state, JointStateData)
        # RobotDescription has three joints
        self.assertEqual(len(state.names), 3)
        self.assertEqual(len(state.positions), 3)

    def test_joint_limits_clamp_targets(self):
        """Commanded joint positions should respect description limits."""
        self.commander.command_joint_positions(['joint1'], [5.0])
        state = self.commander.read_joint_state()
        self.assertLessEqual(state.positions[0], 1.0)

        self.commander.command_joint_positions(['joint1'], [-5.0])
        state = self.commander.read_joint_state()
        self.assertGreaterEqual(state.positions[0], -1.0)


class TestSafePoseManager(unittest.TestCase):
    def setUp(self):
        fallback = SafePoseFallback(
            joint_names=['joint1', 'joint2', 'joint3'],
            positions=[0.0, 0.5, 1.0],
            ready=True,
        )
        self.safe_pose_loader = SafePoseLoader(fallback=fallback)
        # Use the fallback directly without reading a file
        safe_pose = SafePose(
            joint_names=fallback.joint_names,
            positions=fallback.positions,
            ready=fallback.ready,
            source='test_fallback',
        )
        joint_limits = {
            'joint1': JointLimit('joint1', min_position=-1.0, max_position=1.0),
            'joint2': JointLimit('joint2', min_position=-0.5, max_position=1.5),
            'joint3': JointLimit('joint3', min_position=0.0, max_position=2.0),
        }
        robot_description = RobotDescription(joint_limits=joint_limits)
        commander = HardwareCommander(joint_limit_rate=10.0, robot_description=robot_description)
        # Seed the commander state with the fallback SAFE_POSE
        commander.set_joint_state(
            JointStateData(
                names=safe_pose.joint_names,
                positions=safe_pose.positions,
                velocities=[0.0] * len(safe_pose.joint_names),
                efforts=[0.0] * len(safe_pose.joint_names),
            )
        )
        self.manager = SafePoseManager(commander, self.safe_pose_loader, safe_pose_file='')
        # Manually inject the SafePose so that we do not depend on filesystem paths.
        self.manager._safe_pose = safe_pose  # type: ignore[attr-defined]

    def test_safe_pose_available(self):
        self.assertTrue(self.manager.safe_pose_available())

    def test_check_at_safe_pose(self):
        status = self.manager.check_at_safe_pose(tolerance=0.1)
        self.assertTrue(status.in_pose)
        self.assertLessEqual(status.max_error, 0.1)

    def test_move_to_safe_pose_best_effort(self):
        # Move commander away from SAFE_POSE then command move back.
        self.manager._commander.command_joint_positions(  # type: ignore[attr-defined]
            ['joint1', 'joint2', 'joint3'],
            [0.3, 0.7, 1.3],
        )
        status_before = self.manager.check_at_safe_pose(tolerance=0.01)
        self.assertFalse(status_before.in_pose)
        self.assertTrue(self.manager.move_to_safe_pose())
        status_after = self.manager.check_at_safe_pose(tolerance=0.2)
        self.assertTrue(status_after.in_pose)


if __name__ == '__main__':
    unittest.main()
