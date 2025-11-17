"""Unit tests for HardwareCommander."""
import unittest
from driver.hardware_commander import HardwareCommander, JointStateData, CartesianTarget
from driver.robot_description_loader import JointLimit, RobotDescription
from driver.safe_pose_loader import SafePose


class TestHardwareCommander(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures."""
        self.safe_pose = SafePose(
            joint_names=['joint_1', 'joint_2', 'joint_3'],
            positions=[0.0, 0.5, 1.0],
            ready=True,
            source='test',
        )
        joint_limits = {
            'joint_1': JointLimit('joint_1', min_position=-1.0, max_position=1.0),
            'joint_2': JointLimit('joint_2', min_position=-0.5, max_position=1.5),
            'joint_3': JointLimit('joint_3', min_position=0.0, max_position=2.0),
        }
        self.robot_description = RobotDescription(joint_limits=joint_limits)
        self.commander = HardwareCommander(
            self.safe_pose,
            joint_limit_rate=10.0,
            robot_description=self.robot_description,
        )

    def test_initialization(self):
        """Test commander initializes correctly."""
        self.assertIsNotNone(self.commander)
        self.assertEqual(self.commander._safe_pose, self.safe_pose)
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
        self.assertEqual(len(state.names), 3)
        self.assertEqual(len(state.positions), 3)
        self.assertEqual(state.names, ['joint_1', 'joint_2', 'joint_3'])

    def test_check_at_safe_pose(self):
        """Test safe pose checking."""
        # Initially should be at safe pose
        self.assertTrue(self.commander.check_at_safe_pose(tolerance=0.1))

    def test_move_to_safe_pose(self):
        """Test moving to safe pose."""
        result = self.commander.move_to_safe_pose()
        self.assertTrue(result)

    def test_joint_limits_clamp_targets(self):
        """Commanded joint positions should respect description limits."""
        self.commander.command_joint_positions(['joint_1'], [5.0])
        state = self.commander.read_joint_state()
        self.assertLessEqual(state.positions[0], 1.0)

        self.commander.command_joint_positions(['joint_1'], [-5.0])
        state = self.commander.read_joint_state()
        self.assertGreaterEqual(state.positions[0], -1.0)


if __name__ == '__main__':
    unittest.main()
