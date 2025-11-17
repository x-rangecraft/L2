"""Unit tests for parameter schema."""
import unittest
from driver.parameter_schema import SafePoseFallback, JointCommandConfig


class TestParameterSchema(unittest.TestCase):
    def test_safe_pose_fallback(self):
        """Test SafePoseFallback dataclass."""
        fallback = SafePoseFallback(
            joint_names=['j1', 'j2', 'j3'],
            positions=[0.0, 0.5, 1.0]
        )
        self.assertEqual(len(fallback.joint_names), 3)
        self.assertEqual(len(fallback.positions), 3)
        self.assertFalse(fallback.ready)

    def test_joint_command_config(self):
        """Test JointCommandConfig dataclass."""
        config = JointCommandConfig(
            enabled=True,
            topic='/test/topic',
            mode='position',
            rate_limit=0.5
        )
        self.assertTrue(config.enabled)
        self.assertEqual(config.mode, 'position')
        self.assertEqual(config.rate_limit, 0.5)


if __name__ == '__main__':
    unittest.main()
