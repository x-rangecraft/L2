"""Unit tests for SafePoseLoader."""
import unittest
import tempfile
from pathlib import Path
from driver.safe_pose_loader import SafePoseLoader, SafePose
from driver.parameter_schema import SafePoseFallback


class TestSafePoseLoader(unittest.TestCase):
    def test_load_from_fallback(self):
        """Test loading safe pose from fallback when file doesn't exist."""
        fallback = SafePoseFallback(
            joint_names=['j1', 'j2'],
            positions=[0.1, 0.2]
        )
        loader = SafePoseLoader(fallback)
        pose = loader.load('nonexistent.yaml')

        self.assertEqual(pose.joint_names, ['j1', 'j2'])
        self.assertEqual(pose.positions, [0.1, 0.2])

    def test_load_from_valid_file(self):
        """Test loading safe pose from a valid YAML file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("""
safe_pose:
  joint_names:
    - joint_1
    - joint_2
    - joint_3
  positions:
    - 0.0
    - -0.3
    - 0.6
  metadata:
    frame_id: base_link
""")
            temp_path = f.name

        try:
            loader = SafePoseLoader()
            pose = loader.load(temp_path)

            self.assertEqual(pose.joint_names, ['joint_1', 'joint_2', 'joint_3'])
            self.assertEqual(pose.positions, [0.0, -0.3, 0.6])
            self.assertEqual(pose.frame_id, 'base_link')
        finally:
            Path(temp_path).unlink()


if __name__ == '__main__':
    unittest.main()
