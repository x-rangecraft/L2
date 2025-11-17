"""Unit tests for SafePoseLoader."""
import tempfile
import unittest
from pathlib import Path

from driver.config.parameter_schema import SafePoseFallback
from driver.config.safe_pose_loader import SafePoseLoader, SafePose


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
        self.assertFalse(pose.ready)

    def test_load_from_valid_file(self):
        """Test loading safe pose from a valid YAML file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("""
safe_pose:
  joint_names:
    - joint1
    - joint2
    - joint3
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

            self.assertEqual(pose.joint_names, ['joint1', 'joint2', 'joint3'])
            self.assertEqual(pose.positions, [0.0, -0.3, 0.6])
            self.assertEqual(pose.frame_id, 'base_link')
            self.assertFalse(pose.ready)
        finally:
            Path(temp_path).unlink()

    def test_ready_marker_file(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("""
joint_names: [j1, j2]
positions: [0.0, 1.0]
metadata:
  frame_id: base_link
  ready: false
""")
            temp_path = Path(f.name)

        try:
            Path(str(temp_path) + '.ready').touch()
            loader = SafePoseLoader()
            pose = loader.load(str(temp_path))
            self.assertTrue(pose.ready)
        finally:
            Path(str(temp_path)).unlink(missing_ok=True)
            Path(str(temp_path) + '.ready').unlink(missing_ok=True)

    def test_load_without_config_prefix(self):
        config_dir = Path(__file__).resolve().parents[1] / 'config'
        config_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=config_dir, delete=False) as f:
            f.write("""
joint_names:
  - j1
  - j2
positions:
  - 0.1
  - 0.2
metadata:
  ready: true
""")
            filename = Path(f.name).name

        try:
            loader = SafePoseLoader()
            pose = loader.load(filename)
            self.assertEqual(pose.joint_names, ['j1', 'j2'])
            self.assertEqual(pose.positions, [0.1, 0.2])
            self.assertTrue(pose.ready)
        finally:
            Path(config_dir / filename).unlink(missing_ok=True)
            Path(config_dir / f"{filename}.ready").unlink(missing_ok=True)


if __name__ == '__main__':
    unittest.main()
