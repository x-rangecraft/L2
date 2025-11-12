from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import (  # type: ignore
    PackageNotFoundError,
    get_package_share_directory,
)

try:
    import xacro  # type: ignore
except ImportError:  # pragma: no cover
    xacro = None  # type: ignore


class RobotDescNode(Node):
    """Publishes the URDF description string on /robot_description."""

    def __init__(self) -> None:
        super().__init__("robot_desc_node")
        self.declare_parameter("urdf_path", "urdf/yam.urdf")
        self.declare_parameter("xacro_args", [])
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("frame_prefix", "")

        self._description = ""
        self._share_dir = self._discover_share_dir()

        qos = QoSProfile(depth=1)
        self._publisher = self.create_publisher(String, "/robot_description", qos)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 0.1)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_timer_cb)
        self._reload_srv = self.create_service(Trigger, "reload_description", self._handle_reload)

        success, msg = self._load_description()
        if success:
            self.get_logger().info(msg)
            self._publish_description()
        else:
            self.get_logger().error(msg)

    def _discover_share_dir(self) -> Path:
        try:
            return Path(get_package_share_directory("description"))
        except (PackageNotFoundError, ValueError):
            return Path(__file__).resolve().parents[2]

    def _publish_timer_cb(self) -> None:
        if not self._description:
            success, msg = self._load_description()
            if not success:
                self.get_logger().throttle(5000, msg)
                return
        self._publish_description()

    def _publish_description(self) -> None:
        msg = String()
        msg.data = self._description
        self._publisher.publish(msg)

    def _handle_reload(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        success, message = self._load_description()
        resp.success = success
        resp.message = message
        if success:
            self._publish_description()
        return resp

    def _load_description(self) -> tuple[bool, str]:
        try:
            urdf_path = self._resolve_urdf_path()
            if urdf_path.suffix == ".xacro":
                description = self._render_xacro(urdf_path)
            else:
                description = urdf_path.read_text()
            prefix = str(self.get_parameter("frame_prefix").value)
            if prefix:
                description = description.replace("link_", f"{prefix}link_")
                description = description.replace("joint_", f"{prefix}joint_")
            self._description = description
            return True, f"Loaded robot description from {urdf_path}"
        except Exception as exc:  # pragma: no cover - defensive
            return False, f"Failed to load robot description: {exc}"

    def _resolve_urdf_path(self) -> Path:
        raw_path = str(self.get_parameter("urdf_path").value)
        if raw_path.startswith("package://"):
            pkg, rel = raw_path[len("package://"):].split("/", 1)
            base = Path(get_package_share_directory(pkg))
            return (base / rel).resolve()
        candidate = Path(raw_path)
        if not candidate.is_absolute():
            candidate = (self._share_dir / candidate).resolve()
        if not candidate.exists():
            raise FileNotFoundError(candidate)
        return candidate

    def _render_xacro(self, path: Path) -> str:
        if xacro is None:
            raise RuntimeError("xacro module not available")
        mappings = self._parse_xacro_args()
        doc = xacro.process_file(str(path), mappings=mappings)
        return doc.toxml()

    def _parse_xacro_args(self) -> Dict[str, str]:
        raw_args = self.get_parameter("xacro_args").value
        if isinstance(raw_args, str):
            args = [segment.strip() for segment in raw_args.split(",") if segment.strip()]
        elif isinstance(raw_args, (list, tuple, set)):
            args = [str(item) for item in raw_args if str(item).strip()]
        elif raw_args is None:
            args = []
        else:
            args = [str(raw_args).strip()]
        mappings: Dict[str, str] = {}
        for arg in args:
            if ":=" not in arg:
                continue
            key, value = arg.split(":=", 1)
            mappings[key] = value
        return mappings


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotDescNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down robot_desc_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
