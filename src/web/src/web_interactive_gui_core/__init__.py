"""web_interactive_gui_core - Web 交互界面核心模块"""

from .camera_manager import CameraManager
from .perception_client import PerceptionClient, RecordResult, SegmentResult
from .web_gui import State, WebInteractiveGui, main
from .web_server import WebServerManager

__all__ = [
    "CameraManager",
    "PerceptionClient",
    "SegmentResult",
    "RecordResult",
    "WebServerManager",
    "WebInteractiveGui",
    "State",
    "main",
]
