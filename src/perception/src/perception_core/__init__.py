"""
Perception Core - 视觉分析节点核心模块

提供以下能力:
- 目标分割 (NanoSAM)
- 点云计算
- 向量化 (CLIP + DINOv3)
- 物体存储与检索 (FAISS)
- 抓取位姿生成 (Contact-GraspNet)
"""

from .constants import *
from .error_codes import PerceptionError, ErrorCode

__version__ = '0.1.0'
__all__ = [
    'PerceptionError',
    'ErrorCode',
]

