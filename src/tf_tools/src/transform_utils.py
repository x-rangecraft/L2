#!/usr/bin/env python3
"""
TransformUtils - TF 转换工具类

提供点转换和位姿转换的纯函数实现。
"""
from typing import List

import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose


class TransformUtils:
    """TF 转换工具类 - 纯函数实现，无节点依赖"""

    @staticmethod
    def transform_points(
        points_in: List[Point],
        transform: TransformStamped
    ) -> List[Point]:
        """
        Apply transform to points array.
        
        Args:
            points_in: 输入点列表
            transform: TF 变换
            
        Returns:
            转换后的点列表
        """
        # 提取变换参数
        t = transform.transform.translation
        q = transform.transform.rotation

        # 构建变换矩阵
        T = TransformUtils.transform_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)

        # 将点列表转换为 (N, 3)
        points = np.array([[p.x, p.y, p.z] for p in points_in], dtype=float)

        # 添加齐次坐标
        ones = np.ones((points.shape[0], 1))
        points_homo = np.hstack([points, ones])  # (N, 4)

        # 应用变换
        transformed = (T @ points_homo.T).T[:, :3]  # (N, 3)

        # 转换回 Point 列表
        points_out: List[Point] = []
        for x, y, z in transformed:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            points_out.append(point)

        return points_out

    @staticmethod
    def transform_poses(
        poses_in: List[Pose],
        transform: TransformStamped
    ) -> List[Pose]:
        """
        Apply transform to poses array (position + orientation).
        
        Args:
            poses_in: 输入位姿列表
            transform: TF 变换
            
        Returns:
            转换后的位姿列表
        """
        poses_out: List[Pose] = []
        
        for pose_in in poses_in:
            # 使用 ROS2 标准库转换
            # do_transform_pose 接受 Pose 对象，返回 Pose 对象
            result_pose = do_transform_pose(pose_in, transform)
            poses_out.append(result_pose)
        
        return poses_out

    @staticmethod
    def transform_to_matrix(
        tx: float, ty: float, tz: float,
        qx: float, qy: float, qz: float, qw: float
    ) -> np.ndarray:
        """
        Convert translation + quaternion to 4x4 transformation matrix.
        
        Args:
            tx, ty, tz: 平移向量
            qx, qy, qz, qw: 四元数（旋转）
            
        Returns:
            4x4 齐次变换矩阵
        """
        # 四元数转旋转矩阵
        x, y, z, w = qx, qy, qz, qw
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

        # 构建 4x4 变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        return T

