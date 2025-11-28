#!/usr/bin/env python3
"""
Robot TF Node - 统一的 TF 管理节点

功能：
1. TF 发布：读取 URDF 和静态配置，发布静态和动态 TF
2. TF 转换：提供坐标转换 Service

替代原有的 robot_tf_publisher 节点，新增坐标转换服务。
"""
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener

from tf_tools.srv import TransformPoints


class URDFJoint:
    """Represents a joint from URDF with kinematic information."""

    def __init__(self, name: str, joint_type: str, parent: str, child: str):
        self.name = name
        self.joint_type = joint_type  # revolute, prismatic, fixed, etc.
        self.parent = parent
        self.child = child
        # Origin transform (parent -> child when joint is at zero position)
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]
        # Joint axis (for revolute/prismatic joints)
        self.axis = [0.0, 0.0, 1.0]

    def is_movable(self) -> bool:
        """Check if this joint can move."""
        return self.joint_type in ('revolute', 'prismatic', 'continuous')


class URDFParser:
    """Parse URDF file and extract joint information."""

    @staticmethod
    def parse_urdf_file(urdf_path: Path) -> List[URDFJoint]:
        """Parse URDF file and return list of joints."""
        from xml.etree import ElementTree as ET
        
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        tree = ET.parse(urdf_path)
        root = tree.getroot()

        joints = []
        for joint_elem in root.findall('joint'):
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type')

            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')
            if parent_elem is None or child_elem is None:
                continue

            parent_link = parent_elem.get('link')
            child_link = child_elem.get('link')

            joint = URDFJoint(joint_name, joint_type, parent_link, child_link)

            # Parse origin
            origin_elem = joint_elem.find('origin')
            if origin_elem is not None:
                xyz_str = origin_elem.get('xyz', '0 0 0')
                rpy_str = origin_elem.get('rpy', '0 0 0')
                joint.origin_xyz = [float(x) for x in xyz_str.split()]
                joint.origin_rpy = [float(x) for x in rpy_str.split()]

            # Parse axis
            axis_elem = joint_elem.find('axis')
            if axis_elem is not None:
                xyz_str = axis_elem.get('xyz', '0 0 1')
                joint.axis = [float(x) for x in xyz_str.split()]

            joints.append(joint)

        return joints


class RobotTF(Node):
    """
    ROS2 Node that provides unified TF management.
    
    功能：
    - 发布静态 TF：固定变换（world→base_link, world→camera_link 等）
    - 发布动态 TF：基于 joint_states 的机器人关节变换
    - 提供转换服务：/tf_tools/transform_points
    """

    def __init__(self):
        super().__init__('robot_tf')

        # Declare parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('static_tf_config', '')  # Path to static_tf_config.yaml

        # Get parameters
        urdf_path_str = self.get_parameter('urdf_path').value
        if not urdf_path_str:
            self.get_logger().error('Parameter urdf_path is required!')
            sys.exit(1)

        urdf_path = Path(urdf_path_str)
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        static_config_path = self.get_parameter('static_tf_config').value

        # Parse URDF
        self.get_logger().info(f'Loading URDF from: {urdf_path}')
        try:
            self.joints = URDFParser.parse_urdf_file(urdf_path)
            self.get_logger().info(f'Loaded {len(self.joints)} joints from URDF')
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF: {e}')
            sys.exit(1)

        # Build joint name to joint object mapping
        self.joint_map: Dict[str, URDFJoint] = {j.name: j for j in self.joints}
        self.movable_joints = [j for j in self.joints if j.is_movable()]
        self.fixed_joints = [j for j in self.joints if not j.is_movable()]

        self.get_logger().info(
            f'Joints: {len(self.movable_joints)} movable, {len(self.fixed_joints)} fixed'
        )

        # Current joint positions
        self.joint_positions: Dict[str, float] = {}

        # ================================================================
        # TF 发布功能（保留原有逻辑）
        # ================================================================
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Load and publish static TF transforms
        if static_config_path:
            self._load_and_publish_static_tf(Path(static_config_path))

        # Subscribe to joint_states with reliable QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )

        # Timer for publishing TF (fixed joints)
        timer_period = 1.0 / max(0.1, publish_rate)
        self.timer = self.create_timer(timer_period, self.publish_fixed_transforms)

        # ================================================================
        # TF 转换功能（新增）
        # ================================================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 创建转换服务
        self.transform_service = self.create_service(
            TransformPoints,
            '/tf_tools/transform_points',
            self._transform_points_callback
        )

        self.get_logger().info(
            f'robot_tf started: publishing at {publish_rate} Hz, '
            f'transform service available at /tf_tools/transform_points'
        )

    # ================================================================
    # 静态 TF 加载（保留原有逻辑）
    # ================================================================
    def _load_and_publish_static_tf(self, config_path: Path):
        """Load static TF configuration and publish them once."""
        if not config_path.exists():
            self.get_logger().warn(f'Static TF config not found: {config_path}')
            return

        try:
            with open(config_path, 'r') as f:
                config = json.load(f)

            transforms = []

            # Load transforms from config
            for name, tf_data in config.get('transforms', {}).items():
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = tf_data['parent_frame']
                transform.child_frame_id = tf_data['child_frame']

                # Translation
                tr = tf_data['translation_m']
                transform.transform.translation.x = tr['x']
                transform.transform.translation.y = tr['y']
                transform.transform.translation.z = tr['z']

                # Rotation (quaternion)
                quat = tf_data['quaternion']['forward_parent_to_child']
                transform.transform.rotation.x = quat['x']
                transform.transform.rotation.y = quat['y']
                transform.transform.rotation.z = quat['z']
                transform.transform.rotation.w = quat['w']

                transforms.append(transform)

            if transforms:
                self.static_tf_broadcaster.sendTransform(transforms)
                self.get_logger().info(f'Published {len(transforms)} static TF transforms')

        except Exception as e:
            self.get_logger().error(f'Failed to load static TF config: {e}')

    # ================================================================
    # 动态 TF 发布（保留原有逻辑）
    # ================================================================
    def joint_state_callback(self, msg: JointState):
        """Callback for joint_states topic."""
        # Update joint positions
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_map:
                self.joint_positions[name] = position

        # Publish dynamic transforms for movable joints
        self.publish_dynamic_transforms(msg.header.stamp)

    def publish_dynamic_transforms(self, stamp):
        """Publish TF transforms for movable joints based on current joint positions."""
        transforms = []

        for joint in self.movable_joints:
            position = self.joint_positions.get(joint.name, 0.0)
            transform = self.compute_transform(joint, position, stamp)
            transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def publish_fixed_transforms(self):
        """Publish TF transforms for fixed joints (published periodically)."""
        stamp = self.get_clock().now().to_msg()
        transforms = []

        for joint in self.fixed_joints:
            transform = self.compute_transform(joint, 0.0, stamp)
            transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def compute_transform(
        self,
        joint: URDFJoint,
        position: float,
        stamp
    ) -> TransformStamped:
        """
        Compute the transform for a joint given its current position.

        For revolute joints: rotate around axis by position (radians)
        For prismatic joints: translate along axis by position (meters)
        For fixed joints: use origin transform only
        """
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = joint.parent
        transform.child_frame_id = joint.child

        # Start with origin transform
        tx, ty, tz = joint.origin_xyz
        roll, pitch, yaw = joint.origin_rpy

        # For movable joints, apply joint position
        if joint.joint_type == 'revolute' or joint.joint_type == 'continuous':
            # Rotate around joint axis
            ax, ay, az = joint.axis

            # Convert origin RPY to quaternion
            qx, qy, qz, qw = self.rpy_to_quaternion(roll, pitch, yaw)

            # Create axis rotation quaternion
            half_angle = position / 2.0
            s = math.sin(half_angle)
            c = math.cos(half_angle)
            axis_qx = ax * s
            axis_qy = ay * s
            axis_qz = az * s
            axis_qw = c

            # Compose quaternions: q_result = q_origin * q_axis
            qx, qy, qz, qw = self.multiply_quaternions(
                qx, qy, qz, qw,
                axis_qx, axis_qy, axis_qz, axis_qw
            )

        elif joint.joint_type == 'prismatic':
            # Translate along joint axis
            ax, ay, az = joint.axis
            tx += ax * position
            ty += ay * position
            tz += az * position

            # Rotation is just the origin rotation
            qx, qy, qz, qw = self.rpy_to_quaternion(roll, pitch, yaw)
        else:
            # Fixed joint - just use origin transform
            qx, qy, qz, qw = self.rpy_to_quaternion(roll, pitch, yaw)

        # Set transform
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = tz
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        return transform

    @staticmethod
    def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
        """Convert roll-pitch-yaw to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    @staticmethod
    def multiply_quaternions(
        q1x: float, q1y: float, q1z: float, q1w: float,
        q2x: float, q2y: float, q2z: float, q2w: float
    ) -> tuple:
        """Multiply two quaternions: result = q1 * q2"""
        qx = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y
        qy = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x
        qz = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w
        qw = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z
        return qx, qy, qz, qw

    # ================================================================
    # TF 转换服务（新增）
    # ================================================================
    def _transform_points_callback(
        self,
        request: TransformPoints.Request,
        response: TransformPoints.Response
    ) -> TransformPoints.Response:
        """
        Service callback for transforming points between coordinate frames.
        
        Args:
            request: 包含 source_frame, target_frame, points_in
            response: 包含 success, message, points_out
        """
        source_frame = request.source_frame
        target_frame = request.target_frame
        points_in = request.points_in

        # 验证输入
        if len(points_in) == 0:
            response.success = False
            response.message = "Empty points array"
            response.points_out = []
            return response

        if len(points_in) % 3 != 0:
            response.success = False
            response.message = f"Points array length ({len(points_in)}) must be multiple of 3"
            response.points_out = []
            return response

        try:
            # 查询变换
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 转换点
            points_out = self._do_transform_points(points_in, transform)

            response.success = True
            response.message = ""
            response.points_out = points_out

        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            response.success = False
            response.message = str(e)
            response.points_out = []

        return response

    def _do_transform_points(
        self,
        points_in: List[float],
        transform: TransformStamped
    ) -> List[float]:
        """
        Apply transform to points array.
        
        Args:
            points_in: 输入点 [x1,y1,z1, x2,y2,z2, ...]
            transform: TF 变换
            
        Returns:
            转换后的点 [x1',y1',z1', x2',y2',z2', ...]
        """
        # 提取变换参数
        t = transform.transform.translation
        q = transform.transform.rotation

        # 构建变换矩阵
        T = self._transform_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)

        # 将点数组转换为 (N, 3)
        points = np.array(points_in).reshape(-1, 3)

        # 添加齐次坐标
        ones = np.ones((points.shape[0], 1))
        points_homo = np.hstack([points, ones])  # (N, 4)

        # 应用变换
        transformed = (T @ points_homo.T).T[:, :3]  # (N, 3)

        # 转换回列表
        return transformed.flatten().tolist()

    @staticmethod
    def _transform_to_matrix(
        tx: float, ty: float, tz: float,
        qx: float, qy: float, qz: float, qw: float
    ) -> np.ndarray:
        """
        Convert translation + quaternion to 4x4 transformation matrix.
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


def main(args=None):
    rclpy.init(args=args)
    node = RobotTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

