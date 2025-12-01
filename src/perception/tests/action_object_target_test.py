#!/usr/bin/env python3
"""ObjectTarget Action 手动测试脚本.

该脚本会:
1. 订阅相机 RGB、深度与 CameraInfo 话题, 等待各获取一帧.
2. 将这些消息填入 ObjectTarget Action Goal (点击坐标默认 320x240).
3. 向 /perception/action/object_target 发送请求, 打印反馈与结果概括.

运行前请确保 camera 与 perception 节点已启动.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

_ENV_FLAG = 'OBJECT_TARGET_TEST_ENV_READY'

if os.environ.get(_ENV_FLAG) != '1':
    script_path = Path(__file__).resolve()
    workspace = script_path.parents[3]
    install_setup = workspace / 'install' / 'setup.zsh'
    if not install_setup.exists():
        print(f'[ERROR] 未找到 {install_setup}, 请先编译并生成 install 目录.', file=sys.stderr)
        sys.exit(1)

    quoted_args = ' '.join(shlex.quote(arg) for arg in sys.argv[1:])
    reexec_cmd = (
        f'source /opt/ros/humble/setup.zsh && '
        f'source \"{install_setup}\" && '
        f'{_ENV_FLAG}=1 python3 \"{script_path}\" {quoted_args}'
    )
    result = subprocess.call(['zsh', '-c', reexec_cmd], cwd=str(workspace))
    sys.exit(result)

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image

from perception.action import ObjectTarget


class ObjectTargetActionTester(Node):
    """订阅相机话题并调用 ObjectTarget Action."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('action_object_target_tester')
        qos_sensor = QoSPresetProfiles.SENSOR_DATA.value

        self._color_topic = args.color_topic
        self._depth_topic = args.depth_topic
        self._camera_info_topic = args.camera_info_topic
        self._action_name = args.action_name
        self._goal_timeout = args.goal_timeout

        self._color_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._camera_info_msg: Optional[CameraInfo] = None

        self._color_event = threading.Event()
        self._depth_event = threading.Event()
        self._camera_event = threading.Event()

        self.create_subscription(Image, self._color_topic, self._color_callback, qos_sensor)
        self.create_subscription(Image, self._depth_topic, self._depth_callback, qos_sensor)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_callback, 10)

        self._action_client = ActionClient(self, ObjectTarget, self._action_name)
        self.get_logger().info(
            "ObjectTargetActionTester 初始化完成, topics: "
            f"color={self._color_topic} depth={self._depth_topic} "
            f"caminfo={self._camera_info_topic}, action={self._action_name}"
        )

    def _color_callback(self, msg: Image) -> None:
        if self._color_msg is None:
            self.get_logger().info(
                f'收到第一帧 RGB ({msg.width}x{msg.height}, encoding={msg.encoding})'
            )
        self._color_msg = msg
        self._color_event.set()

    def _depth_callback(self, msg: Image) -> None:
        if self._depth_msg is None:
            self.get_logger().info(
                f'收到第一帧 Depth ({msg.width}x{msg.height}, encoding={msg.encoding})'
            )
        self._depth_msg = msg
        self._depth_event.set()

    def _camera_callback(self, msg: CameraInfo) -> None:
        if self._camera_info_msg is None:
            self.get_logger().info(
                f'收到 CameraInfo ({msg.width}x{msg.height}, frame={msg.header.frame_id})'
            )
        self._camera_info_msg = msg
        self._camera_event.set()

    def reset_messages(self) -> None:
        """清理上一轮缓存的图像, 保证每次执行使用最新帧."""
        self._color_msg = None
        self._depth_msg = None
        self._camera_info_msg = None
        self._color_event.clear()
        self._depth_event.clear()
        self._camera_event.clear()

    def wait_for_messages(self, timeout: float) -> bool:
        start = time.time()
        while True:
            if self._color_event.is_set() and self._depth_event.is_set() and self._camera_event.is_set():
                return True
            if timeout > 0 and (time.time() - start) > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_action_server(self, timeout: float) -> bool:
        self.get_logger().info(f'等待 Action Server ({self._action_name})...')
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def send_goal(self, click_x: float, click_y: float):
        if not all([self._color_msg, self._depth_msg, self._camera_info_msg]):
            self.get_logger().error('缺少必要输入, 无法构造 Goal')
            return None

        goal = ObjectTarget.Goal()
        goal.color_image = self._color_msg
        goal.depth_image = self._depth_msg
        goal.camera_info = self._camera_info_msg
        goal.click_x = float(click_x)
        goal.click_y = float(click_y)

        self.get_logger().info(
            '发送 Goal: '
            f'color={goal.color_image.width}x{goal.color_image.height} '
            f'depth={goal.depth_image.width}x{goal.depth_image.height} '
            f'click=({goal.click_x:.1f}, {goal.click_y:.1f})'
        )

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self._goal_timeout)
        if not send_future.done():
            self.get_logger().error(f'发送 Goal 超时 (> {self._goal_timeout:.1f}s)')
            return None

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Action Goal 被拒绝')
            return None

        self.get_logger().info('Goal 已接受, 等待结果...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._goal_timeout)
        if not result_future.done():
            self.get_logger().error(f'Action 处理超时 (> {self._goal_timeout:.1f}s)')
            goal_handle.cancel_goal_async()
            return None

        return result_future.result().result

    def _feedback_callback(self, feedback_msg: ObjectTarget.FeedbackMessage) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'反馈: status={feedback.status} progress={feedback.progress:.2f}'
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='ObjectTarget Action 端到端测试脚本')
    parser.add_argument('--color-topic', default='/camera/color/image_raw', help='RGB 图像话题')
    parser.add_argument(
        '--depth-topic', default='/camera/aligned_depth_to_color/image_raw', help='深度图话题'
    )
    parser.add_argument(
        '--camera-info-topic', default='/camera/color/camera_info', help='CameraInfo 话题'
    )
    parser.add_argument(
        '--action-name', default='/perception/action/object_target', help='Action 名称'
    )
    parser.add_argument('--click-x', type=float, default=320.0, help='点击像素 X')
    parser.add_argument('--click-y', type=float, default=240.0, help='点击像素 Y')
    parser.add_argument('--wait-timeout', type=float, default=5.0, help='等待传感器数据的超时秒数')
    parser.add_argument('--goal-timeout', type=float, default=30.0, help='Action 发送/等待超时秒数')
    parser.add_argument('--server-timeout', type=float, default=5.0, help='等待 Action Server 超时秒数')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ObjectTargetActionTester(args)
    try:
        if not node.wait_for_action_server(args.server_timeout):
            node.get_logger().error(f'Action Server 不可用: {args.action_name}')
            return 1

        node.get_logger().info('等待相机数据...')
        node.reset_messages()
        if not node.wait_for_messages(args.wait_timeout):
            node.get_logger().error(f'在 {args.wait_timeout:.1f}s 内未收到全部相机话题')
            return 1

        result = node.send_goal(args.click_x, args.click_y)
        if result is None:
            return 1

        if result.success:
            node.get_logger().info(
                f'Action 成功: mask_area={result.mask_area_pixels}, points={result.point_count}'
            )
            node.get_logger().info(
                '中心点 '
                f'({result.center_3d.x:.4f}, {result.center_3d.y:.4f}, {result.center_3d.z:.4f})'
            )
            return 0
        node.get_logger().error(f'Action 失败: {result.error_message}')
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
