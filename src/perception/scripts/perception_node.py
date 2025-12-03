#!/usr/bin/env python3
"""
Perception Node 入口脚本
"""

# ⚠️ 关键：必须在导入torch之前设置PyTorch CUDA内存分配器配置
# PyTorch的内存分配器在导入时初始化，之后设置环境变量无效
import os

# 设置PyTorch CUDA内存分配器配置（必须在导入torch之前）
# max_split_size_mb: 限制最大内存块大小，避免大块分配失败
# expandable_segments:False: 禁用可扩展段，减少内存碎片
# roundup_power2_divisions: 内存对齐优化
os.environ.setdefault('PYTORCH_CUDA_ALLOC_CONF', 'max_split_size_mb:128,expandable_segments:False')

# ⚠️ 尝试禁用NVML查询（如果PyTorch支持）
# 在某些Jetson设备上，NVML可能不可用，导致内存分配失败
# 注意：这个环境变量可能不被所有PyTorch版本支持
os.environ.setdefault('PYTORCH_NO_CUDA_MEMORY_CACHING', '0')

from perception_core.node import main

if __name__ == '__main__':
    main()

