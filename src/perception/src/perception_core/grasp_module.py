"""
GraspModule - 抓取位姿推理模块

封装 Contact-GraspNet PyTorch 模型，提供 6-DoF 抓取位姿生成能力
参考: SegmentationModule 的实现模式
"""

import asyncio
import logging
import os
import sys
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

import numpy as np
import yaml

from .constants import (
    DEFAULT_MODEL_DIR,
    GRASP_MODEL_FILENAME,
    GRASP_CONFIG_FILENAME,
    DEFAULT_MAX_CANDIDATES,
    DEFAULT_MIN_GRASP_CONFIDENCE,
    DEFAULT_NUM_INPUT_POINTS,
    TIMEOUT_GRASP,
    Z_RANGE_MIN,
    Z_RANGE_MAX,
)
from .error_codes import PerceptionError, ErrorCode
from .async_worker import AsyncWorker

# 延迟导入 PyTorch 和 Contact-GraspNet（可能不可用）
TORCH_AVAILABLE = False
CONTACT_GRASPNET_AVAILABLE = False
IMPORT_ERROR = None

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError as exc:
    IMPORT_ERROR = exc

logger = logging.getLogger(__name__)


@dataclass
class GraspCandidate:
    """抓取候选结果"""
    position: np.ndarray              # 夹爪中心位置 (x, y, z)
    orientation: np.ndarray           # 夹爪朝向四元数 (x, y, z, w)
    width: float                      # 建议夹爪开口宽度（米）
    confidence: float                 # 置信度 (0.0 - 1.0)
    contact_points: np.ndarray        # 接触点 (x, y, z)
    transform_matrix: np.ndarray      # 4x4 变换矩阵


@dataclass
class GraspResult:
    """抓取推理结果"""
    candidates: List[GraspCandidate] = field(default_factory=list)
    inference_time_ms: float = 0.0
    point_count: int = 0


class GraspModule:
    """
    抓取位姿推理模块
    
    封装 Contact-GraspNet PyTorch 模型，提供基于点云的抓取位姿生成能力
    
    线程安全设计：
    - 使用 AsyncWorker 在独立线程池中执行推理
    - 使用 threading.Lock 保护模型访问
    - 使用独立的 CUDA Stream 执行 GPU 操作
    """
    
    def __init__(self, config: Dict[str, Any], worker: AsyncWorker):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                - model_path: 模型权重路径
                - config_path: 模型配置路径
                - max_candidates: 默认最大候选数量
                - min_confidence: 默认最小置信度
                - z_range: [min, max] 点云深度范围
            worker: AsyncWorker 实例（与 SegmentationModule 共享）
        """
        if worker is None:
            raise ValueError("GraspModule 需要有效的 AsyncWorker 实例")
        
        self._config = config
        self._worker = worker
        self._ready = False
        
        # 模型相关
        self._model = None
        self._model_config = None
        self._device = None
        self._stream = None
        
        # ⚠️ 必须：线程锁保护模型访问
        self._model_lock = threading.Lock()
        
        # 配置参数
        self._max_candidates = config.get('max_candidates', DEFAULT_MAX_CANDIDATES)
        self._min_confidence = config.get('min_confidence', DEFAULT_MIN_GRASP_CONFIDENCE)
        self._num_input_points = config.get('num_input_points', DEFAULT_NUM_INPUT_POINTS)
        self._timeout = config.get('timeout', TIMEOUT_GRASP)
        z_range = config.get('z_range', [Z_RANGE_MIN, Z_RANGE_MAX])
        self._z_range_min = z_range[0]
        self._z_range_max = z_range[1]
        
        # 性能统计
        self._inference_times = []
        
        logger.info("GraspModule 配置已加载")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载 Contact-GraspNet 模型
        
        Returns:
            bool: 是否初始化成功
            
        Raises:
            PerceptionError: 初始化失败时抛出
        """
        try:
            # ⚠️ 必须：通过 run_callable 在 AsyncWorker 线程中执行
            return await self._worker.run_callable(self._initialize_sync)
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"GraspModule 初始化失败: {e}")
            raise PerceptionError(
                ErrorCode.GRASP_MODEL_LOAD_FAILED,
                f"GraspModule 初始化失败: {str(e)}"
            )

    def _initialize_sync(self) -> bool:
        """
        实际的模型加载流程，在 AsyncWorker 线程池中执行
        
        这确保 CUDA context 在正确的线程中创建
        """
        start_time = time.time()
        
        # 检查 PyTorch 是否可用
        if not TORCH_AVAILABLE:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"PyTorch 未安装: {IMPORT_ERROR}"
            )
        
        # 检查 CUDA
        if not torch.cuda.is_available():
            raise PerceptionError(
                ErrorCode.CUDA_NOT_AVAILABLE,
                "CUDA 不可用"
            )
        
        # ⚠️ 必须：显式设置 CUDA 设备
        self._device = torch.device("cuda:0")
        torch.cuda.set_device(self._device)
        
        # ⚠️ 必须：禁用 cuDNN（Jetson 兼容性）
        torch.backends.cudnn.enabled = False
        
        # ⚠️ 注意：内存分配配置必须在导入torch之前设置才有效
        # 这里只是确保配置存在（应该在入口脚本中已设置）
        # 如果未设置，尝试设置（虽然可能无效）
        if 'PYTORCH_CUDA_ALLOC_CONF' not in os.environ:
            logger.warning("PYTORCH_CUDA_ALLOC_CONF未在导入torch前设置，内存配置可能无效")
            os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:False'
        else:
            logger.info(f"PyTorch CUDA内存配置: {os.environ.get('PYTORCH_CUDA_ALLOC_CONF')}")
        
        # ⚠️ 必须：创建独立的 CUDA Stream
        self._stream = torch.cuda.Stream(device=self._device)
        
        # 清理初始化前的缓存并记录内存状态
        try:
            if torch.cuda.is_available():
                allocated_init = torch.cuda.memory_allocated() / 1024**2
                reserved_init = torch.cuda.memory_reserved() / 1024**2
                total_init = torch.cuda.get_device_properties(0).total_memory / 1024**2
                logger.info(
                    f"[GPU内存] GraspModule初始化前: "
                    f"已分配={allocated_init:.1f}MB, "
                    f"已保留={reserved_init:.1f}MB, "
                    f"总计={total_init:.1f}MB"
                )
            
            torch.cuda.empty_cache()
        except Exception as e:
            logger.warning(f"初始化前清理缓存失败（可忽略）: {e}")
        
        # 解析模型路径
        model_path = self._resolve_model_path(
            self._config.get('model_path', GRASP_MODEL_FILENAME)
        )
        config_path = self._resolve_model_path(
            self._config.get('config_path', GRASP_CONFIG_FILENAME)
        )
        
        # 验证文件存在
        if not model_path.exists():
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"模型权重文件不存在: {model_path}"
            )
        if not config_path.exists():
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"模型配置文件不存在: {config_path}"
            )
        
        logger.info(f"加载 Contact-GraspNet 配置: {config_path}")
        logger.info(f"加载 Contact-GraspNet 权重: {model_path}")
        
        # 加载配置
        with open(config_path, 'r') as f:
            self._model_config = yaml.safe_load(f)
        
        # 加载模型
        load_start = time.time()
        self._load_model(str(model_path))
        load_time = time.time() - load_start
        
        # 记录模型加载后的内存状态
        try:
            if torch.cuda.is_available():
                allocated_after_load = torch.cuda.memory_allocated() / 1024**2
                reserved_after_load = torch.cuda.memory_reserved() / 1024**2
                logger.info(
                    f"Contact-GraspNet 模型加载完成 (耗时={load_time:.2f}s) | "
                    f"GPU内存: 已分配={allocated_after_load:.1f}MB, 已保留={reserved_after_load:.1f}MB"
                )
        except Exception as e:
            logger.debug(f"无法记录模型加载后的内存状态: {e}")
        
        # Warmup
        warmup_start = time.time()
        self._warmup()
        warmup_time = time.time() - warmup_start
        
        # 记录warmup后的内存状态
        try:
            if torch.cuda.is_available():
                allocated_after_warmup = torch.cuda.memory_allocated() / 1024**2
                reserved_after_warmup = torch.cuda.memory_reserved() / 1024**2
                logger.info(
                    f"Contact-GraspNet warmup 完成 (耗时={warmup_time:.2f}s) | "
                    f"GPU内存: 已分配={allocated_after_warmup:.1f}MB, 已保留={reserved_after_warmup:.1f}MB"
                )
        except Exception as e:
            logger.debug(f"无法记录warmup后的内存状态: {e}")
        
        self._ready = True
        total_time = time.time() - start_time
        
        # 记录最终初始化完成的内存状态
        try:
            if torch.cuda.is_available():
                allocated_final = torch.cuda.memory_allocated() / 1024**2
                reserved_final = torch.cuda.memory_reserved() / 1024**2
                total_final = torch.cuda.get_device_properties(0).total_memory / 1024**2
                free_final = total_final - reserved_final
                logger.info(
                    f"GraspModule 初始化完成 (总耗时={total_time:.2f}s) | "
                    f"GPU内存: 已分配={allocated_final:.1f}MB, 已保留={reserved_final:.1f}MB, "
                    f"总计={total_final:.1f}MB, 可用={free_final:.1f}MB"
                )
        except Exception as e:
            logger.debug(f"无法记录初始化完成的内存状态: {e}")
        
        return True
    
    def _resolve_model_path(self, path_str: str) -> Path:
        """解析模型文件路径"""
        path = Path(path_str)
        if path.is_absolute():
            return path
        
        # 尝试相对于包目录
        package_dir = Path(__file__).parent.parent.parent
        candidate = package_dir / path_str
        if candidate.exists():
            return candidate
        
        # 尝试 models/contact_graspnet 目录
        models_dir = package_dir / 'models' / 'contact_graspnet'
        candidate = models_dir / Path(path_str).name
        if candidate.exists():
            return candidate
        
        return path
    
    def _load_model(self, model_path: str):
        """加载 Contact-GraspNet 模型"""
        # 添加 external 路径到 sys.path
        # 使用 .resolve() 解析符号链接，确保从源代码目录计算路径
        source_file = Path(__file__).resolve()
        external_dir = source_file.parent.parent.parent.parent.parent / 'external'
        contact_graspnet_dir = external_dir / 'contact_graspnet_pytorch'
        
        if str(contact_graspnet_dir) not in sys.path:
            sys.path.insert(0, str(contact_graspnet_dir))
        
        # 动态导入 Contact-GraspNet
        try:
            from contact_graspnet_pytorch.contact_graspnet import ContactGraspnet
            from contact_graspnet_pytorch.checkpoints import CheckpointIO
        except ImportError as e:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"无法导入 Contact-GraspNet: {e}"
            )
        
        # 创建模型
        self._model = ContactGraspnet(self._model_config, self._device)
        self._model.to(self._device)
        
        # 加载权重
        checkpoint_dir = Path(model_path).parent
        checkpoint_io = CheckpointIO(
            checkpoint_dir=str(checkpoint_dir),
            model=self._model
        )
        
        try:
            checkpoint_io.load(Path(model_path).name)
            logger.info("Contact-GraspNet 权重加载成功")
        except Exception as e:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"模型权重加载失败: {e}"
            )
        
        # 设置为评估模式
        self._model.eval()
    
    def _warmup(self):
        """预热模型"""
        if self._model is None:
            return
        
        try:
            # 创建虚拟点云数据
            dummy_pc = np.random.rand(self._num_input_points, 3).astype(np.float32)
            dummy_pc[:, 2] = dummy_pc[:, 2] * 0.5 + 0.3  # Z 范围 0.3-0.8
            
            # 执行一次推理
            with self._model_lock:
                with torch.cuda.stream(self._stream):
                    with torch.no_grad():
                        pc_batch = torch.from_numpy(dummy_pc[np.newaxis, :, :]).to(self._device)
                        pred = self._model(pc_batch)
                        # 显式删除warmup的tensor
                        del pc_batch
                        del pred
                torch.cuda.current_stream().wait_stream(self._stream)
            
            # warmup后清理缓存
            try:
                torch.cuda.empty_cache()
            except Exception as e:
                logger.warning(f"Warmup后清理缓存失败（可忽略）: {e}")
            logger.info("Contact-GraspNet warmup 完成")
        except Exception as e:
            logger.warning(f"Contact-GraspNet warmup 失败: {e}")
            # warmup失败后也尝试清理
            try:
                torch.cuda.empty_cache()
            except Exception:
                pass
    
    async def predict(
        self,
        point_cloud: np.ndarray,
        mask: Optional[np.ndarray] = None,
        max_candidates: Optional[int] = None,
        min_confidence: Optional[float] = None
    ) -> GraspResult:
        """
        执行抓取位姿推理
        
        Args:
            point_cloud: Nx3 点云数据（相机坐标系）
            mask: 可选的分割掩码，用于过滤点云
            max_candidates: 最大候选数量（可选）
            min_confidence: 最小置信度阈值（可选）
            
        Returns:
            GraspResult: 抓取推理结果
            
        Raises:
            PerceptionError: 推理失败时抛出
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY, "抓取模块未就绪")
        
        # 使用默认值
        if max_candidates is None:
            max_candidates = self._max_candidates
        if min_confidence is None:
            min_confidence = self._min_confidence
        
        try:
            return await asyncio.wait_for(
                self._worker.run_callable(
                    self._predict_sync,
                    point_cloud,
                    mask,
                    max_candidates,
                    min_confidence
                ),
                timeout=self._timeout,
            )
        except asyncio.TimeoutError as exc:
            raise PerceptionError(
                ErrorCode.GRASP_INFERENCE_TIMEOUT,
                f"抓取推理超时 (> {self._timeout:.1f}s)"
            ) from exc
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"抓取推理失败: {e}")
            raise PerceptionError(
                ErrorCode.GRASP_INFERENCE_FAILED,
                f"抓取推理失败: {str(e)}"
            )

    def _predict_sync(
        self,
        point_cloud: np.ndarray,
        mask: Optional[np.ndarray],
        max_candidates: int,
        min_confidence: float
    ) -> GraspResult:
        """
        同步执行抓取推理（运行在 AsyncWorker 的线程池中）
        
        Args:
            point_cloud: Nx3 点云数据
            mask: 可选掩码
            max_candidates: 最大候选数量
            min_confidence: 最小置信度
        """
        start_time = time.time()
        logger.info(f"[Grasp推理] 开始推理: 点数={point_cloud.shape[0] if point_cloud is not None else 0}")
        print(f"[Grasp-Inference] START: points={point_cloud.shape[0] if point_cloud is not None else 0}")
        
        # 验证输入
        if point_cloud is None or point_cloud.size == 0:
            raise PerceptionError(
                ErrorCode.POINT_COUNT_INSUFFICIENT,
                "输入点云为空"
            )
        
        if point_cloud.ndim != 2 or point_cloud.shape[1] < 3:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"点云格式错误，期望 (N, 3)，实际 {point_cloud.shape}"
            )
        
        # 只取 XYZ 坐标
        pc = point_cloud[:, :3].astype(np.float32)
        
        # 应用掩码过滤（如果提供）
        if mask is not None:
            pc = self._apply_mask_filter(pc, mask)
        
        # 深度范围过滤
        pc = self._filter_by_z_range(pc)
        
        if pc.shape[0] < 100:
            raise PerceptionError(
                ErrorCode.POINT_COUNT_INSUFFICIENT,
                f"有效点云数量不足: {pc.shape[0]}"
            )
        
        original_point_count = pc.shape[0]
        
        # 预处理点云
        pc_processed, pc_mean = self._preprocess_point_cloud(pc)
        
        # ⚠️ 推理前：激进的内存清理策略
        # 1. 清理PyTorch缓存
        # 2. 同步CUDA操作，确保所有操作完成
        # 3. 重置内存统计（如果支持）
        memory_before_cleanup = None
        try:
            if torch.cuda.is_available():
                allocated_before = torch.cuda.memory_allocated() / 1024**2
                reserved_before = torch.cuda.memory_reserved() / 1024**2
                memory_before_cleanup = (allocated_before, reserved_before)
            
            # 同步所有CUDA操作
            torch.cuda.synchronize()
            # 清理缓存
            torch.cuda.empty_cache()
            # 尝试重置内存峰值统计（释放一些内部缓存）
            if hasattr(torch.cuda, 'reset_peak_memory_stats'):
                torch.cuda.reset_peak_memory_stats()
        except Exception as e:
            logger.warning(f"推理前清理缓存失败（可忽略）: {e}")
        
        # ⚠️ 详细的内存状态监控和日志（强制输出，即使NVML失败）
        memory_info = {}
        memory_query_failed = False
        try:
            if torch.cuda.is_available():
                allocated = torch.cuda.memory_allocated() / 1024**2  # MB
                reserved = torch.cuda.memory_reserved() / 1024**2    # MB
                total = torch.cuda.get_device_properties(0).total_memory / 1024**2  # MB
                free = total - reserved
                
                memory_info = {
                    'allocated': allocated,
                    'reserved': reserved,
                    'total': total,
                    'free': free
                }
                
                # 计算清理效果
                cleanup_effect = ""
                if memory_before_cleanup:
                    allocated_before, reserved_before = memory_before_cleanup
                    allocated_freed = allocated_before - allocated
                    reserved_freed = reserved_before - reserved
                    cleanup_effect = f" (清理释放: 已分配-{allocated_freed:.1f}MB, 已保留-{reserved_freed:.1f}MB)"
                
                # ⚠️ 强制输出内存信息（使用error级别确保输出）
                msg = (
                    f"[GPU内存] 推理前状态: "
                    f"已分配={allocated:.1f}MB, "
                    f"已保留={reserved:.1f}MB, "
                    f"总计={total:.1f}MB, "
                    f"可用={free:.1f}MB"
                    f"{cleanup_effect}"
                )
                logger.info(msg)
                # 同时print确保输出（用于调试）
                print(f"[GPU-MEM] {msg}")
                
                # 如果可用内存不足500MB，发出警告
                if free < 500:
                    logger.error(f"[GPU内存] ⚠️ 严重警告: 仅剩{free:.1f}MB可用，推理很可能失败！")
                    print(f"[GPU-MEM-ERROR] 仅剩{free:.1f}MB可用！")
                elif free < 1000:
                    logger.warning(f"[GPU内存] ⚠️ 警告: 可用内存仅{free:.1f}MB，推理可能失败")
                    print(f"[GPU-MEM-WARN] 可用内存仅{free:.1f}MB")
        except Exception as e:
            # NVML查询失败是正常的（Jetson上常见），但强制记录
            memory_query_failed = True
            error_msg = f"[GPU内存] ⚠️ 无法查询GPU内存状态（NVML可能不可用）: {e}"
            logger.warning(error_msg)
            print(f"[GPU-MEM-ERROR] {error_msg}")
            memory_info = {'error': str(e)}
        
        # ⚠️ 必须：加锁保护模型访问
        pc_batch = None
        pred = None
        inference_start_time = time.time()
        try:
            lock_start = time.time()
            logger.debug(f"[Grasp推理] 等待模型锁...")
            print(f"[Grasp-Inference] Waiting for model lock...")
            
        with self._model_lock:
                lock_acquired_time = time.time() - lock_start
                if lock_acquired_time > 0.1:
                    logger.warning(f"[Grasp推理] 获取模型锁耗时{lock_acquired_time:.2f}s（可能被其他推理占用）")
                    print(f"[Grasp-Inference-WARN] Lock acquired after {lock_acquired_time:.2f}s")
                
                logger.debug(f"[Grasp推理] 开始GPU推理...")
                print(f"[Grasp-Inference] Starting GPU inference...")
                
            with torch.cuda.stream(self._stream):
                with torch.no_grad():
                    # 转换为 batch tensor
                        tensor_start = time.time()
                    pc_batch = torch.from_numpy(
                        pc_processed[np.newaxis, :, :]
                    ).to(self._device)
                        tensor_time = time.time() - tensor_start
                        if tensor_time > 0.5:
                            logger.warning(f"[Grasp推理] Tensor转换耗时{tensor_time:.2f}s（可能内存分配慢）")
                            print(f"[Grasp-Inference-WARN] Tensor creation took {tensor_time:.2f}s")
                    
                    # 模型推理
                        model_start = time.time()
                        logger.debug(f"[Grasp推理] 执行模型前向传播...")
                        print(f"[Grasp-Inference] Running model forward pass...")
                    pred = self._model(pc_batch)
                        model_time = time.time() - model_start
                        logger.info(f"[Grasp推理] 模型推理完成，耗时={model_time:.2f}s")
                        print(f"[Grasp-Inference] Model inference done: {model_time:.2f}s")
                    
                        # 提取结果到CPU（立即释放GPU内存）
                        extract_start = time.time()
                    pred_grasps = pred['pred_grasps_cam'].detach().cpu().numpy()
                    pred_scores = pred['pred_scores'].detach().cpu().numpy()
                    pred_points = pred['pred_points'].detach().cpu().numpy()
                    offset_pred = pred['offset_pred'].detach().cpu().numpy()
                        extract_time = time.time() - extract_start
                        logger.debug(f"[Grasp推理] 结果提取完成，耗时={extract_time:.2f}s")
                        print(f"[Grasp-Inference] Results extracted: {extract_time:.2f}s")
                        
                        # ⚠️ 显式删除GPU tensor，立即释放内存
                        del pc_batch
                        pc_batch = None
                        del pred
                        pred = None
            
            # 同步 CUDA 操作
                sync_start = time.time()
            torch.cuda.current_stream().wait_stream(self._stream)
                sync_time = time.time() - sync_start
                if sync_time > 0.1:
                    logger.warning(f"[Grasp推理] CUDA同步耗时{sync_time:.2f}s")
                    print(f"[Grasp-Inference-WARN] CUDA sync took {sync_time:.2f}s")
                
                inference_total_time = time.time() - inference_start_time
                logger.info(f"[Grasp推理] GPU推理阶段总耗时={inference_total_time:.2f}s")
                print(f"[Grasp-Inference] GPU inference total: {inference_total_time:.2f}s")
        finally:
            # ⚠️ 确保异常情况下也能清理GPU tensor
            if pc_batch is not None:
                try:
                    del pc_batch
                except Exception:
                    pass
            if pred is not None:
                try:
                    del pred
                except Exception:
                    pass
            
        # ⚠️ 推理后：清理GPU缓存，释放中间分配的内存
        # 确保内存及时释放，避免累积导致OOM
        memory_after_inference = None
        try:
            if torch.cuda.is_available() and not memory_query_failed:
                try:
                    allocated_after = torch.cuda.memory_allocated() / 1024**2
                    reserved_after = torch.cuda.memory_reserved() / 1024**2
                    memory_after_inference = (allocated_after, reserved_after)
                except Exception:
                    pass  # NVML可能失败，忽略
            
            torch.cuda.empty_cache()
            
            # 记录清理后的内存状态（强制输出）
            if torch.cuda.is_available() and memory_info and not memory_query_failed:
                try:
                    allocated_final = torch.cuda.memory_allocated() / 1024**2
                    reserved_final = torch.cuda.memory_reserved() / 1024**2
                    total = memory_info.get('total', 0)
                    free_final = total - reserved_final
                    
                    # 计算推理过程中的内存变化
                    if memory_after_inference:
                        allocated_after, reserved_after = memory_after_inference
                        inference_allocated = allocated_after - memory_info.get('allocated', 0)
                        inference_reserved = reserved_after - memory_info.get('reserved', 0)
                        cleanup_freed = reserved_after - reserved_final
                        
                        msg = (
                            f"[GPU内存] 推理后状态: "
                            f"已分配={allocated_final:.1f}MB, "
                            f"已保留={reserved_final:.1f}MB, "
                            f"可用={free_final:.1f}MB | "
                            f"推理占用: +{inference_allocated:.1f}MB(分配)/+{inference_reserved:.1f}MB(保留), "
                            f"清理释放: {cleanup_freed:.1f}MB"
                        )
                        logger.info(msg)
                        print(f"[GPU-MEM] {msg}")
                except Exception as e:
                    logger.debug(f"推理后无法查询内存状态: {e}")
        except Exception as e:
            logger.warning(f"推理后清理缓存失败（可忽略）: {e}")
        
        # 后处理
        postprocess_start = time.time()
        logger.debug(f"[Grasp推理] 开始后处理...")
        print(f"[Grasp-Inference] Starting post-processing...")
        
        candidates = self._postprocess_predictions(
            pred_grasps,
            pred_scores,
            pred_points,
            offset_pred,
            pc_mean,
            max_candidates,
            min_confidence
        )
        
        postprocess_time = time.time() - postprocess_start
        logger.debug(f"[Grasp推理] 后处理完成，耗时={postprocess_time:.2f}s，候选数={len(candidates)}")
        print(f"[Grasp-Inference] Post-processing done: {postprocess_time:.2f}s, candidates={len(candidates)}")
        
        # 记录推理时间
        inference_time = (time.time() - start_time) * 1000
        self._inference_times.append(inference_time)
        
        total_time = time.time() - start_time
        logger.info(
            f"[Grasp推理] 完成: {len(candidates)} 候选, "
            f"推理时间={inference_time:.1f}ms, 总耗时={total_time:.2f}s, 点数={original_point_count}"
        )
        print(f"[Grasp-Inference] COMPLETE: {len(candidates)} candidates, total={total_time:.2f}s")
        
        return GraspResult(
            candidates=candidates,
            inference_time_ms=inference_time,
            point_count=original_point_count
        )
    
    def _apply_mask_filter(
        self,
        point_cloud: np.ndarray,
        mask: np.ndarray
    ) -> np.ndarray:
        """根据掩码过滤点云"""
        # 假设掩码是与深度图对应的，需要重新映射
        # 这里简化处理，假设 mask 是点云的布尔索引
        if mask.dtype == bool and len(mask) == len(point_cloud):
            return point_cloud[mask]
        return point_cloud
    
    def _filter_by_z_range(self, point_cloud: np.ndarray) -> np.ndarray:
        """根据 Z 轴范围过滤点云"""
        z_mask = (point_cloud[:, 2] >= self._z_range_min) & \
                 (point_cloud[:, 2] <= self._z_range_max)
        return point_cloud[z_mask]
    
    def _preprocess_point_cloud(
        self,
        point_cloud: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        预处理点云
        
        1. 调整点数到 num_input_points
        2. 转换到内部坐标系
        3. 中心化
        """
        pc = point_cloud.copy()
        num_points = self._num_input_points
        
        # 调整点数
        if pc.shape[0] > num_points:
            # 随机下采样
            indices = np.random.choice(pc.shape[0], num_points, replace=False)
            pc = pc[indices]
        elif pc.shape[0] < num_points:
            # 上采样（重复采样）
            required = num_points - pc.shape[0]
            indices = np.random.choice(pc.shape[0], required, replace=True)
            pc = np.concatenate([pc, pc[indices]], axis=0)
        
        # 转换到内部坐标系 (x left, y up, z front)
        # OpenCV: x right, y down, z front -> Internal: x left, y up, z front
        pc[:, :2] *= -1
        
        # 计算中心并中心化
        pc_mean = np.mean(pc, axis=0)
        pc = pc - pc_mean
        
        return pc.astype(np.float32), pc_mean
    
    def _postprocess_predictions(
        self,
        pred_grasps: np.ndarray,
        pred_scores: np.ndarray,
        pred_points: np.ndarray,
        offset_pred: np.ndarray,
        pc_mean: np.ndarray,
        max_candidates: int,
        min_confidence: float
    ) -> List[GraspCandidate]:
        """
        后处理模型预测
        
        1. 还原到原始坐标系
        2. 按置信度排序
        3. 选择 top-k 候选
        4. 转换为 GraspCandidate 格式
        """
        # Reshape predictions
        pred_grasps = pred_grasps.reshape(-1, 4, 4)  # N x 4 x 4
        pred_scores = pred_scores.reshape(-1)        # N
        pred_points = pred_points.reshape(-1, 3)     # N x 3
        offset_pred = offset_pred.reshape(-1)        # N
        
        # 还原到原始坐标系
        # Internal -> OpenCV: x left -> x right, y up -> y down
        pred_grasps[:, :2, :] *= -1
        pred_points[:, :2] *= -1
        
        # 还原中心偏移
        pred_grasps[:, :3, 3] += pc_mean.reshape(1, 3)
        pred_points[:, :3] += pc_mean.reshape(1, 3)
        
        # 计算夹爪开口宽度
        gripper_width = self._model_config['DATA']['gripper_width']
        extra_opening = self._model_config['TEST'].get('extra_opening', 0.005)
        gripper_openings = np.minimum(offset_pred + extra_opening, gripper_width)
        
        # ⚠️ 诊断：记录模型输出的统计信息
        total_predictions = len(pred_scores)
        max_score = float(np.max(pred_scores)) if total_predictions > 0 else 0.0
        min_score = float(np.min(pred_scores)) if total_predictions > 0 else 0.0
        mean_score = float(np.mean(pred_scores)) if total_predictions > 0 else 0.0
        
        logger.info(
            f"[Grasp后处理] 模型输出统计: "
            f"总数={total_predictions}, "
            f"最大置信度={max_score:.3f}, "
            f"最小置信度={min_score:.3f}, "
            f"平均置信度={mean_score:.3f}, "
            f"阈值={min_confidence:.3f}"
        )
        print(f"[Grasp-PostProcess] Total={total_predictions}, Max={max_score:.3f}, Min={min_score:.3f}, Mean={mean_score:.3f}, Threshold={min_confidence:.3f}")
        
        # 过滤低置信度
        valid_mask = pred_scores >= min_confidence
        valid_count = int(np.sum(valid_mask))
        
        logger.info(
            f"[Grasp后处理] 置信度过滤: "
            f"有效候选={valid_count}/{total_predictions}, "
            f"过滤掉={total_predictions - valid_count}个"
        )
        print(f"[Grasp-PostProcess] Valid candidates: {valid_count}/{total_predictions}")
        
        if valid_count == 0:
            logger.warning(
                f"[Grasp后处理] ⚠️ 所有候选都被过滤！"
                f"最大置信度={max_score:.3f} < 阈值={min_confidence:.3f}"
            )
            print(f"[Grasp-PostProcess-WARN] All candidates filtered! Max={max_score:.3f} < Threshold={min_confidence:.3f}")
        
        # 排序并选择 top-k
        sorted_indices = np.argsort(pred_scores[valid_mask])[::-1]
        selected_indices = np.where(valid_mask)[0][sorted_indices[:max_candidates]]
        
        candidates = []
        for idx in selected_indices:
            # 提取变换矩阵
            transform = pred_grasps[idx]  # 4x4
            
            # 提取位置
            position = transform[:3, 3]
            
            # 提取旋转矩阵并转换为四元数
            rotation_matrix = transform[:3, :3]
            quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)
            
            candidates.append(GraspCandidate(
                position=position.copy(),
                orientation=quaternion.copy(),
                width=float(gripper_openings[idx]),
                confidence=float(pred_scores[idx]),
                contact_points=pred_points[idx].copy(),
                transform_matrix=transform.copy()
            ))
        
        return candidates
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """
        将旋转矩阵转换为四元数 (x, y, z, w)
        
        Args:
            R: 3x3 旋转矩阵
            
        Returns:
            np.ndarray: 四元数 (x, y, z, w)
        """
        # 使用 Shepperd 方法
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        # 归一化
        q = np.array([x, y, z, w])
        q = q / np.linalg.norm(q)
        
        return q
    
    def get_performance_stats(self) -> Dict[str, float]:
        """获取性能统计"""
        stats = {}
        
        if self._inference_times:
            times = np.array(self._inference_times)
            stats['avg_inference_time_ms'] = float(np.mean(times))
            stats['max_inference_time_ms'] = float(np.max(times))
            stats['min_inference_time_ms'] = float(np.min(times))
            stats['inference_count'] = len(self._inference_times)
        
        return stats

