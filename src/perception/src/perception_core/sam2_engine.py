#!/usr/bin/env python3
"""
SAM2 Engine for Zero-Shot Object Segmentation
Supports both PyTorch and TensorRT inference with feature caching
"""

import os
import sys
import time
import copy
import importlib
import numpy as np
import torch
import cv2
import threading
import atexit
from typing import Optional, Tuple, Dict, List, Any
import ast
import yaml
from pathlib import Path

try:
    # Try to import TensorRT from system
    # UV environments don't have TensorRT, but we can use system TensorRT
    if '/usr/lib/python3/dist-packages' not in sys.path:
        sys.path.append('/usr/lib/python3/dist-packages')
    
    import tensorrt as trt
    import pycuda.driver as cuda
    # Don't import pycuda.autoinit here - we'll initialize manually to avoid conflicts with PyTorch
    TRT_AVAILABLE = True
except ImportError as e:
    TRT_AVAILABLE = False
    print(f"⚠ TensorRT not available: {e}")
    print("  Falling back to PyTorch (slower but works)")
    print("  To enable TensorRT: run 'bash scripts/deployment/convert_models_system.sh'")


class SAM2Engine:
    """
    SAM2 inference engine with feature caching and TensorRT support.
    """

    def __init__(self, config_path: str, device: str = "cuda", package_share_dir: Optional[Path] = None):
        """
        Initialize SAM2 engine.

        Args:
            config_path: Path to SAM2 configuration YAML
            device: Device to run inference on ('cuda' or 'cpu')
            package_share_dir: Optional package share directory for resolving relative paths
        """
        self.device = device
        self._torch_has_cuda = torch.cuda.is_available()
        self._using_primary_cuda_context = False
        self.cuda_context = None
        self.cuda_device_id = 0
        self._context_cleaned = False
        self._debug_path = Path("/tmp/sam2_engine_debug.log")
        self.package_share_dir = Path(package_share_dir) if package_share_dir else None

        # Early CUDA initialization for Jetson - clear any stale state
        if device == "cuda" and self._torch_has_cuda:
            try:
                # Initialize CUDA context early
                torch.cuda.init()
                # Clear CUDA cache
                torch.cuda.empty_cache()
                # Set memory allocator config for better stability
                os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'
                print("✓ CUDA initialized and cache cleared")
            except Exception as e:
                print(f"Warning: CUDA initialization issue: {e}")
        
        if isinstance(config_path, dict):
            # If config_path is already a dict, use it directly
            self.config = config_path
        else:
            # Otherwise load from file
            config_path = Path(config_path)
            if not config_path.is_absolute():
                if self.package_share_dir:
                    config_path = self.package_share_dir / config_path
                else:
                    config_path = config_path.resolve()
            self.config = self._load_config(config_path)

        self._resolve_resource_paths()
        
        # Feature cache
        self.cached_features = None
        self.cached_image_hash = None
        self.cache_timestamp = None
        self._engine_lock = threading.RLock()
        
        # Performance tracking
        self.encode_times = []
        self.decode_times = []
        
        # Workaround for Jetson cuDNN issues
        # COMPLETELY disable cuDNN to avoid compatibility problems
        if device == "cuda" and self._torch_has_cuda:
            try:
                # Disable cuDNN entirely - use PyTorch native implementations
                torch.backends.cudnn.enabled = False
                print("✓ cuDNN completely disabled (using PyTorch native algorithms)")
                print("  This avoids cuDNN compatibility issues on Jetson")
            except Exception as e:
                print(f"Warning: Could not disable cuDNN: {e}")
        
        # Override device to CPU automatically if CUDA build is unavailable
        if device == "cuda" and not self._torch_has_cuda:
            print("⚠ Requested CUDA but torch was built without CUDA support; falling back to CPU for decoder.")
            self.device = "cpu"
        else:
            self.device = device

        # Initialize model
        self.use_tensorrt = self.config['sam2']['use_tensorrt'] and TRT_AVAILABLE
        if self.use_tensorrt:
            print("Initializing SAM2 with TensorRT acceleration...")
            self._init_tensorrt()
            atexit.register(self._cleanup_context)
        else:
            print("Initializing SAM2 with PyTorch...")
            self._init_pytorch()
        
        print(f"SAM2 Engine initialized successfully (TensorRT: {self.use_tensorrt})")
        self._debug_log(f"Engine init complete (TensorRT={self.use_tensorrt}, device={self.device}, torch_cuda={self._torch_has_cuda})")
    
    def __del__(self):
        """Cleanup CUDA context on deletion."""
        # ⚠️ 注意：__del__ 在 atexit 之前执行，但可能多次调用
        # 使用 _context_cleaned 标志确保只清理一次
        self._cleanup_context()

    def _cleanup_context(self):
        """Release retained CUDA context safely."""
        # ⚠️ 幂等性检查：确保只清理一次
        if not hasattr(self, '_engine_lock') or self._context_cleaned:
            return
        
        # ⚠️ 使用锁保护，避免并发清理
        if not hasattr(self, '_engine_lock'):
            return
            
        try:
            with self._engine_lock:
                # 再次检查，避免在获取锁期间被其他线程清理
                if self._context_cleaned:
                    return
                
                if getattr(self, 'use_tensorrt', False):
                    # Release TensorRT resources prior to tearing down CUDA context
                    for collection_name in ('inputs', 'outputs'):
                        buffers = getattr(self, collection_name, [])
                        for buf in buffers:
                            device_mem = buf.get('device')
                            try:
                                if device_mem is not None:
                                    device_mem.free()
                            except Exception:
                                pass
                    self.inputs = []
                    self.outputs = []
                    self.bindings = {}

                    stream = getattr(self, 'stream', None)
                    if stream is not None:
                        try:
                            stream.done = True  # hint for GC
                        except Exception:
                            pass
                        self.stream = None

                    for attr in ('trt_context', 'trt_engine'):
                        obj = getattr(self, attr, None)
                        if obj is not None:
                            try:
                                del obj
                            except Exception:
                                pass
                            setattr(self, attr, None)

                    # ⚠️ 关键修复：安全地pop CUDA context
                    cuda_ctx = getattr(self, 'cuda_context', None)
                    if cuda_ctx is not None:
                        try:
                            # 检查context是否仍然有效
                            if hasattr(cuda_ctx, 'get_device'):
                                # 尝试获取设备信息来验证context是否有效
                                try:
                                    cuda_ctx.get_device()
                                except:
                                    # Context已经无效，直接清理引用
                                    self.cuda_context = None
                                    self._context_cleaned = True
                                    return
                            
                            # 安全地pop context（确保栈平衡）
                            # 注意：retain_primary_context()创建的context只需要pop一次
                            if self._using_primary_cuda_context:
                                # Primary context: 只需要pop一次
                                try:
                                    cuda_ctx.pop()
                                except RuntimeError as e:
                                    # Context可能已经被pop或无效，忽略错误
                                    if "not on top" not in str(e).lower():
                                        self._debug_log(f"Context pop warning: {e}")
                                try:
                                    cuda_ctx.detach()
                                except Exception:
                                    pass
                            else:
                                # Standalone context: pop并detach
                                try:
                                    cuda_ctx.pop()
                                except RuntimeError as e:
                                    if "not on top" not in str(e).lower():
                                        self._debug_log(f"Context pop warning: {e}")
                                try:
                                    cuda_ctx.detach()
                                except Exception:
                                    pass
                        except Exception as e:
                            # 清理过程中的任何异常都不应该阻止程序退出
                            self._debug_log(f"Context cleanup exception (可忽略): {e}")
                        finally:
                            # 无论成功与否，都清除引用
                            self.cuda_context = None
                
                # ⚠️ 标记为已清理，防止重复清理
                self._context_cleaned = True
                self._debug_log("CUDA context cleaned up")
        except Exception as e:
            # 即使清理失败，也标记为已清理，避免重复尝试
            self._context_cleaned = True
            self._debug_log(f"Context cleanup failed (可忽略): {e}")
    
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config

    def _resolve_resource_paths(self):
        """Resolve relative resource paths inside the configuration."""
        if 'sam2' not in self.config:
            return

        resources = self.config['sam2']
        models_root = os.environ.get('L1_STAGE2_MODELS_DIR')
        for key in ('model_path', 'tensorrt_engine_path'):
            if key in resources and resources[key]:
                resources[key] = str(self._resolve_path(resources[key], models_root))

        if 'config_path' in resources and resources['config_path']:
            config_value = resources['config_path']
            if Path(config_value).suffix in {'.yaml', '.yml'}:
                resources['config_path'] = str(self._resolve_path(config_value))

    def _resolve_path(self, value: str, preferred_root: Optional[str] = None) -> Path:
        """Resolve resource path with multiple fallbacks."""
        path = Path(value)
        if path.is_absolute() and path.exists():
            return path

        candidates = []

        if preferred_root:
            candidates.append(Path(preferred_root) / path)

        if self.package_share_dir:
            candidates.append(self.package_share_dir / path)

        module_root = Path(__file__).resolve()
        for parent in module_root.parents:
            candidates.append(parent / path)

        candidates.append(Path.cwd() / path)

        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()

        resolved = path.resolve()
        if not resolved.exists():
            print(f"⚠ Resource not found for '{value}', attempted paths: {[str(c) for c in candidates]}")
        return resolved
    
    def _init_pytorch(self):
        """Initialize PyTorch SAM2 model."""
        try:
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            
            self.sam2_model = self._build_sam2_model()
            self.predictor = SAM2ImagePredictor(self.sam2_model)
        except ImportError as e:
            raise ImportError(
                "SAM2 library not found. Please install: pip install sam2"
            ) from e
    
    def _init_tensorrt(self):
        """Initialize TensorRT engine for SAM2."""
        engine_path = self.config['sam2']['tensorrt_engine_path']
        
        # Convert relative path to absolute
        if not os.path.isabs(engine_path):
            engine_path = os.path.join(os.getcwd(), engine_path)
        
        if not os.path.exists(engine_path):
            print(f"")
            print(f"{'='*60}")
            print(f"⚠ TensorRT engine not found!")
            print(f"{'='*60}")
            print(f"Expected location: {engine_path}")
            print(f"")
            print(f"To create TensorRT engine (recommended for speed):")
            print(f"  bash scripts/deployment/convert_models_uv.sh")
            print(f"")
            print(f"Falling back to PyTorch mode (slower but functional)")
            print(f"{'='*60}")
            print(f"")
            self.use_tensorrt = False
            self._init_pytorch()
            return
        
        # Initialize PyTorch model for decoder FIRST (lightweight)
        self.sam2_model = self._build_sam2_model()
        print(f"✓ SAM2 decoder loaded successfully")
        
        # Create predictor for mask decoding
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        self.predictor = SAM2ImagePredictor(self.sam2_model)

        # Initialize CUDA driver via PyCUDA
        try:
            cuda.init()
        except Exception as exc:
            print("⚠ Unable to initialize CUDA driver for TensorRT:", exc)
            print("  Falling back to PyTorch mode (TensorRT disabled)")
            self.use_tensorrt = False
            self._init_pytorch()
            return

        if cuda.Device.count() == 0:
            print("⚠ No CUDA devices detected; disabling TensorRT acceleration")
            self.use_tensorrt = False
            self._init_pytorch()
            return

        # Select device 0 by default
        self.cuda_device_id = 0

        if self._torch_has_cuda:
            try:
                device_id = torch.cuda.current_device()
            except Exception:
                device_id = 0
            self.cuda_device_id = device_id
            # Attach to PyTorch's primary context to share resources
            self.cuda_context = cuda.Device(device_id).retain_primary_context()
            self.cuda_context.push()
            self._using_primary_cuda_context = True
            print(f"✓ pycuda attached to PyTorch CUDA context (device {device_id})")
        else:
            # Create dedicated context when PyTorch lacks CUDA
            self.cuda_context = cuda.Device(self.cuda_device_id).make_context()
            self._using_primary_cuda_context = False
            print(f"✓ pycuda created standalone CUDA context on device {self.cuda_device_id}")
        
        # Load TensorRT engine
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.trt_engine = trt.Runtime(self.trt_logger).deserialize_cuda_engine(f.read())
        
        self.trt_context = self.trt_engine.create_execution_context()
        
        # Allocate buffers
        self._allocate_buffers()
    
    def _allocate_buffers(self):
        """Allocate GPU buffers for TensorRT (TensorRT 10+ API)."""
        self.inputs = []
        self.outputs = []
        self.bindings = {}
        self.stream = cuda.Stream()
        
        # Use TensorRT 10+ API
        num_io_tensors = self.trt_engine.num_io_tensors
        for i in range(num_io_tensors):
            name = self.trt_engine.get_tensor_name(i)
            shape = self.trt_engine.get_tensor_shape(name)
            dtype = trt.nptype(self.trt_engine.get_tensor_dtype(name))
            mode = self.trt_engine.get_tensor_mode(name)
            
            # Calculate size
            size = trt.volume(shape)
            
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            self.bindings[name] = int(device_mem)
            
            if mode == trt.TensorIOMode.INPUT:
                self.inputs.append({'host': host_mem, 'device': device_mem, 'name': name})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem, 'name': name})
    
    def encode_image(self, image: np.ndarray, force_encode: bool = False) -> np.ndarray:
        """
        Encode image to feature space. Uses caching to avoid re-encoding.
        
        Args:
            image: Input image (H, W, 3) in RGB format
            force_encode: Force re-encoding even if cached
            
        Returns:
            Image features (can be cached and reused)
        """
        start_time = time.time()
        with self._engine_lock:
            # Check cache
            image_hash = hash(image.tobytes())
            cache_valid = (
                not force_encode and
                self.cached_features is not None and
                self.cached_image_hash == image_hash and
                self.config['sam2']['cache_features']
            )
            
            if cache_valid:
                # Check cache timeout
                if self.cache_timestamp is not None:
                    age = time.time() - self.cache_timestamp
                    if age > self.config['sam2']['cache_timeout']:
                        cache_valid = False
            
            if cache_valid:
                return self.cached_features
            
            # Encode image
            if self.use_tensorrt:
                # TensorRT returns a list of all output features
                trt_outputs = self._encode_with_tensorrt(image)
                # Set the multi-output features in predictor for TensorRT mode
                self._set_predictor_features(image, trt_outputs)
                # For caching, use the main feature (image_embed)
                features = trt_outputs[0]
            else:
                features = self._encode_with_pytorch(image)
            
            # Update cache
            self.cached_features = features
            self.cached_image_hash = image_hash
            self.cache_timestamp = time.time()
        
        encode_time = time.time() - start_time
        self.encode_times.append(encode_time)
        
        # ⚠️ 编码后清理PyTorch缓存，为其他模块（如GraspModule）释放内存
        # SAM2使用TensorRT时，清理PyTorch缓存不会影响TensorRT的内存
        if self._torch_has_cuda:
            try:
                torch.cuda.empty_cache()
            except Exception as e:
                # 清理失败不影响功能，只记录警告
                self._debug_log(f"编码后清理缓存失败（可忽略）: {e}")
        
        print(f"Image encoding time: {encode_time*1000:.2f} ms")
        return features
    
    def _encode_with_pytorch(self, image: np.ndarray) -> np.ndarray:
        """Encode image using PyTorch."""
        with torch.no_grad():
            self.predictor.set_image(image)
            # Features are stored internally in predictor
            return self.predictor.get_image_embedding()
    
    def _set_predictor_features(self, image: np.ndarray, trt_outputs_list: List[np.ndarray]):
        """
        Set image features in predictor using TensorRT multi-output.
        
        Args:
            image: Original input image (H, W, 3)
            trt_outputs_list: List of output tensors from TensorRT
                              [image_embed, high_res_feat_0, high_res_feat_1, ...]
        """
        with torch.no_grad():
            # Convert all TensorRT outputs (NumPy arrays) to torch tensors
            features_tensors = [
                torch.from_numpy(f).to(self.device) 
                for f in trt_outputs_list
            ]
            
            # Split into image_embed and high_res_feats
            # Order matches Sam2EncoderWrapper output: image_embed first, then high_res_feats
            image_embed_tensor = features_tensors[0]
            high_res_feats_tensors = features_tensors[1:]  # All remaining are high_res_feats
            
            # Set predictor internal state with REAL features from TensorRT
            self.predictor._features = {
                "image_embed": image_embed_tensor,
                "high_res_feats": high_res_feats_tensors  # Use real features!
            }
            self.predictor._orig_hw = [(image.shape[0], image.shape[1])]
            self.predictor._is_image_set = True
            self.predictor._is_batch = False
            
            print(f"✓ Predictor features set with REAL TensorRT outputs")
            print(f"  Image size: {image.shape[0]}x{image.shape[1]}")
            print(f"  image_embed shape: {image_embed_tensor.shape}")
            print(f"  high_res_feats:")
            for i, feat in enumerate(high_res_feats_tensors):
                print(f"    [{i}] shape: {feat.shape}")
    
    def _encode_with_tensorrt(self, image: np.ndarray) -> List[np.ndarray]:
        """
        Encode image using TensorRT.
        Returns list of feature tensors: [image_embed, high_res_feat_0, high_res_feat_1, ...]
        """
        # Preprocess image
        preprocessed = self._preprocess_image(image)
        
        # Copy input to GPU
        np.copyto(self.inputs[0]['host'], preprocessed.ravel())
        cuda.memcpy_htod_async(
            self.inputs[0]['device'],
            self.inputs[0]['host'],
            self.stream
        )
        
        # Run inference (TensorRT 10+ API)
        for inp in self.inputs:
            self.trt_context.set_tensor_address(inp['name'], inp['device'])
        for out in self.outputs:
            self.trt_context.set_tensor_address(out['name'], out['device'])
        self.trt_context.execute_async_v3(stream_handle=self.stream.handle)
        
        # Copy ALL outputs from GPU (multi-output support)
        output_features = []
        for output in self.outputs:
            cuda.memcpy_dtoh_async(
                output['host'],
                output['device'],
                self.stream
            )
        self.stream.synchronize()
        
        # Reshape all outputs (TensorRT 10+ API)
        for output in self.outputs:
            output_name = output['name']
            output_shape = self.trt_engine.get_tensor_shape(output_name)
            features = output['host'].reshape(output_shape)
            output_features.append(features)
        
        return output_features
    
    def decode_mask(
        self,
        point_coords: np.ndarray,
        point_labels: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Decode segmentation mask from point prompts using cached features.
        
        Args:
            point_coords: Point coordinates (N, 2) in (x, y) format
            point_labels: Point labels (N,) - 1 for foreground, 0 for background
            
        Returns:
            mask: Binary segmentation mask (H, W)
            confidence: Confidence score for the mask
        """
        start_time = time.time()
        with self._engine_lock:
            self._debug_log(f"decode_mask start (points={point_coords}, labels={point_labels})")
            if self.cached_features is None:
                raise RuntimeError("No cached features. Call encode_image first.")
            
            # Default labels if not provided
            if point_labels is None:
                point_labels = np.ones(len(point_coords), dtype=np.int32)
            
            # Ensure correct shape
            if point_coords.ndim == 1:
                point_coords = point_coords.reshape(1, 2)
            if point_labels.ndim == 0:
                point_labels = np.array([point_labels])
            
            # Decode mask
            self._debug_log("predictor.predict begin")
            masks, scores, _ = self.predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                multimask_output=self.config['sam2']['point_prompt']['multimask_output']
            )
            self._debug_log("predictor.predict finished")
            
            # Select best mask
            if len(masks) > 1:
                best_idx = np.argmax(scores)
                mask = masks[best_idx]
                confidence = scores[best_idx]
            else:
                mask = masks[0]
                confidence = scores[0]
            self._debug_log(f"predictor output ready (confidence={confidence})")

        # Post-process mask
        mask = self._post_process_mask(mask)
        self._debug_log("post_process completed")
        
        decode_time = time.time() - start_time
        self.decode_times.append(decode_time)
        
        # ⚠️ 解码后清理PyTorch缓存，为其他模块（如GraspModule）释放内存
        # SAM2使用TensorRT时，清理PyTorch缓存不会影响TensorRT的内存
        if self._torch_has_cuda:
            try:
                torch.cuda.empty_cache()
            except Exception as e:
                # 清理失败不影响功能，只记录警告
                self._debug_log(f"解码后清理缓存失败（可忽略）: {e}")

        print(f"Mask decoding time: {decode_time*1000:.2f} ms (confidence: {confidence:.3f})")
        self._debug_log(f"decode_mask done (time_ms={decode_time*1000:.2f}, confidence={confidence:.3f})")

        return mask, float(confidence)
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for model input."""
        # Resize if needed
        target_size = tuple(self.config['sam2']['image_size'])
        if image.shape[:2] != target_size:
            image = cv2.resize(image, target_size[::-1])
        
        # Normalize
        mean = np.array(self.config['sam2']['normalize_mean'])
        std = np.array(self.config['sam2']['normalize_std'])
        image = (image.astype(np.float32) / 255.0 - mean) / std
        
        # HWC to CHW
        image = np.transpose(image, (2, 0, 1))
        
        # Add batch dimension
        image = np.expand_dims(image, 0)
        
        return image.astype(np.float32)
    
    def _post_process_mask(self, mask: np.ndarray) -> np.ndarray:
        """Apply post-processing to mask."""
        post_config = self.config['sam2']['post_process']
        
        # Convert to uint8
        mask = (mask > 0.5).astype(np.uint8) * 255
        
        # Morphological operations
        if post_config['morphology']['enabled']:
            kernel_size = post_config['morphology']['kernel_size']
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            operation = post_config['morphology']['operation']
            
            if operation == 'close':
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            elif operation == 'open':
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            elif operation == 'dilate':
                mask = cv2.dilate(mask, kernel)
            elif operation == 'erode':
                mask = cv2.erode(mask, kernel)
        
        # Keep only largest connected component
        if post_config['largest_component']:
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            if num_labels > 1:
                # Find largest component (excluding background)
                largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
                mask = (labels == largest_label).astype(np.uint8) * 255
        
        # Smooth boundary
        if post_config['smooth_boundary']:
            kernel_size = post_config['smooth_kernel_size']
            mask = cv2.GaussianBlur(mask, (kernel_size, kernel_size), 0)
            mask = (mask > 127).astype(np.uint8) * 255
        
        return mask
    
    def get_performance_stats(self) -> Dict[str, float]:
        """Get performance statistics."""
        stats = {}
        
        if len(self.encode_times) > 0:
            stats['avg_encode_time'] = np.mean(self.encode_times) * 1000  # ms
            stats['max_encode_time'] = np.max(self.encode_times) * 1000
            stats['min_encode_time'] = np.min(self.encode_times) * 1000
        
        if len(self.decode_times) > 0:
            stats['avg_decode_time'] = np.mean(self.decode_times) * 1000  # ms
            stats['max_decode_time'] = np.max(self.decode_times) * 1000
            stats['min_decode_time'] = np.min(self.decode_times) * 1000
        
        stats['using_tensorrt'] = self.use_tensorrt
        stats['cache_enabled'] = self.config['sam2']['cache_features']
        
        return stats
    
    def clear_cache(self):
        """Clear feature cache."""
        with self._engine_lock:
            self.cached_features = None
            self.cached_image_hash = None
            self.cache_timestamp = None

    def _resolve_model_path(self, model_path: str) -> str:
        """Resolve a model checkpoint path to an absolute path."""
        if os.path.isabs(model_path):
            return model_path
        return os.path.abspath(os.path.join(os.getcwd(), model_path))

    def _debug_log(self, message: str) -> None:
        """Write internal debug messages to a temporary log."""
        try:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            with self._debug_path.open("a", encoding="utf-8") as dbg:
                dbg.write(f"[{timestamp}] {message}\n")
        except Exception:
            pass

    def _resolve_config_file(self, config_name: str) -> Path:
        """Resolve SAM2 config name to an on-disk YAML file."""
        try:
            import sam2  # noqa: F401
        except ImportError as exc:  # pragma: no cover - import failure handled upstream
            raise ImportError(
                "SAM2 library not found. Please install: pip install sam2"
            ) from exc

        config_root = Path(importlib.import_module("sam2").__file__).resolve().parent / "configs"
        rel_path_str = config_name.strip().lstrip("./")
        if not rel_path_str.endswith((".yaml", ".yml")):
            rel_path_str = f"{rel_path_str}.yaml"
        config_file = config_root / Path(rel_path_str)
        if not config_file.is_file():
            raise FileNotFoundError(
                f"SAM2 config file not found: {config_file}. "
                "Verify that the sam2 package is installed with configuration assets."
            )
        return config_file

    def _build_sam2_model(self):
        """Construct a SAM2 model by explicitly loading YAML configuration."""
        model_config_path = self.config['sam2']['config_path']
        checkpoint_path = self._resolve_model_path(self.config['sam2']['model_path'])

        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(
                f"SAM2 checkpoint not found at {checkpoint_path}. "
                "Run the model download script or adjust sam2.model_path."
            )

        config_file = self._resolve_config_file(model_config_path)
        with open(config_file, "r", encoding="utf-8") as cfg_file:
            raw_cfg = yaml.safe_load(cfg_file)

        if "model" not in raw_cfg:
            raise KeyError(f"Invalid SAM2 config: 'model' section missing in {config_file}")

        model_cfg = copy.deepcopy(raw_cfg["model"])
        self._apply_default_decoder_overrides(model_cfg)

        sam2_model = self._instantiate_config(model_cfg)
        self._load_sam2_checkpoint(sam2_model, checkpoint_path)
        sam2_model = sam2_model.to(self.device)
        sam2_model.eval()
        return sam2_model

    @staticmethod
    def _apply_default_decoder_overrides(model_cfg: Dict[str, Any]) -> None:
        """Add stability overrides that Hydra applied in the original builder."""
        decoder_args = model_cfg.setdefault("sam_mask_decoder_extra_args", {})
        decoder_args.setdefault("dynamic_multimask_via_stability", True)
        decoder_args.setdefault("dynamic_multimask_stability_delta", 0.05)
        decoder_args.setdefault("dynamic_multimask_stability_thresh", 0.98)

    def _instantiate_config(self, cfg: Any) -> Any:
        """
        Recursively instantiate SAM2 modules described via `_target_` dictionaries.
        """
        if isinstance(cfg, dict):
            cfg_copy = {
                key: (value if isinstance(value, (dict, list)) else self._coerce_scalar(value))
                for key, value in cfg.items()
            }
            target_path = cfg_copy.pop("_target_", None)
            resolved_kwargs = {key: self._instantiate_config(value) for key, value in cfg_copy.items()}
            if target_path is None:
                return resolved_kwargs
            target_cls = self._import_from_string(target_path)
            return target_cls(**resolved_kwargs)
        if isinstance(cfg, list):
            return [self._instantiate_config(item) for item in cfg]
        return cfg

    @staticmethod
    def _import_from_string(target_path: str):
        """Import a symbol given its fully-qualified dotted path."""
        try:
            module_path, attr_name = target_path.rsplit(".", 1)
        except ValueError as exc:
            raise ImportError(f"Invalid target specification: '{target_path}'") from exc
        module = importlib.import_module(module_path)
        try:
            return getattr(module, attr_name)
        except AttributeError as exc:
            raise ImportError(
                f"Unable to locate attribute '{attr_name}' in module '{module_path}'"
            ) from exc

    @staticmethod
    def _coerce_scalar(value: Any) -> Any:
        """Convert scalar strings to native Python numeric/boolean types when possible."""
        if not isinstance(value, str):
            return value
        text = value.strip()
        if text.lower() in {"null", "none"}:
            return None
        if text.lower() == "true":
            return True
        if text.lower() == "false":
            return False
        try:
            # literal_eval handles ints, floats (including scientific notation)
            return ast.literal_eval(text)
        except (ValueError, SyntaxError):
            return value

    @staticmethod
    def _load_sam2_checkpoint(model, ckpt_path: str) -> None:
        """Load checkpoint weights into a SAM2 model."""
        try:
            checkpoint = torch.load(ckpt_path, map_location="cpu", weights_only=True)
        except TypeError:
            checkpoint = torch.load(ckpt_path, map_location="cpu")

        if isinstance(checkpoint, dict) and "model" in checkpoint:
            state_dict = checkpoint["model"]
        else:
            state_dict = checkpoint

        incompat = model.load_state_dict(state_dict, strict=False)
        missing_keys = getattr(incompat, "missing_keys", [])
        unexpected_keys = getattr(incompat, "unexpected_keys", [])

        if missing_keys or unexpected_keys:
            raise RuntimeError(
                f"Checkpoint load mismatch. Missing keys: {missing_keys}, "
                f"Unexpected keys: {unexpected_keys}"
            )


if __name__ == "__main__":
    # Test code
    config_path = "/home/jetson/L1/configs/algorithms/sam2_config.yaml"
    engine = SAM2Engine(config_path)
    
    # Create dummy image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Encode
    features = engine.encode_image(test_image)
    print(f"Encoded features shape: {features.shape}")
    
    # Decode with point
    point = np.array([[320, 240]])  # Center point
    mask, confidence = engine.decode_mask(point)
    print(f"Mask shape: {mask.shape}, Confidence: {confidence}")
    
    # Print stats
    print("\nPerformance Stats:")
    for key, value in engine.get_performance_stats().items():
        print(f"  {key}: {value}")
