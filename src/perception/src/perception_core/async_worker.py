"""
AsyncWorker - 专用异步执行器

在独立的事件循环线程中调度任务，并使用线程池执行 CPU 密集型函数。
提供 await-able 句柄以兼容 rclpy 的协程回调环境。
"""

from __future__ import annotations

import asyncio
import functools
import threading
import time
from concurrent.futures import Future as ConcurrentFuture, ThreadPoolExecutor, TimeoutError
from typing import Any, Callable, Coroutine, Optional


class _WorkerAwaitable:
    """将 concurrent future 转换为可 await 的对象（基于轮询）。"""

    def __init__(self, future: ConcurrentFuture, poll_interval: float):
        self._future = future
        self._poll_interval = poll_interval

    def __await__(self):
        while True:
            try:
                return self._future.result(timeout=self._poll_interval)
            except TimeoutError:
                # 暂停一小段时间，避免忙轮询
                time.sleep(self._poll_interval)
                yield


class AsyncWorker:
    """
    统一的异步执行器

    - 在独立线程中运行 asyncio 事件循环
    - 使用 ThreadPoolExecutor 执行阻塞任务
    - 通过 await-able 句柄返回结果，兼容 rclpy executor 的协程模式
    """

    def __init__(
        self,
        name: str = "perception_async_worker",
        *,
        max_workers: Optional[int] = None,
        poll_interval: float = 0.001,
    ):
        self._name = name
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix=name)
        self._poll_interval = poll_interval
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._loop_ready = threading.Event()
        self._start_lock = threading.Lock()
        self._stopping = threading.Event()

    # ------------------------------------------------------------------
    # 生命周期管理
    # ------------------------------------------------------------------
    def start(self):
        """启动事件循环线程（幂等）。"""
        if self._thread and self._thread.is_alive():
            return

        with self._start_lock:
            if self._thread and self._thread.is_alive():
                return

            self._loop_ready.clear()
            self._thread = threading.Thread(
                target=self._run_loop, name=self._name, daemon=True
            )
            self._thread.start()

        self._loop_ready.wait()

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._loop_ready.set()

        try:
            loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop=loop)
            for task in pending:
                task.cancel()
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.close()

    def shutdown(self, wait: bool = True):
        """停止事件循环线程并回收线程池。"""
        if self._stopping.is_set():
            return

        self._stopping.set()

        if self._loop and self._loop.is_running():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except RuntimeError:
                pass

        if self._thread and wait:
            self._thread.join(timeout=5.0)

        self._executor.shutdown(wait=wait)
        self._loop = None
        self._thread = None

    # ------------------------------------------------------------------
    # 任务调度
    # ------------------------------------------------------------------
    def _ensure_running(self):
        if self._stopping.is_set():
            raise RuntimeError("AsyncWorker 已停止，无法调度新任务")
        if self._loop is None or not self._loop.is_running():
            self.start()

    def run_coroutine(self, coro: Coroutine[Any, Any, Any]) -> _WorkerAwaitable:
        """
        在工作线程的事件循环上执行协程。

        返回 await-able 句柄，可在 rclpy executor 的协程上下文中等待结果。
        """
        self._ensure_running()
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return _WorkerAwaitable(future, self._poll_interval)

    def run_callable(self, func: Callable[..., Any], *args, **kwargs) -> _WorkerAwaitable:
        """
        在线程池中执行阻塞函数。

        Args:
            func: 同步函数
            *args, **kwargs: 函数参数

        Returns:
            _WorkerAwaitable: 可 await 的句柄
        """
        async def _call():
            loop = asyncio.get_running_loop()
            bound = functools.partial(func, *args, **kwargs)
            return await loop.run_in_executor(self._executor, bound)

        return self.run_coroutine(_call())
