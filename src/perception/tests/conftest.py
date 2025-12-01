import pytest

from perception_core.async_worker import AsyncWorker


@pytest.fixture(scope="module")
def async_worker():
    worker = AsyncWorker(name="test_async_worker")
    yield worker
    worker.shutdown()
