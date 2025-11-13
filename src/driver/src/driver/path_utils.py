"""Shared path helpers for the robot_driver package."""
from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional

try:
    from ament_index_python.packages import (  # type: ignore
        get_package_share_directory,
        PackageNotFoundError,
    )
except ImportError:  # pragma: no cover
    get_package_share_directory = None  # type: ignore
    PackageNotFoundError = RuntimeError  # type: ignore

PACKAGE_NAME = 'robot_driver'


def _candidate_roots(extra: Optional[Iterable[Path]] = None) -> list[Path]:
    roots: list[Path] = []
    if extra:
        roots.extend(Path(p) for p in extra)

    if get_package_share_directory:
        try:
            share = Path(get_package_share_directory(PACKAGE_NAME))
            roots.append(share)
        except PackageNotFoundError:
            pass

    roots.append(Path(__file__).resolve().parents[2])  # package source root
    roots.append(Path.cwd())
    return roots


def resolve_relative_path(
    path_str: str,
    *,
    must_exist: bool = False,
    search_roots: Optional[Iterable[Path]] = None,
) -> Path:
    """Resolve ``path_str`` to an absolute :class:`Path`.

    - Absolute paths are returned as-is (optionally validated).
    - Relative paths are resolved against candidate roots in order.
    - If ``must_exist`` is True, the first existing candidate is returned; otherwise the
      first candidate path is returned even if it does not exist yet.
    """

    candidate = Path(path_str)
    if candidate.is_absolute():
        if must_exist and not candidate.exists():
            raise FileNotFoundError(candidate)
        return candidate

    for root in _candidate_roots(search_roots):
        candidate = (root / path_str).resolve()
        if not must_exist or candidate.exists():
            return candidate

    # Nothing existed; default to first root even if missing
    fallback_roots = _candidate_roots(search_roots)
    if not fallback_roots:
        raise FileNotFoundError(path_str)
    return (fallback_roots[0] / path_str).resolve()
