from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Sequence, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # type: ignore

try:
    import xacro  # type: ignore
except ImportError:  # pragma: no cover
    xacro = None  # type: ignore

DEFAULT_URDF_PATH = "urdf/yam.urdf"
DEFAULT_LOG_RELATIVE = "../../log/robot_desc_check.log"
PACKAGE_NAME = "description"


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        result = run_asset_check(
            urdf_path_arg=args.urdf_path,
            xacro_args=args.xacro_args,
            frame_prefix=args.frame_prefix,
            log_file_arg=args.log_file,
        )
    except Exception as exc:  # pragma: no cover - defensive
        _append_log(
            log_path=_resolve_log_path(args.log_file),
            payload={
                "timestamp": _now_iso(),
                "status": "error",
                "message": f"Unhandled exception during asset check: {exc}",
            },
        )
        print(f"[robot_desc_check] ERROR: {exc}", file=sys.stderr)
        return 1

    status = result["status"]
    msg = result["message"]
    if status == "ok":
        print(f"[robot_desc_check] {msg}")
        return 0
    print(f"[robot_desc_check] ERROR: {msg}", file=sys.stderr)
    return 1


def run_asset_check(
    *,
    urdf_path_arg: str,
    xacro_args: Sequence[str],
    frame_prefix: str,
    log_file_arg: str,
) -> Dict[str, object]:
    share_dir = _discover_share_dir()
    resolved_urdf = _resolve_urdf_path(urdf_path_arg, share_dir)
    urdf_text = _render_description(resolved_urdf, xacro_args)
    mesh_uris = _extract_mesh_uris(urdf_text)

    missing: List[Dict[str, str]] = []
    remote_meshes: List[str] = []
    resolved_meshes: List[str] = []
    for uri in mesh_uris:
        if uri.startswith(("http://", "https://")):
            remote_meshes.append(uri)
            continue
        try:
            mesh_path = _resolve_mesh_uri(uri, resolved_urdf.parent)
            if not mesh_path.exists():
                missing.append({"uri": uri, "resolved_path": str(mesh_path)})
            else:
                resolved_meshes.append(str(mesh_path))
        except Exception as exc:  # pragma: no cover - defensive
            missing.append({"uri": uri, "error": str(exc)})

    record = {
        "timestamp": _now_iso(),
        "status": "ok" if not missing else "warning",
        "message": _build_message(resolved_urdf, missing),
        "urdf": str(resolved_urdf),
        "mesh_total": len(mesh_uris),
        "mesh_resolved": len(resolved_meshes),
        "mesh_remote": len(remote_meshes),
        "frame_prefix": frame_prefix,
        "xacro_args": list(xacro_args),
        "missing_meshes": missing,
        "remote_meshes": remote_meshes,
    }
    _append_log(_resolve_log_path(log_file_arg), record)
    return record


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="robot_desc_check",
        description="Validate URDF/xacro and mesh assets for robot_desc_node.",
    )
    parser.add_argument(
        "--urdf-path",
        default=DEFAULT_URDF_PATH,
        help="URDF or xacro path (supports package://). Default: %(default)s",
    )
    parser.add_argument(
        "--xacro-arg",
        action="append",
        dest="xacro_args",
        default=[],
        help="Repeated KEY:=VALUE pairs forwarded to xacro",
    )
    parser.add_argument(
        "--frame-prefix",
        default="",
        help="Optional frame prefix recorded in the log payload",
    )
    parser.add_argument(
        "--log-file",
        default=DEFAULT_LOG_RELATIVE,
        help="Relative or absolute path for the asset check log file",
    )
    return parser.parse_args(argv)


def _discover_share_dir() -> Path:
    try:
        return Path(get_package_share_directory(PACKAGE_NAME))
    except (PackageNotFoundError, ValueError):  # pragma: no cover - fallback for devel trees
        return Path(__file__).resolve().parents[2]


def _resolve_urdf_path(raw_path: str, share_dir: Path) -> Path:
    if raw_path.startswith("package://"):
        pkg, rel = raw_path[len("package://") :].split("/", 1)
        base = _get_package_share(pkg)
        candidate = (base / rel).resolve()
    else:
        candidate = Path(raw_path)
        if not candidate.is_absolute():
            candidate = (share_dir / candidate).resolve()
    if not candidate.exists():
        raise FileNotFoundError(f"URDF/xacro not found: {candidate}")
    return candidate


def _render_description(path: Path, xacro_args: Sequence[str]) -> str:
    if path.suffix == ".xacro":
        if xacro is None:
            raise RuntimeError("xacro module not available but xacro file requested")
        mappings = _parse_xacro_args(xacro_args)
        doc = xacro.process_file(str(path), mappings=mappings)
        return doc.toxml()
    return path.read_text()


def _parse_xacro_args(args: Sequence[str]) -> Dict[str, str]:
    mappings: Dict[str, str] = {}
    for arg in args:
        if ":=" not in arg:
            continue
        key, value = arg.split(":=", 1)
        mappings[key] = value
    return mappings


def _extract_mesh_uris(urdf_text: str) -> List[str]:
    root = ET.fromstring(urdf_text)
    uris = {elem.attrib["filename"] for elem in root.iter("mesh") if "filename" in elem.attrib}
    return sorted(uris)


def _resolve_mesh_uri(uri: str, urdf_dir: Path) -> Path:
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://") :].split("/", 1)
        base = _get_package_share(pkg)
        return (base / rel).resolve()
    if uri.startswith("file://"):
        return Path(uri[len("file://") :]).resolve()
    candidate = Path(uri)
    if not candidate.is_absolute():
        candidate = (urdf_dir / candidate).resolve()
    return candidate


def _build_message(urdf_path: Path, missing: List[Dict[str, str]]) -> str:
    if not missing:
        return f"Asset check passed for {urdf_path}"
    return f"Asset check found {len(missing)} missing mesh file(s) for {urdf_path}"


def _append_log(log_path: Path, payload: Dict[str, object]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as log_file:
        json.dump(payload, log_file, ensure_ascii=False)
        log_file.write("\n")


def _resolve_log_path(path_arg: str) -> Path:
    path = Path(path_arg)
    if path.is_absolute():
        return path
    base = Path(__file__).resolve().parents[2]
    return (base / path).resolve()


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _get_package_share(pkg: str) -> Path:
    try:
        return Path(get_package_share_directory(pkg))
    except (PackageNotFoundError, ValueError):
        if pkg == PACKAGE_NAME:
            return Path(__file__).resolve().parents[2]
        raise


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
