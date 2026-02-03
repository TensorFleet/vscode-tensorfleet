"""
TensorFleet configuration helper for robotic Python template.

Responsibility:
- Load connection-related settings from environment variables (dotenv),
  falling back to config/robot_config.yaml defaults.
- Decide whether to connect directly to rosbridge or via VM Manager proxy.

This file is copied into new workspaces by the VS Code extension.
"""

import json
import os
from pathlib import Path
from typing import Any, Optional

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenv is optional

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

from .url_utils import to_proxy_websocket_url


def _first_non_empty(*values: Optional[str]) -> str:
    """Return the first non-empty string value."""
    for value in values:
        if isinstance(value, str) and value.strip():
            return value
    return ""


def _load_yaml_config() -> dict[str, Any]:
    """Load robot_config.yaml from config directory."""
    config_path = Path.cwd() / "config" / "robot_config.yaml"
    try:
        contents = config_path.read_text(encoding="utf-8")
        if yaml:
            return yaml.safe_load(contents) or {}
        # Fallback: basic YAML parsing for simple key: value structures
        result: dict[str, Any] = {}
        for line in contents.split("\n"):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if ":" in line:
                key, _, value = line.partition(":")
                key = key.strip()
                value = value.strip().strip("\"'")
                if value:
                    result[key] = value
        return result
    except FileNotFoundError:
        return {}  # Config file is optional
    except Exception as err:
        print(f"[TF-CONFIG] Could not parse config at {config_path}, using defaults. {err}")
        return {}


def _load_tensorfleet_metadata() -> dict[str, Any]:
    """Load .tensorfleet metadata file."""
    marker_path = Path.cwd() / ".tensorfleet"
    try:
        raw = marker_path.read_text(encoding="utf-8").strip()
        if not raw:
            return {}
        return json.loads(raw)
    except FileNotFoundError:
        return {}
    except Exception as err:
        print(f"[TF-CONFIG] Could not parse .tensorfleet at {marker_path}: {err}")
        return {}


def _num_env(key: str, fallback: float) -> float:
    """Get numeric environment variable with fallback."""
    raw = os.environ.get(key)
    if raw is None:
        return fallback
    try:
        return float(raw)
    except ValueError:
        return fallback


def _load_config_sources() -> tuple[dict, dict, dict, dict]:
    """Load configuration sources (YAML config and .tensorfleet metadata)."""
    cfg = _load_yaml_config()
    marker = _load_tensorfleet_metadata()
    marker_env = marker.get("env", {}) or {}
    network = cfg.get("network", {}) or {}
    return cfg, marker, marker_env, network


def _resolve_base_url(marker_env: dict) -> str:
    """Resolve the base/VM manager URL from various sources."""
    return _first_non_empty(
        os.environ.get("TENSORFLEET_BASE_URL"),
        os.environ.get("TENSORFLEET_VM_MANAGER_URL"),
        marker_env.get("baseUrl"),
        marker_env.get("vmManagerUrl"),
    )


def _resolve_rosbridge_connection(marker_env: dict, network: dict) -> dict[str, Any]:
    """Resolve rosbridge connection details (host, port, URL)."""
    port_str = (
        os.environ.get("R2B_PORT")
        or os.environ.get("ROSBRIDGE_PORT")
        or marker_env.get("rosbridgePort")
        or network.get("rosbridge_port")
        or "9091"
    )
    port = int(port_str) if port_str else 9091

    host = (
        os.environ.get("R2B_HOST")
        or os.environ.get("ROS_HOST")
        or marker_env.get("r2bHost")
        or network.get("vm_ip")
        or ""
    )

    rosbridge_url = (
        os.environ.get("ROSBRIDGE_URL")
        or marker_env.get("rosbridgeUrl")
        or network.get("rosbridge_url")
        or (f"ws://{host}:{port}" if host else f"ws://127.0.0.1:{port}")
    )

    return {"host": host, "port": port, "rosbridge_url": rosbridge_url}


def _resolve_proxy_connection(base_url: str, marker_env: dict) -> dict[str, Any]:
    """Resolve proxy connection details."""
    vm_manager_url = (
        os.environ.get("TENSORFLEET_VM_MANAGER_URL")
        or marker_env.get("vmManagerUrl")
        or base_url
        or ""
    )

    node_id = os.environ.get("TENSORFLEET_NODE_ID") or marker_env.get("nodeId") or ""
    token = os.environ.get("TENSORFLEET_JWT") or ""

    explicit_proxy_url = os.environ.get("TENSORFLEET_PROXY_URL") or marker_env.get("proxyUrl") or ""
    proxy_url = (
        explicit_proxy_url
        or (to_proxy_websocket_url(vm_manager_url) if vm_manager_url else "")
        or (to_proxy_websocket_url(base_url) if base_url else "")
    )

    use_proxy = bool(proxy_url and node_id and token)

    return {
        "vm_manager_url": vm_manager_url,
        "node_id": node_id,
        "token": token,
        "proxy_url": proxy_url,
        "use_proxy": use_proxy,
    }


def get_tensorfleet_settings() -> dict[str, Any]:
    """
    Build resolved TensorFleet settings for this workspace.

    Precedence:
      1. Environment variables (from .env, tasks, or shell)
      2. config/robot_config.yaml values
      3. Hardcoded safe defaults

    Returns:
        Dictionary with connection settings including:
        - base_url, rosbridge_url, host, port
        - vm_manager_url, node_id, token, use_proxy, proxy_url
        - target_port
    """
    _cfg, _marker, marker_env, network = _load_config_sources()

    base_url = _resolve_base_url(marker_env)
    rb_conn = _resolve_rosbridge_connection(marker_env, network)
    proxy_conn = _resolve_proxy_connection(base_url, marker_env)

    fallback_host = rb_conn["host"] or marker_env.get("r2bHost") or "127.0.0.1"

    return {
        "base_url": proxy_conn["vm_manager_url"] or base_url or "",
        "rosbridge_url": rb_conn["rosbridge_url"],
        "host": fallback_host,
        "port": rb_conn["port"],
        "vm_manager_url": proxy_conn["vm_manager_url"],
        "node_id": proxy_conn["node_id"],
        "token": proxy_conn["token"],
        "use_proxy": proxy_conn["use_proxy"],
        "proxy_url": proxy_conn["proxy_url"],
        "target_port": int(
            _num_env(
                "ROSBRIDGE_PORT",
                marker_env.get("rosbridgePort") or network.get("rosbridge_port") or 9091,
            )
        ),
    }
