"""
Shared URL utilities for TensorFleet robotic Python template.
"""

from urllib.parse import urlparse, urlunparse


def to_proxy_websocket_url(vm_manager_url: str) -> str:
    """
    Convert a VM Manager URL (http/https or ws/wss) to a proxy WebSocket URL.

    Args:
        vm_manager_url: The base VM Manager URL

    Returns:
        The derived proxy WebSocket URL, or empty string on failure
    """
    if not vm_manager_url:
        return ""

    try:
        parsed = urlparse(vm_manager_url)

        # Already a WebSocket URL - just ensure /ws path
        if parsed.scheme in ("ws", "wss"):
            path = parsed.path
            if not path or path == "/":
                path = "/ws"
            return urlunparse((parsed.scheme, parsed.netloc, path, "", "", ""))

        # Convert HTTP(S) to WS(S)
        protocol = "wss" if parsed.scheme == "https" else "ws"
        base_path = parsed.path.rstrip("/")
        path = base_path if base_path.endswith("/ws") else f"{base_path}/ws"

        return urlunparse((protocol, parsed.netloc, path, "", "", ""))
    except Exception:
        return ""
