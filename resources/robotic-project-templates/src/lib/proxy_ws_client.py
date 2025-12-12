"""
Minimal WebSocket proxy client for Python scripts.

Connects to the TensorFleet VM Manager WebSocket proxy and performs a
login handshake, then exposes a transport compatible with roslibpy.
"""

import json
import threading
from typing import Any, Callable, Optional

try:
    import websocket  # websocket-client library
except ImportError:
    websocket = None  # type: ignore


class ProxyWebSocketClient:
    """
    A WebSocket client that routes traffic through the VM Manager proxy.

    This class wraps the websocket-client library and handles:
    - Connection to the proxy WebSocket endpoint
    - Login handshake with token, nodeId, and targetPort
    - Message forwarding after successful authentication

    Compatible with roslibpy's RosBridgeClient transport.
    """

    def __init__(
        self,
        proxy_url: str,
        token: str,
        node_id: str,
        target_port: int,
        vm_manager_url: str = "",
    ):
        """
        Initialize the proxy WebSocket client.

        Args:
            proxy_url: WebSocket URL of the VM Manager proxy (ws(s)://.../ws)
            token: JWT token for authentication
            node_id: Target VM/node identifier
            target_port: Port inside VM (e.g., 9091 for rosbridge)
            vm_manager_url: Optional base URL for deriving proxy_url
        """
        if websocket is None:
            raise ImportError(
                "websocket-client is required for proxy connections. "
                "Install it with: pip install websocket-client"
            )

        self.proxy_url = proxy_url
        self.token = token
        self.node_id = node_id
        self.target_port = target_port
        self.vm_manager_url = vm_manager_url

        self._ws: Optional[websocket.WebSocketApp] = None
        self._state = "disconnected"  # disconnected, connecting, authenticating, connected, error, closed
        self._queue: list[str] = []
        self._lock = threading.Lock()
        self._connected_event = threading.Event()
        self._error: Optional[Exception] = None

        # Callbacks for roslibpy compatibility
        self.on_open: Optional[Callable[[], None]] = None
        self.on_message: Optional[Callable[[str], None]] = None
        self.on_close: Optional[Callable[[int, str], None]] = None
        self.on_error: Optional[Callable[[Exception], None]] = None

    def connect(self, timeout: float = 10.0) -> None:
        """
        Connect to the proxy and perform the login handshake.

        Args:
            timeout: Maximum time to wait for connection in seconds

        Raises:
            ConnectionError: If connection or authentication fails
            TimeoutError: If connection times out
        """
        self._state = "connecting"
        self._connected_event.clear()
        self._error = None

        # Create WebSocket connection
        self._ws = websocket.WebSocketApp(
            self.proxy_url,
            on_open=self._on_ws_open,
            on_message=self._on_ws_message,
            on_close=self._on_ws_close,
            on_error=self._on_ws_error,
        )

        # Run in background thread
        self._ws_thread = threading.Thread(target=self._ws.run_forever, daemon=True)
        self._ws_thread.start()

        # Wait for connection or error
        if not self._connected_event.wait(timeout):
            self._state = "error"
            raise TimeoutError(f"Connection to {self.proxy_url} timed out after {timeout}s")

        if self._error:
            raise self._error

        if self._state != "connected":
            raise ConnectionError(f"Failed to connect to proxy: state={self._state}")

    def send(self, data: str) -> None:
        """Send a message through the proxy."""
        with self._lock:
            if self._state != "connected" or self._ws is None:
                self._queue.append(data)
                return
            self._ws.send(data)

    def close(self, code: int = 1000, reason: str = "") -> None:
        """Close the WebSocket connection."""
        self._state = "closed"
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass

    @property
    def is_connected(self) -> bool:
        """Return True if connected and authenticated."""
        return self._state == "connected"

    def _on_ws_open(self, ws: Any) -> None:
        """Handle WebSocket connection opened."""
        self._state = "authenticating"

        # Send login message
        login_msg = {
            "type": "login",
            "token": self.token,
            "nodeId": self.node_id,
            "targetPort": self.target_port,
        }
        try:
            ws.send(json.dumps(login_msg))
        except Exception as err:
            self._state = "error"
            self._error = ConnectionError(f"Failed to send login message: {err}")
            self._connected_event.set()

    def _on_ws_message(self, ws: Any, data: str) -> None:
        """Handle incoming WebSocket message."""
        if self._state == "authenticating":
            # Parse login response
            try:
                msg = json.loads(data)
                if msg.get("type") == "loginResponse":
                    if msg.get("success"):
                        self._state = "connected"
                        self._connected_event.set()
                        self._flush_queue()
                        if self.on_open:
                            self.on_open()
                    else:
                        self._state = "error"
                        self._error = ConnectionError(
                            msg.get("message", "VM Manager proxy login failed")
                        )
                        self._connected_event.set()
                    return
            except json.JSONDecodeError:
                pass

            # Unexpected message during auth
            self._state = "error"
            self._error = ConnectionError(
                "Unexpected message while authenticating with VM Manager proxy"
            )
            self._connected_event.set()
            return

        # Forward normal messages
        if self.on_message:
            self.on_message(data)

    def _on_ws_close(self, ws: Any, close_status_code: int, close_msg: str) -> None:
        """Handle WebSocket connection closed."""
        self._state = "closed"
        self._connected_event.set()  # Unblock any waiting connect()
        if self.on_close:
            self.on_close(close_status_code or 1000, close_msg or "")

    def _on_ws_error(self, ws: Any, error: Exception) -> None:
        """Handle WebSocket error."""
        self._state = "error"
        self._error = error
        self._connected_event.set()
        if self.on_error:
            self.on_error(error)

    def _flush_queue(self) -> None:
        """Send any queued messages after connection is established."""
        with self._lock:
            while self._queue and self._ws and self._state == "connected":
                msg = self._queue.pop(0)
                self._ws.send(msg)


def create_proxy_websocket(
    proxy_url: str = "",
    token: str = "",
    node_id: str = "",
    target_port: int = 9091,
    vm_manager_url: str = "",
) -> ProxyWebSocketClient:
    """
    Create a proxy WebSocket client for routing traffic through VM Manager.

    Args:
        proxy_url: WebSocket URL of the proxy (derived from vm_manager_url if empty)
        token: JWT token for authentication
        node_id: Target VM/node identifier
        target_port: Port inside VM (default: 9091 for rosbridge)
        vm_manager_url: Optional base URL for deriving proxy_url

    Returns:
        ProxyWebSocketClient instance
    """
    from .url_utils import to_proxy_websocket_url

    resolved_proxy_url = proxy_url or to_proxy_websocket_url(vm_manager_url)
    if not resolved_proxy_url:
        raise ValueError("Missing proxy_url for VM Manager WebSocket proxy")

    return ProxyWebSocketClient(
        proxy_url=resolved_proxy_url,
        token=token,
        node_id=node_id,
        target_port=target_port,
        vm_manager_url=vm_manager_url,
    )
