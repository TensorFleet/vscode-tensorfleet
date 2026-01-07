"""
Shared utilities for robotic control scripts.

Common functions used across robotic tutorial scripts to reduce duplication
and keep individual tutorials focused on one concept.
"""

import json
import threading
import time
from typing import Any, Callable, Dict, List, Optional

try:
    import roslibpy
except ImportError:
    roslibpy = None  # type: ignore

from .tensorfleet_config import get_tensorfleet_settings
from .proxy_ws_client import create_proxy_websocket


def sleep(seconds: float) -> None:
    """Sleep for the specified number of seconds."""
    time.sleep(seconds)


def wait_for(
    check_fn: Callable[[], bool],
    label: str,
    timeout_seconds: float = 10.0,
    interval_seconds: float = 0.1,
) -> bool:
    """
    Wait for a condition to become true.

    Args:
        check_fn: Function that returns True when condition is met
        label: Description of what we're waiting for (for error messages)
        timeout_seconds: Maximum time to wait
        interval_seconds: How often to check the condition

    Returns:
        True if condition was met

    Raises:
        TimeoutError: If condition not met within timeout
    """
    start = time.time()
    while time.time() - start < timeout_seconds:
        if check_fn():
            return True
        time.sleep(interval_seconds)
    raise TimeoutError(f"Timeout waiting for {label}")


class ProxyRosClient:
    """
    A roslibpy-compatible ROS client that works through the VM Manager proxy.

    This class provides the same interface as roslibpy.Ros but routes all
    traffic through the TensorFleet VM Manager WebSocket proxy.
    """

    def __init__(self, proxy_ws):
        """
        Initialize the proxy ROS client.

        Args:
            proxy_ws: Connected ProxyWebSocketClient instance
        """
        self._proxy = proxy_ws
        self._is_connected = True
        self._callbacks: Dict[str, List[Callable]] = {}
        self._id_counter = 0
        self._lock = threading.Lock()
        self._pending_calls: Dict[str, Callable] = {}

        # Wire up message handling
        self._proxy.on_message = self._handle_message
        self._proxy.on_close = self._handle_close
        self._proxy.on_error = self._handle_error

    def _handle_message(self, data: str) -> None:
        """Handle incoming rosbridge message."""
        try:
            msg = json.loads(data)
            op = msg.get("op")


            if op == "publish":
                topic = msg.get("topic")
                if topic and topic in self._callbacks:
                    for callback in self._callbacks[topic]:
                        try:
                            callback(msg.get("msg", {}))
                        except Exception as e:
                            print(f"[ProxyRos] Callback error: {e}")
                else:
                    pass

            elif op == "service_response":
                # Handle service response
                call_id = msg.get("id")
                if call_id and call_id in self._pending_calls:
                    callback = self._pending_calls.pop(call_id)
                    callback(msg.get("values", {}))

        except json.JSONDecodeError:
            pass

    def _handle_close(self, code: int, reason: str) -> None:
        """Handle connection close."""
        self._is_connected = False

    def _handle_error(self, error: Exception) -> None:
        """Handle connection error."""
        self._is_connected = False

    def _next_id(self) -> str:
        """Generate unique message ID."""
        with self._lock:
            self._id_counter += 1
            return f"call_{self._id_counter}"

    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._is_connected and self._proxy.is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        """Set connection status."""
        self._is_connected = value

    def send_on_ready(self, message: dict, callback: Optional[Callable] = None) -> None:
        """Send a message when the connection is ready."""
        self._send(message)
        if callback:
            callback()

    def _send(self, message: dict) -> None:
        """Send a rosbridge message."""
        self._proxy.send(json.dumps(message))

    def advertise(self, topic: str, msg_type: str) -> None:
        """Advertise a topic."""
        self._send({
            "op": "advertise",
            "topic": topic,
            "type": msg_type
        })

    def unadvertise(self, topic: str) -> None:
        """Unadvertise a topic."""
        self._send({
            "op": "unadvertise",
            "topic": topic
        })

    def publish(self, topic: str, message: dict) -> None:
        """Publish a message to a topic."""
        self._send({
            "op": "publish",
            "topic": topic,
            "msg": message
        })

    def subscribe(self, topic: str, msg_type: str, callback: Callable) -> None:
        """Subscribe to a topic."""
        if topic not in self._callbacks:
            self._callbacks[topic] = []
            self._send({
                "op": "subscribe",
                "topic": topic,
                "type": msg_type
            })
        self._callbacks[topic].append(callback)

    def unsubscribe(self, topic: str) -> None:
        """Unsubscribe from a topic."""
        if topic in self._callbacks:
            del self._callbacks[topic]
            self._send({
                "op": "unsubscribe",
                "topic": topic
            })

    def call_service(self, service: str, service_type: str, request: dict, callback: Callable) -> None:
        """Call a service."""
        call_id = self._next_id()
        self._pending_calls[call_id] = callback
        self._send({
            "op": "call_service",
            "id": call_id,
            "service": service,
            "type": service_type,
            "args": request
        })

    def terminate(self) -> None:
        """Close the connection."""
        self._is_connected = False
        self._proxy.close()

    def close(self) -> None:
        """Alias for terminate."""
        self.terminate()

    def run(self, timeout: float = 10.0) -> None:
        """No-op for compatibility - proxy is already connected."""
        pass


class ProxyTopic:
    """
    A roslibpy.Topic-compatible wrapper for ProxyRosClient.
    """

    def __init__(self, ros: ProxyRosClient, name: str, message_type: str, **kwargs):
        self.ros = ros
        self.name = name
        self.message_type = message_type
        self._is_advertised = False
        self._subscribers: List[Callable] = []

    def advertise(self) -> None:
        """Advertise the topic."""
        if not self._is_advertised:
            self.ros.advertise(self.name, self.message_type)
            self._is_advertised = True

    def unadvertise(self) -> None:
        """Unadvertise the topic."""
        if self._is_advertised:
            self.ros.unadvertise(self.name)
            self._is_advertised = False

    def publish(self, message) -> None:
        """Publish a message."""
        if not self._is_advertised:
            self.advertise()
        msg_dict = message if isinstance(message, dict) else getattr(message, '_data', message)
        if hasattr(msg_dict, '__dict__'):
            msg_dict = msg_dict.__dict__
        self.ros.publish(self.name, msg_dict)

    def subscribe(self, callback: Callable) -> None:
        """Subscribe to the topic."""
        self._subscribers.append(callback)
        self.ros.subscribe(self.name, self.message_type, callback)

    def unsubscribe(self) -> None:
        """Unsubscribe from the topic."""
        self.ros.unsubscribe(self.name)
        self._subscribers.clear()


class ProxyService:
    """
    A roslibpy.Service-compatible wrapper for ProxyRosClient.
    """

    def __init__(self, ros: ProxyRosClient, name: str, service_type: str):
        self.ros = ros
        self.name = name
        self.service_type = service_type

    def call(self, request: dict, callback: Optional[Callable] = None, timeout: float = 5.0) -> Any:
        """Call the service."""
        result = [None]
        done = threading.Event()

        def on_response(response):
            result[0] = response
            done.set()

        self.ros.call_service(self.name, self.service_type, request, on_response)

        if callback:
            # Async mode
            def wait_and_callback():
                if done.wait(timeout):
                    callback(result[0])
            threading.Thread(target=wait_and_callback, daemon=True).start()
            return None
        else:
            # Sync mode
            if done.wait(timeout):
                return result[0]
            raise TimeoutError(f"Service call to {self.name} timed out")


def connect_to_robot(url: Optional[str] = None, timeout: float = 10.0):
    """
    Connect to rosbridge and return ROS client.

    If TensorFleet proxy settings are available (base/vm-manager URL + node ID + JWT),
    traffic is routed through the derived proxy socket. Otherwise, we connect directly
    to the resolved rosbridge URL.

    Args:
        url: Optional direct rosbridge URL (overrides config if proxy not available)
        timeout: Connection timeout in seconds

    Returns:
        Connected roslibpy.Ros client (or ProxyRosClient for proxy connections)

    Raises:
        ImportError: If roslibpy is not installed
        ConnectionError: If connection fails
        TimeoutError: If connection times out
    """
    if roslibpy is None:
        raise ImportError("roslibpy is required. Install it with: pip install roslibpy")

    settings = get_tensorfleet_settings()
    use_proxy = settings["use_proxy"] and settings["proxy_url"]

    if use_proxy:
        print(
            f"[CONNECT] Connecting via VM Manager proxy at {settings['proxy_url']} "
            f"(nodeId={settings['node_id']}, targetPort={settings['target_port']})..."
        )

        proxy_ws = create_proxy_websocket(
            proxy_url=settings["proxy_url"],
            vm_manager_url=settings["vm_manager_url"],
            token=settings["token"],
            node_id=settings["node_id"],
            target_port=settings["target_port"],
        )

        # Connect and authenticate
        proxy_ws.connect(timeout=timeout)

        # Create our proxy-aware ROS client
        ros = ProxyRosClient(proxy_ws)

        print("[CONNECT] Connected to rosbridge via proxy")
        return ros

    else:
        # Direct connection using roslibpy
        direct_url = settings["rosbridge_url"] or url
        if not direct_url:
            raise ValueError("No rosbridge URL available")

        # Parse URL for host and port
        from urllib.parse import urlparse
        parsed = urlparse(direct_url)
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 9091

        print(f"[CONNECT] Connecting to rosbridge at {direct_url}...")
        ros = roslibpy.Ros(host=host, port=port)

        # Connect with timeout
        ros.run(timeout=timeout)

        if not ros.is_connected:
            raise ConnectionError(f"Failed to connect to rosbridge at {direct_url}")

        print("[CONNECT] Connected to rosbridge")
        return ros


# Make ProxyTopic and ProxyService available for type checking but prefer roslibpy when available
def Topic(ros, name: str, message_type: str, **kwargs):
    """Create a Topic compatible with the ROS client type."""
    if isinstance(ros, ProxyRosClient):
        return ProxyTopic(ros, name, message_type, **kwargs)
    return roslibpy.Topic(ros, name, message_type, **kwargs)


def Service(ros, name: str, service_type: str):
    """Create a Service compatible with the ROS client type."""
    if isinstance(ros, ProxyRosClient):
        return ProxyService(ros, name, service_type)
    return roslibpy.Service(ros, name, service_type)
