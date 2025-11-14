#!/usr/bin/env python3
"""
Simple client for the rosbridge WebSocket server running inside the VM.

This is a trimmed‑down version of the working example you provided, focused on:

  - Connectivity test via /rosapi/topics
  - Sending control commands to ROS 2 topics
  - Subscribing to a raw image topic, running YOLO on CPU, and
    republishing an annotated image back into ROS

Dependencies (on the host):
  pip install roslibpy numpy
  # For YOLO image mode:
  pip install ultralytics opencv-python
"""

import argparse
import base64
import sys
import time
import threading

try:
    import roslibpy  # type: ignore[import]
except ImportError:  # pragma: no cover - import-time check
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it on the host with:", file=sys.stderr)
    print("  pip install roslibpy", file=sys.stderr)
    sys.exit(1)

try:
    import numpy as np  # type: ignore[import]
except ImportError:  # pragma: no cover - optional
    np = None  # Image/Yolo mode will check for this explicitly.

try:
    from ultralytics import YOLO  # type: ignore[import]
except ImportError:  # pragma: no cover - optional
    YOLO = None  # YOLO mode will check for this explicitly.

try:
    import cv2  # type: ignore[import]
except ImportError:  # pragma: no cover - optional
    cv2 = None  # Compressed image mode will check for this explicitly.


YOLO_MODEL = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Vision client: subscribe to images over rosbridge, run YOLO, and republish.",
    )
    parser.add_argument(
        "--host",
        default="172.16.0.10",
        help="ROS bridge host (default: 172.16.0.10 inside VM)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9091,
        help="ROS bridge WebSocket port (default: 9091)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout in seconds for rosbridge service calls (default: 5.0).",
    )
    parser.add_argument(
        "--image-topic",
        default="/camera/image_raw/compressed",
        help=(
            "Source image topic for image_yolo mode. "
            "Default: /camera/image_raw/compressed (sensor_msgs/CompressedImage). "
            "Use e.g. /camera/image_raw for raw sensor_msgs/Image."
        ),
    )
    parser.add_argument(
        "--annotated-image-topic",
        default="/camera/image_annotated",
        help="Destination topic for annotated images in image_yolo mode (default: /camera/image_annotated).",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Maximum number of images to process in image_yolo mode (default: 0; <=0 means infinite).",
    )
    parser.add_argument(
        "--no-yolo",
        action="store_true",
        help="In image_yolo mode, disable YOLO and just republish images.",
    )
    parser.add_argument(
        "--max-wait-seconds",
        type=float,
        default=0.0,
        help=(
            "In image_yolo mode with --max-images 0, maximum wall-clock "
            "seconds to wait before exiting (default: 0 for infinite)."
        ),
    )
    return parser.parse_args()


def _require_numpy() -> None:
    if np is None:  # type: ignore[truthy-function]
        print(
            "ERROR: numpy is required for image_yolo mode.\n"
            "Install it with:\n"
            "  pip install numpy",
            file=sys.stderr,
        )
        sys.exit(1)


def _require_yolo() -> None:
    if YOLO is None:  # type: ignore[truthy-function]
        print(
            "ERROR: ultralytics YOLO is required for image_yolo mode.\n"
            "Install it (and CPU deps) with:\n"
            "  pip install ultralytics opencv-python",
            file=sys.stderr,
        )
        sys.exit(1)


def _get_yolo_model():
    global YOLO_MODEL
    _require_yolo()
    if YOLO_MODEL is None:
        # Small default model for CPU inference.
        YOLO_MODEL = YOLO("yolov8n.pt")  # type: ignore[call-arg]
    return YOLO_MODEL


def _run_yolo_on_image(img_np):
    """
    Run YOLO on an RGB image and return an annotated RGB image.
    """
    print("Running YOLO inference on image ...")
    model = _get_yolo_model()
    # YOLO expects BGR images; convert from RGB.
    bgr = img_np[..., ::-1].copy()
    try:
        results = model(bgr, verbose=False, device="cpu")  # type: ignore[call-arg]
    except TypeError:
        # Older versions might not accept device kwarg.
        results = model(bgr, verbose=False)  # type: ignore[call-arg]
    annotated_bgr = results[0].plot()
    annotated_rgb = annotated_bgr[..., ::-1]
    print("YOLO inference finished.")
    return annotated_rgb


def _require_cv2() -> None:
    if cv2 is None:  # type: ignore[truthy-function]
        print(
            "ERROR: OpenCV (cv2) is required for compressed image mode.\n"
            "Install it with:\n"
            "  pip install opencv-python",
            file=sys.stderr,
        )
        sys.exit(1)


def _decode_ros_image(img_msg: dict):
    """
    Decode a ROS sensor_msgs/Image-style message from rosbridge into a numpy array.

    Returns (image_np, meta) where image_np is HxWxC uint8 and meta tracks how
    the data field was encoded so we can re-encode it later.
    """
    _require_numpy()

    height = img_msg["height"]
    width = img_msg["width"]
    step = img_msg.get("step")
    encoding = img_msg.get("encoding", "rgb8")
    data_field = img_msg["data"]

    if isinstance(data_field, str):
        raw = base64.b64decode(data_field)
        encoding_kind = "base64"
    else:
        raw = bytes(data_field)
        encoding_kind = "list"

    if step is None or step == 0:
        # Assume tightly packed RGB
        channels = 3
        step = width * channels
    else:
        channels = step // width

    img = np.frombuffer(raw, dtype=np.uint8)  # type: ignore[arg-type]
    img = img.reshape((height, width, channels))

    meta = {
        "encoding": encoding,
        "encoding_kind": encoding_kind,
        "step": step,
        "height": height,
        "width": width,
    }
    return img, meta


def _decode_compressed_ros_image(img_msg: dict):
    """
    Decode a ROS sensor_msgs/CompressedImage-style message into an RGB numpy array.

    Returns (image_np, meta) where image_np is HxWxC uint8 (RGB) and meta is
    compatible with _encode_ros_image so that we can republish as a raw
    sensor_msgs/Image message.
    """
    _require_numpy()
    _require_cv2()

    data_field = img_msg["data"]

    if isinstance(data_field, str):
        compressed = base64.b64decode(data_field)
        encoding_kind = "base64"
    else:
        compressed = bytes(data_field)
        encoding_kind = "list"

    buf = np.frombuffer(compressed, dtype=np.uint8)  # type: ignore[arg-type]
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # type: ignore[arg-type]
    if bgr is None:
        raise RuntimeError("cv2.imdecode returned None for compressed image")

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # type: ignore[arg-type]
    height, width, _channels = rgb.shape

    # We republish as rgb8 raw image data.
    meta = {
        "encoding": "rgb8",
        # We always encode annotated images as base64 strings when republishing
        # (works well over rosbridge and keeps messages compact).
        "encoding_kind": encoding_kind or "base64",
        "step": width * 3,
        "height": height,
        "width": width,
    }
    return rgb, meta


def _encode_ros_image(img, template_msg: dict, meta: dict) -> dict:
    """
    Encode a numpy image back into a sensor_msgs/Image-style dict.

    This copies the original message and overrides only the image-related fields.
    """
    _require_numpy()

    height, width, channels = img.shape
    buf = img.astype("uint8").tobytes()

    if meta["encoding_kind"] == "base64":
        data_field = base64.b64encode(buf).decode("ascii")
    else:
        data_field = list(buf)

    out = dict(template_msg)
    out["height"] = height
    out["width"] = width
    out["step"] = width * channels
    out["encoding"] = meta.get("encoding", "rgb8")
    out["data"] = data_field
    return out


def _lookup_topic_type(client: "roslibpy.Ros", topic: str, timeout: float = 5.0) -> str:
    """Ask rosapi for the type of a given topic for image streams."""
    service = roslibpy.Service(client, "/rosapi/topic_type", "rosapi/TopicType")
    request = roslibpy.ServiceRequest({"topic": topic})
    try:
        response = service.call(request, timeout=timeout)
    except Exception as exc:  # pragma: no cover - network-dependent
        print(
            f"WARNING: Failed to query /rosapi/topic_type for '{topic}': {exc}",
            file=sys.stderr,
        )
        return ""

    topic_type_ros2 = response.get("type", "") or ""
    if not topic_type_ros2:
        print(
            f"WARNING: /rosapi/topic_type returned empty type for '{topic}'.",
            file=sys.stderr,
        )
    else:
        print(f"Resolved topic '{topic}' type as '{topic_type_ros2}'.")

    # roslibpy expects ROS 1 style names (pkg/Msg), but rosapi on ROS 2 returns
    # 'pkg/msg/Msg'. Convert if needed.
    if "/msg/" in topic_type_ros2:
        pkg, msg = topic_type_ros2.split("/msg/", 1)
        topic_type_ros1 = f"{pkg}/{msg}"
    else:
        topic_type_ros1 = topic_type_ros2

    return topic_type_ros1


def image_yolo(client: "roslibpy.Ros", args: argparse.Namespace) -> None:
    """
    Simple pipeline:
      - subscribe to an image topic from the VM via rosbridge
      - convert to numpy
      - optionally run YOLO on CPU + draw annotations
      - publish annotated image to another ROS topic
    """
    _require_numpy()
    if not args.no_yolo:
        _require_yolo()

    src_topic = args.image_topic
    dst_topic = args.annotated_image_topic
    max_images = args.max_images

    print(f"Subscribing to image topic '{src_topic}' ...")
    resolved_type = _lookup_topic_type(client, src_topic, timeout=args.timeout)
    if not resolved_type:
        # Fallback to a common ROS 1 style type name expected by roslibpy.
        # We assume a raw Image here; compressed streams should normally be
        # discoverable via rosapi.
        resolved_type = "sensor_msgs/Image"
        print(
            f"Falling back to default type '{resolved_type}' for '{src_topic}'.",
        )

    print(f"Input topic '{src_topic}' resolved type: '{resolved_type}'.")
    input_is_compressed = "CompressedImage" in resolved_type

    subscriber = roslibpy.Topic(
        client,
        src_topic,
        resolved_type,
        queue_length=1,
    )

    # Always publish annotated frames as raw sensor_msgs/Image so that tools
    # like ImagePanel and RawMessagesPanel can visualize them easily.
    annotated_type = "sensor_msgs/Image"

    print(
        f"Advertising annotated image topic '{dst_topic}' "
        f"({annotated_type}) ...",
    )
    publisher = roslibpy.Topic(
        client,
        dst_topic,
        annotated_type,
    )

    processed = 0
    done_event = threading.Event()
    print("Waiting for images ...")

    def _on_image(msg):
        nonlocal processed
        print(f"Received image message on '{src_topic}'.")
        img_msg = msg

        if input_is_compressed:
            img_np, meta = _decode_compressed_ros_image(img_msg)
        else:
            img_np, meta = _decode_ros_image(img_msg)

        if args.no_yolo:
            annotated = img_np
        else:
            # Run YOLO on CPU and get annotated frame.
            annotated = _run_yolo_on_image(img_np)

        annotated_msg = _encode_ros_image(annotated, img_msg, meta)
        publisher.publish(roslibpy.Message(annotated_msg))

        processed += 1
        print(
            f"Processed image {processed} "
            f"({meta['width']}x{meta['height']}), "
            f"published to {dst_topic}",
        )

        if max_images > 0 and processed >= max_images:
            subscriber.unsubscribe()
            done_event.set()

    subscriber.subscribe(_on_image)

    if max_images > 0:
        # Wait until we've processed the requested number of images.
        done_event.wait()
    else:
        # Run until interrupted or until max_wait_seconds (if > 0) elapses.
        start = time.time()
        try:
            while True:
                if args.max_wait_seconds > 0.0 and (time.time() - start) > args.max_wait_seconds:
                    print(
                        f"No images received within {args.max_wait_seconds} seconds; "
                        "exiting image_yolo.",
                    )
                    subscriber.unsubscribe()
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            subscriber.unsubscribe()

    print("Completed image_yolo processing.")


def main() -> None:
    args = parse_args()
    print(f"Connecting to rosbridge at {args.host}:{args.port} using roslibpy ...")
    client = roslibpy.Ros(host=args.host, port=args.port)
    try:
        client.run()
    except Exception as exc:  # pragma: no cover - network-dependent
        print(f"ERROR: Failed to connect to rosbridge: {exc}", file=sys.stderr)
        sys.exit(1)

    print("roslibpy connection established successfully.")

    try:
        image_yolo(client, args)
    finally:
        client.terminate()


if __name__ == "__main__":
    main()
