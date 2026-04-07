#!/usr/bin/env python3
"""
Vision client: subscribe to images over rosbridge, run YOLO, and republish.

This script:
  - Connects to the rosbridge WebSocket server (via proxy or direct)
  - Subscribes to an image topic from the VM via rosbridge
  - Converts to numpy
  - Optionally runs YOLO on CPU + draws annotations
  - Publishes annotated image to another ROS topic

Usage:
  python src/vision_yolo.py

Environment overrides:
  - TENSORFLEET_BASE_URL, TENSORFLEET_JWT (for proxy connection)
  - ROS_HOST, ROS_PORT, ROSBRIDGE_URL (for direct connection)
  - IMAGE_TOPIC, ANNOTATED_IMAGE_TOPIC
  - NO_YOLO (set to 1 to disable YOLO)
  - MAX_IMAGES, MAX_WAIT_SECONDS

Dependencies:
  pip install roslibpy numpy
  # For YOLO image mode:
  pip install ultralytics opencv-python
"""

import argparse
import base64
import os
import sys
import time
import threading

# Add parent directory to path for lib imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import roslibpy
except ImportError:
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it with: pip install roslibpy", file=sys.stderr)
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    np = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from lib.robotic_utils import connect_to_robot, Topic, Service
except ImportError:
    try:
        from robotic_utils import connect_to_robot, Topic, Service
    except ImportError:
        connect_to_robot = None
        Topic = None
        Service = None


YOLO_MODEL = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Vision client: subscribe to images over rosbridge, run YOLO, and republish.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=float(os.getenv("TIMEOUT", "5.0")),
        help="Timeout in seconds for rosbridge service calls (default: 5.0).",
    )
    parser.add_argument(
        "--image-topic",
        default=os.getenv("IMAGE_TOPIC", "/camera/image_raw"),
        help="Source image topic for image_yolo mode.",
    )
    parser.add_argument(
        "--annotated-image-topic",
        default=os.getenv("ANNOTATED_IMAGE_TOPIC", "/camera/image_annotated"),
        help="Destination topic for annotated images (default: /camera/image_annotated).",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=int(os.getenv("MAX_IMAGES", "0")),
        help="Maximum number of images to process (default: 0; <=0 means infinite).",
    )
    parser.add_argument(
        "--no-yolo",
        action="store_true",
        default=os.getenv("NO_YOLO", "").lower() in ("1", "true", "yes"),
        help="Disable YOLO and just republish images.",
    )
    parser.add_argument(
        "--max-wait-seconds",
        type=float,
        default=float(os.getenv("MAX_WAIT_SECONDS", "0.0")),
        help="Maximum wall-clock seconds to wait before exiting (default: 0 for infinite).",
    )
    return parser.parse_args()


def _require_numpy() -> None:
    if np is None:
        print(
            "ERROR: numpy is required for image_yolo mode.\n"
            "Install it with:\n"
            "  pip install numpy",
            file=sys.stderr,
        )
        sys.exit(1)


def _require_yolo() -> None:
    if YOLO is None:
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
        YOLO_MODEL = YOLO("yolov8n.pt")
    return YOLO_MODEL


_COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
    "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

_DETECTION_COLORS = [
    (92, 196, 214),
    (86, 170, 176),
    (232, 166, 76),
    (140, 160, 180),
    (214, 96, 96),
]


def _draw_detections(img_rgb, results):
    """Draw mission-UI style annotations onto an RGB image using OpenCV."""
    _require_cv2()
    out = img_rgb.copy()
    h, w = out.shape[:2]

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return out

    for idx, box in enumerate(boxes):
        xyxy = box.xyxy[0].tolist()
        x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
        score = float(box.conf[0])
        cls_id = int(box.cls[0])
        label = _COCO_CLASSES[cls_id] if cls_id < len(_COCO_CLASSES) else f"class_{cls_id}"
        label = label.replace("_", " ").title()

        color_rgb = _DETECTION_COLORS[idx % len(_DETECTION_COLORS)]
        color_bgr = (color_rgb[2], color_rgb[1], color_rgb[0])
        overlay = out.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color_bgr, 2)
        cv2.addWeighted(overlay, 0.88, out, 0.12, 0, out)

        score_pct = int(round(score * 100))
        text = f"{label} . {score_pct}%"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.42
        thickness = 1
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        pad_x, pad_y = 6, 3
        chip_w = tw + pad_x * 2
        chip_h = th + baseline + pad_y * 2

        cx = max(0, min(w - chip_w - 1, x1))
        cy = y1 - chip_h - 2
        if cy < 0:
            cy = y1 + 2

        chip_overlay = out.copy()
        cv2.rectangle(chip_overlay, (cx, cy), (cx + chip_w, cy + chip_h),
                      (10, 14, 18), cv2.FILLED)
        cv2.addWeighted(chip_overlay, 0.78, out, 0.22, 0, out)

        # 1px accent border: blend color_bgr at 0.28 over near-black chip
        border_bgr = tuple(int(10 * 0.72 + c * 0.28) for c in color_bgr)
        cv2.rectangle(out, (cx, cy), (cx + chip_w, cy + chip_h), border_bgr, 1)

        cv2.putText(out, text, (cx + pad_x, cy + pad_y + th),
                    font, font_scale, (235, 242, 245), thickness, cv2.LINE_AA)

    return out


def _run_yolo_on_image(img_np):
    """Run YOLO on an RGB image and return an annotated RGB image."""
    print("Running YOLO inference on image ...")
    model = _get_yolo_model()
    bgr = img_np[..., ::-1].copy()
    try:
        results = model(bgr, verbose=False, device="cpu")
    except TypeError:
        results = model(bgr, verbose=False)
    annotated_rgb = _draw_detections(img_np, results)
    print("YOLO inference finished.")
    return annotated_rgb


def _require_cv2() -> None:
    if cv2 is None:
        print(
            "ERROR: OpenCV (cv2) is required for compressed image mode.\n"
            "Install it with:\n"
            "  pip install opencv-python",
            file=sys.stderr,
        )
        sys.exit(1)


def _decode_ros_image(img_msg: dict):
    """Decode a ROS sensor_msgs/Image-style message from rosbridge into a numpy array."""
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
        channels = 3
        step = width * channels
    else:
        channels = step // width

    img = np.frombuffer(raw, dtype=np.uint8)
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
    """Decode a ROS sensor_msgs/CompressedImage-style message into an RGB numpy array."""
    _require_numpy()
    _require_cv2()

    data_field = img_msg["data"]

    if isinstance(data_field, str):
        compressed = base64.b64decode(data_field)
        encoding_kind = "base64"
    else:
        compressed = bytes(data_field)
        encoding_kind = "list"

    buf = np.frombuffer(compressed, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("cv2.imdecode returned None for compressed image")

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height, width, _channels = rgb.shape

    meta = {
        "encoding": "rgb8",
        "encoding_kind": encoding_kind or "base64",
        "step": width * 3,
        "height": height,
        "width": width,
    }
    return rgb, meta


def _encode_ros_image(img, template_msg: dict, meta: dict) -> dict:
    """Encode a numpy image back into a sensor_msgs/Image-style dict."""
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


def _lookup_topic_type(client, topic: str, timeout: float = 5.0) -> str:
    """Ask rosapi for the type of a given topic for image streams."""
    try:
        if Service:
            service = Service(client, "/rosapi/topic_type", "rosapi/TopicType")
        else:
            service = roslibpy.Service(client, "/rosapi/topic_type", "rosapi/TopicType")
        
        response = service.call({"topic": topic}, timeout=timeout)
    except Exception as exc:
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

    if "/msg/" in topic_type_ros2:
        pkg, msg = topic_type_ros2.split("/msg/", 1)
        topic_type_ros1 = f"{pkg}/{msg}"
    else:
        topic_type_ros1 = topic_type_ros2

    return topic_type_ros1


def image_yolo(client, args: argparse.Namespace) -> None:
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
        resolved_type = "sensor_msgs/Image"
        print(f"Falling back to default type '{resolved_type}' for '{src_topic}'.")

    print(f"Input topic '{src_topic}' resolved type: '{resolved_type}'.")
    input_is_compressed = "CompressedImage" in resolved_type

    # Use Topic factory for compatibility
    if Topic:
        subscriber = Topic(client, src_topic, resolved_type, queue_length=1)
        publisher = Topic(client, dst_topic, "sensor_msgs/Image")
    else:
        subscriber = roslibpy.Topic(client, src_topic, resolved_type, queue_length=1)
        publisher = roslibpy.Topic(client, dst_topic, "sensor_msgs/Image")

    print(
        f"Advertising annotated image topic '{dst_topic}' "
        f"(sensor_msgs/Image) ..."
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
            annotated = _run_yolo_on_image(img_np)

        annotated_msg = _encode_ros_image(annotated, img_msg, meta)
        publisher.publish(annotated_msg)

        processed += 1
        print(
            f"Processed image {processed} "
            f"({meta['width']}x{meta['height']}), "
            f"published to {dst_topic}"
        )

        if max_images > 0 and processed >= max_images:
            subscriber.unsubscribe()
            done_event.set()

    subscriber.subscribe(_on_image)

    if max_images > 0:
        done_event.wait()
    else:
        start = time.time()
        try:
            while True:
                if args.max_wait_seconds > 0.0 and (time.time() - start) > args.max_wait_seconds:
                    print(
                        f"No images received within {args.max_wait_seconds} seconds; "
                        "exiting image_yolo."
                    )
                    subscriber.unsubscribe()
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            subscriber.unsubscribe()

    print("Completed image_yolo processing.")


def main() -> None:
    args = parse_args()
    client = None

    try:
        if connect_to_robot:
            client = connect_to_robot()
        else:
            # Fallback to direct connection
            host = os.getenv("ROS_HOST", "172.16.0.10")
            port = int(os.getenv("ROS_PORT", "9091"))
            print(f"Connecting to rosbridge at {host}:{port} using roslibpy ...")
            client = roslibpy.Ros(host=host, port=port)
            client.run()

        print("roslibpy connection established successfully.")
        image_yolo(client, args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if client:
            client.terminate()


if __name__ == "__main__":
    main()
