"""
rlai_object_detection/object_detector.py

Open-vocabulary object detection driven by a text target.

Pipeline:
  /depth_camera/image_raw  ->  YOLOE (class prompt = target text)
      ->  /search/detections        (vision_msgs/Detection2DArray)
      ->  /search/detections/image  (optional debug overlay)

A single model instance serves every target because the class prompt is set at
runtime from `/search/target_label`.  ultralytics is imported lazily: the node
still starts and reports a clear error when the package is missing.

Install the runtime dependency with:
  python3 -m pip install --user ultralytics

Usage:
  ros2 run rlai_object_detection object_detector --ros-args \
    -p model_path:=$HOME/models/yoloe-11s-seg.pt -p use_sim_time:=true
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLOE
    ULTRALYTICS_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on the local environment
    YOLOE = None
    ULTRALYTICS_AVAILABLE = False


class ObjectDetector(Node):
    """Runs an open-vocabulary detector on the RGB stream for a text prompt."""

    def __init__(self) -> None:
        super().__init__('object_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('max_rate_hz', 8.0)
        self.declare_parameter('warmup', True)
        self.declare_parameter('initial_label', '')
        self.declare_parameter('image_topic', '/depth_camera/image_raw')
        self.declare_parameter('target_topic', '/search/target_label')
        self.declare_parameter('detections_topic', '/search/detections')
        self.declare_parameter('debug_image_topic', '/search/detections/image')
        self.declare_parameter('publish_debug_image', True)

        self._model_path = str(self.get_parameter('model_path').value)
        self._device = str(self.get_parameter('device').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._confidence = float(self.get_parameter('confidence').value)
        max_rate = float(self.get_parameter('max_rate_hz').value)
        self._min_interval = 1.0 / max(max_rate, 0.5)
        self._warmup_enabled = bool(self.get_parameter('warmup').value)

        image_topic = str(self.get_parameter('image_topic').value)
        target_topic = str(self.get_parameter('target_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        debug_topic = str(self.get_parameter('debug_image_topic').value)
        self._publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self._bridge = CvBridge()
        self._model = None
        self._current_label = ''
        self._last_inference = 0.0

        self._detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, debug_topic, 2)
        self.create_subscription(Image, image_topic, self._on_image, 2)
        self.create_subscription(String, target_topic, self._on_target, 10)

        self._load_model()

        initial_label = str(self.get_parameter('initial_label').value)
        if initial_label:
            self._set_prompt(initial_label)

        self.get_logger().info(
            f'object_detector started: image={image_topic} target={target_topic} '
            f'out={detections_topic}'
        )

    # ------------------------------------------------------------------ model

    def _load_model(self) -> None:
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                'ultralytics is not installed; run: python3 -m pip install --user ultralytics'
            )
            return
        if not self._model_path:
            self.get_logger().warn(
                'model_path is empty; set it to a YOLOE checkpoint to enable detection'
            )
            return
        if self._device.startswith('cuda'):
            try:
                import torch
                torch.backends.cudnn.benchmark = True
            except ImportError:
                pass

        try:
            self._model = YOLOE(self._model_path)
            self._model.to(self._device)
            self.get_logger().info(f'loaded {self._model_path} on {self._device}')
            self._warmup()
        except Exception as exc:  # noqa: BLE001 - surface any loader failure
            self._model = None
            self.get_logger().error(f'failed to load model: {exc}')

    def _warmup(self) -> None:
        """Pay the CLIP build and CUDA kernel costs at startup, not on frame one."""
        if not self._warmup_enabled:
            return

        started = time.monotonic()
        try:
            self._set_prompt('object')
            dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
            self._model.predict(
                dummy,
                imgsz=self._imgsz,
                conf=self._confidence,
                device=self._device,
                verbose=False,
            )
            self._current_label = ''
            self.get_logger().info(f'warmup completed in {time.monotonic() - started:.1f} s')
        except Exception as exc:  # noqa: BLE001 - warmup is best effort
            self.get_logger().warn(f'warmup failed: {exc}')

    def _on_target(self, msg: String) -> None:
        label = msg.data.strip()
        if not label:
            return
        self._set_prompt(label)

    def _set_prompt(self, label: str) -> None:
        if label == self._current_label:
            return
        if self._model is None:
            self._current_label = label
            self.get_logger().warn(f'target set to "{label}" but no model is loaded')
            return

        try:
            self._model.set_classes([label])
        except TypeError:
            # Older releases need the text embeddings explicitly.
            embeddings = self._model.get_text_pe([label])
            self._model.set_classes([label], embeddings)
        except Exception as exc:  # noqa: BLE001 - keep the node alive
            self.get_logger().error(f'failed to set class prompt "{label}": {exc}')
            return

        self._current_label = label
        self.get_logger().info(f'class prompt set to "{label}"')

    # -------------------------------------------------------------- inference

    def _on_image(self, msg: Image) -> None:
        if self._model is None or not self._current_label:
            return

        now = time.monotonic()
        if now - self._last_inference < self._min_interval:
            return
        self._last_inference = now

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - bad encoding should not kill the node
            self.get_logger().error(f'image conversion failed: {exc}')
            return

        # cv_bridge can hand back a strided view of the message buffer; the
        # ultralytics letterbox step needs a contiguous array.
        frame = np.ascontiguousarray(frame)

        try:
            results = self._model.predict(
                frame,
                imgsz=self._imgsz,
                conf=self._confidence,
                device=self._device,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001 - keep publishing on inference errors
            self.get_logger().error(f'inference failed: {exc}')
            return

        self._publish_detections(results, msg.header, frame)

    def _publish_detections(self, results, header, frame) -> None:
        output = Detection2DArray()
        output.header = header

        boxes = getattr(results[0], 'boxes', None) if results else None
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                score = float(box.conf[0]) if box.conf is not None else 0.0

                detection = Detection2D()
                detection.header = header
                detection.bbox.center.position.x = (x1 + x2) * 0.5
                detection.bbox.center.position.y = (y1 + y2) * 0.5
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = max(x2 - x1, 0.0)
                detection.bbox.size_y = max(y2 - y1, 0.0)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self._current_label
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                output.detections.append(detection)

                if self._publish_debug and self._debug_pub is not None:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
                    cv2.putText(
                        frame,
                        f'{self._current_label} {score:.2f}',
                        (int(x1), max(int(y1) - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 200, 0),
                        1,
                        cv2.LINE_AA,
                    )

        self._detections_pub.publish(output)

        if self._publish_debug and self._debug_pub is not None and output.detections:
            try:
                self._debug_pub.publish(self._bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            except Exception as exc:  # noqa: BLE001 - debug output must never be fatal
                self.get_logger().debug(f'debug image publish failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
