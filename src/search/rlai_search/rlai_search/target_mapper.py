"""
rlai_search/target_mapper.py

Projects 2D detections into map coordinates using the depth image and TF.

Subscribes:
  /search/detections         vision_msgs/Detection2DArray
  /depth_camera/depth        sensor_msgs/Image (32FC1, metres)
  /depth_camera/camera_info  sensor_msgs/CameraInfo

Publishes:
  /search/target_location    rlai_interfaces/ObjectLocation
  /search/target_markers     visualization_msgs/MarkerArray

Depth is sampled as the median of a window inside the detection box, which is
robust against holes and edge bleeding compared with a single centre pixel.
Invalid, NaN and out-of-range returns are discarded; when no valid sample
exists the detection is dropped instead of producing a bogus coordinate.

Usage:
  ros2 run rlai_search target_mapper --ros-args -p use_sim_time:=true
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from rlai_interfaces.msg import ObjectLocation


class TargetMapper(Node):
    """Turns the best detection of the current frame into a map pose."""

    def __init__(self) -> None:
        super().__init__('target_mapper')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detections_topic', '/search/detections')
        self.declare_parameter('depth_topic', '/depth_camera/depth')
        self.declare_parameter('camera_info_topic', '/depth_camera/camera_info')
        self.declare_parameter('location_topic', '/search/target_location')
        self.declare_parameter('marker_topic', '/search/target_markers')
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 6.0)
        self.declare_parameter('window_ratio', 0.4)
        self.declare_parameter('smoothing_alpha', 0.4)
        self.declare_parameter('publish_markers', True)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._window_ratio = float(self.get_parameter('window_ratio').value)
        self._alpha = float(self.get_parameter('smoothing_alpha').value)
        self._publish_markers = bool(self.get_parameter('publish_markers').value)

        self._bridge = CvBridge()
        self._depth = None
        self._depth_frame = ''
        self._depth_stamp = None
        self._camera = None
        self._smoothed = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._location_pub = self.create_publisher(
            ObjectLocation, str(self.get_parameter('location_topic').value), 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10
        )

        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth, 2
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), self._on_info, 5
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            self._on_detections,
            10,
        )

        self.get_logger().info(
            f'target_mapper ready: detections -> {self._map_frame} poses'
        )

    # ------------------------------------------------------------- callbacks

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self._depth_frame = msg.header.frame_id
            self._depth_stamp = msg.header.stamp
        except Exception as exc:  # noqa: BLE001 - bad depth encoding must not be fatal
            self.get_logger().error(f'depth conversion failed: {exc}', throttle_duration_sec=5.0)

    def _on_info(self, msg: CameraInfo) -> None:
        self._camera = msg

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self._depth is None or self._camera is None or not msg.detections:
            return

        detection = max(
            msg.detections,
            key=lambda item: item.results[0].hypothesis.score if item.results else 0.0,
        )
        if not detection.results:
            return

        label = detection.results[0].hypothesis.class_id
        score = float(detection.results[0].hypothesis.score)

        point = self._deproject(detection)
        if point is None:
            return

        map_point = self._to_map(point)
        if map_point is None:
            return

        position = self._smooth(label, map_point)
        self._publish(label, score, position)

    # --------------------------------------------------------------- geometry

    def _deproject(self, detection):
        """Return the object position in the depth optical frame, or None."""
        height, width = self._depth.shape[:2]

        u = int(round(detection.bbox.center.position.x))
        v = int(round(detection.bbox.center.position.y))
        if not (0 <= u < width and 0 <= v < height):
            return None

        half_w = max(int(detection.bbox.size_x * self._window_ratio * 0.5), 1)
        half_h = max(int(detection.bbox.size_y * self._window_ratio * 0.5), 1)
        patch = self._depth[
            max(v - half_h, 0):min(v + half_h + 1, height),
            max(u - half_w, 0):min(u + half_w + 1, width),
        ]
        if patch.size == 0:
            return None

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > self._min_depth) & (valid < self._max_depth)]
        if valid.size == 0:
            return None

        depth = float(np.median(valid))
        fx = self._camera.k[0]
        fy = self._camera.k[4]
        cx = self._camera.k[2]
        cy = self._camera.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return None

        point = PointStamped()
        point.header.frame_id = self._depth_frame
        if self._depth_stamp is not None:
            point.header.stamp = self._depth_stamp
        else:
            point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth
        return point

    def _to_map(self, point: PointStamped):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                point.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF lookup failed: {exc}', throttle_duration_sec=5.0
            )
            return None

        return do_transform_point(point, transform)

    def _smooth(self, label: str, point) -> tuple:
        """Exponential smoothing keeps the marker from jumping between frames."""
        measured = np.array([point.point.x, point.point.y, point.point.z])
        previous = self._smoothed.get(label)
        if previous is None:
            filtered = measured
        else:
            filtered = self._alpha * measured + (1.0 - self._alpha) * previous
        self._smoothed[label] = filtered
        return tuple(float(value) for value in filtered)

    # ---------------------------------------------------------------- output

    def _publish(self, label: str, score: float, position: tuple) -> None:
        stamp = self.get_clock().now().to_msg()

        location = ObjectLocation()
        location.header.stamp = stamp
        location.header.frame_id = self._map_frame
        location.label = label
        location.confidence = float(score)
        location.region = ''
        location.map_pose.header.stamp = stamp
        location.map_pose.header.frame_id = self._map_frame
        location.map_pose.pose.position.x = position[0]
        location.map_pose.pose.position.y = position[1]
        location.map_pose.pose.position.z = position[2]
        location.map_pose.pose.orientation.w = 1.0

        variance = 0.05 ** 2
        location.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        self._location_pub.publish(location)

        if not self._publish_markers:
            return

        markers = MarkerArray()
        arrow = Marker()
        arrow.header.stamp = stamp
        arrow.header.frame_id = self._map_frame
        arrow.ns = 'target'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = location.map_pose.pose
        arrow.scale.x = 0.5
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.2
        arrow.color.a = 1.0
        arrow.lifetime = Duration(seconds=5.0).to_msg()
        markers.markers.append(arrow)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = self._map_frame
        text.ns = 'target_label'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = position[0]
        text.pose.position.y = position[1]
        text.pose.position.z = position[2] + 0.4
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f'{label} ({position[0]:.2f}, {position[1]:.2f})'
        text.lifetime = Duration(seconds=5.0).to_msg()
        markers.markers.append(text)

        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetMapper()
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
