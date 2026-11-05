"""
rlai_search/search_manager.py

FindObject action server: drives the robot through the configured search points,
asks the detector for the current target, and reports where the object was seen.

Flow:
  1. publish the target text on /search/target_label
  2. for each region in search_points.yaml: NavigateToPose -> observe
  3. accept a detection once /search/target_location confidence passes
     min_confidence; when only a weak detection appears, extend the observation
     window once before moving on (second-look mechanism)
  4. abort the current navigation when /safety_status reports a blockage

The node runs on a MultiThreadedExecutor so action feedback and sensor
callbacks keep flowing while the execute callback blocks on navigation.

Usage:
  ros2 run rlai_search search_manager --ros-args -p use_sim_time:=true
  ros2 action send_goal /find_object rlai_interfaces/action/FindObject \
    "{target_label: backpack}" --feedback
"""

import math
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from rlai_interfaces.action import FindObject
from rlai_interfaces.msg import ObjectLocation, SafetyStatus


class SearchManager(Node):
    """Sequences navigation and observation to locate a text-described object."""

    def __init__(self) -> None:
        super().__init__('search_manager')

        default_points = f'{get_package_share_directory("rlai_search")}/config/search_points.yaml'
        self.declare_parameter('search_points_file', default_points)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('review_confidence', 0.25)
        self.declare_parameter('observe_timeout', 8.0)
        self.declare_parameter('review_timeout', 6.0)
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('abort_on_safety', True)
        self.declare_parameter('target_topic', '/search/target_label')
        self.declare_parameter('location_topic', '/search/target_location')
        self.declare_parameter('safety_topic', '/safety_status')
        self.declare_parameter('nav_action', 'navigate_to_pose')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._review_confidence = float(self.get_parameter('review_confidence').value)
        self._observe_timeout = float(self.get_parameter('observe_timeout').value)
        self._review_timeout = float(self.get_parameter('review_timeout').value)
        self._settle_time = float(self.get_parameter('settle_time').value)
        self._nav_timeout = float(self.get_parameter('nav_timeout').value)
        self._abort_on_safety = bool(self.get_parameter('abort_on_safety').value)

        self._regions = self._load_regions(str(self.get_parameter('search_points_file').value))
        if not self._regions:
            self.get_logger().warn('no search regions configured; every goal will fail')

        self._location = None
        self._location_time = 0.0
        self._safety = None
        self._safety_time = 0.0

        callback_group = ReentrantCallbackGroup()

        self._target_pub = self.create_publisher(
            String, str(self.get_parameter('target_topic').value), 10
        )
        self.create_subscription(
            ObjectLocation,
            str(self.get_parameter('location_topic').value),
            self._on_location,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            SafetyStatus,
            str(self.get_parameter('safety_topic').value),
            self._on_safety,
            10,
            callback_group=callback_group,
        )

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action').value),
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            FindObject,
            'find_object',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f'search_manager ready with {len(self._regions)} region(s); '
            'action: /find_object'
        )

    # ------------------------------------------------------------- interface

    def _load_regions(self, path: str) -> list:
        try:
            with open(path, encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().error(f'cannot read search points "{path}": {exc}')
            return []

        frame_id = data.get('frame_id')
        if frame_id:
            self._frame_id = str(frame_id)

        regions = []
        for entry in data.get('regions', []):
            if 'name' not in entry:
                continue
            regions.append({
                'name': str(entry['name']),
                'x': float(entry.get('x', 0.0)),
                'y': float(entry.get('y', 0.0)),
                'yaw': float(entry.get('yaw', 0.0)),
            })
        return regions

    def _on_goal(self, goal_request) -> GoalResponse:
        if not goal_request.target_label.strip():
            self.get_logger().warn('rejecting goal: target_label is empty')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('cancel request received')
        return CancelResponse.ACCEPT

    def _on_location(self, msg: ObjectLocation) -> None:
        self._location = msg
        self._location_time = time.monotonic()

    def _on_safety(self, msg: SafetyStatus) -> None:
        self._safety = msg
        self._safety_time = time.monotonic()

    # -------------------------------------------------------------- execution

    def _execute(self, goal_handle) -> FindObject.Result:
        request = goal_handle.request
        target = request.target_label.strip()
        started = time.monotonic()

        result = FindObject.Result()
        result.success = False

        if not target:
            result.message = 'empty target_label'
            goal_handle.abort()
            return result
        if not self._regions:
            result.message = 'no search regions configured'
            goal_handle.abort()
            return result

        regions = self._regions
        if request.max_regions > 0:
            regions = regions[:request.max_regions]

        self._publish_target(target)
        total = len(regions)
        self.get_logger().info(f'searching for "{target}" over {total} region(s)')

        for index, region in enumerate(regions, start=1):
            if goal_handle.is_cancel_requested:
                result.message = 'cancelled'
                result.elapsed_sec = float(time.monotonic() - started)
                goal_handle.canceled()
                return result

            region_name = region['name']
            self._publish_feedback(goal_handle, 'NAVIGATING', region_name, index - 1, total)
            if not self._navigate(region, goal_handle):
                self.get_logger().warn(f'navigation to "{region_name}" failed; skipping')
                continue

            self._location = None
            time.sleep(self._settle_time)

            self._publish_feedback(goal_handle, 'OBSERVING', region_name, index - 1, total)
            found = self._observe(goal_handle, region, max(request.min_confidence, 0.0))
            if found is not None:
                result.success = True
                result.location = found.map_pose
                result.region = region_name
                result.confidence = float(found.confidence)
                result.elapsed_sec = float(time.monotonic() - started)
                result.message = f'found "{target}" in region "{region_name}"'
                self.get_logger().info(result.message)
                goal_handle.succeed()
                return result

        result.elapsed_sec = float(time.monotonic() - started)
        result.message = f'"{target}" not found in any searched region'
        self.get_logger().warn(result.message)
        goal_handle.abort()
        return result

    def _navigate(self, region: dict, goal_handle) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server is not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(region)

        send_future = self._nav_client.send_goal_async(goal)
        nav_goal_handle = self._wait(send_future, 10.0)
        if nav_goal_handle is None or not nav_goal_handle.accepted:
            return False

        result_future = nav_goal_handle.get_result_async()
        deadline = time.monotonic() + self._nav_timeout

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                return False
            if self._safety_blocked():
                self.get_logger().warn('cancelling navigation: forward path blocked')
                nav_goal_handle.cancel_goal_async()
                return False
            if time.monotonic() > deadline:
                self.get_logger().warn('navigation timed out')
                nav_goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def _observe(self, goal_handle, region: dict, threshold: float):
        """Return an ObjectLocation that passes the threshold, or None."""
        region_name = region['name']
        started = time.monotonic()
        best = None

        while time.monotonic() - started < self._observe_timeout:
            if goal_handle.is_cancel_requested:
                return None
            location = self._fresh_location(started)
            if location is not None:
                if location.confidence >= threshold:
                    return location
                if best is None or location.confidence > best.confidence:
                    best = location
            time.sleep(0.1)

        if best is None or best.confidence < self._review_confidence:
            return None

        # Second look: the detector saw something weak, give it another window.
        self.get_logger().info(
            f'weak detection in "{region_name}" ({best.confidence:.2f}); observing again'
        )

        review_started = time.monotonic()
        while time.monotonic() - review_started < self._review_timeout:
            if goal_handle.is_cancel_requested:
                return None
            location = self._fresh_location(review_started)
            if location is not None and location.confidence >= threshold:
                return location
            time.sleep(0.1)
        return None

    # ----------------------------------------------------------------- helpers

    def _fresh_location(self, since: float):
        if self._location is None or self._location_time < since:
            return None
        return self._location

    def _safety_blocked(self) -> bool:
        if not self._abort_on_safety or self._safety is None:
            return False
        # Ignore stale safety reports so a dead publisher cannot stall the task.
        if time.monotonic() - self._safety_time > 2.0:
            return False
        return bool(self._safety.blocked)

    def _make_pose(self, region: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = region['x']
        pose.pose.position.y = region['y']
        pose.pose.orientation.z = math.sin(region['yaw'] * 0.5)
        pose.pose.orientation.w = math.cos(region['yaw'] * 0.5)
        return pose

    def _publish_target(self, target: str) -> None:
        message = String()
        message.data = target
        self._target_pub.publish(message)

    def _publish_feedback(
        self, goal_handle, state: str, region: str, done: int, total: int
    ) -> None:
        feedback = FindObject.Feedback()
        feedback.state = state
        feedback.current_region = region
        feedback.regions_done = done
        feedback.regions_total = total
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _wait(future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline or not rclpy.ok():
                return None
            time.sleep(0.05)
        return future.result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SearchManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
