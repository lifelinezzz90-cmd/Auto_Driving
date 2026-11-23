"""Autonomous multi-waypoint patrol node for PadBot E2."""

import math
import os
import signal
import sys
from typing import Any, Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert 2D yaw angle to quaternion (qx, qy, qz, qw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class E2WaypointPatrolNode(Node):
    """Node orchestrating multi-point waypoint patrol on PadBot E2."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize waypoint patrol node."""
        super().__init__('e2_waypoint_patrol', *args, **kwargs)

        # Determine default waypoints file path
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('yunyan_e2_bridge')
            default_cfg = os.path.join(
                pkg_share, 'config', 'e2_waypoints.yaml'
            )
        except Exception:
            default_cfg = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    '..',
                    'config',
                    'e2_waypoints.yaml',
                )
            )

        self.declare_parameter('waypoints_file', default_cfg)
        cfg_path = os.path.expanduser(
            str(self.get_parameter('waypoints_file').value)
        )

        self.get_logger().info(f'Loading waypoints config from: {cfg_path}')
        config = self._load_yaml_config(cfg_path)

        self.loops = int(config.get('loops', 1))
        self.goal_timeout_sec = float(config.get('goal_timeout_sec', 45.0))
        self.dwell_sec = float(config.get('dwell_sec', 2.0))
        self.retry_on_fail = int(config.get('retry_on_fail', 1))
        self.waypoints: List[Dict[str, Any]] = config.get('waypoints', [])

        # State machine variables
        self.state = 'SEND_GOAL'
        self.current_wp_idx = 0
        self.completed_loops = 0
        self.current_retry = 0
        self._last_status: Optional[str] = None
        self._status_seq = 0
        self._goal_sent_status_seq = 0
        self._cancel_future: Optional[Any] = None
        self._cancel_start_time = self.get_clock().now()
        self._shutdown_requested = False
        self.init_failed = False

        self.goal_start_time = self.get_clock().now()
        self.wait_exec_start_time = self.get_clock().now()
        self.dwell_start_time = self.get_clock().now()

        # ROS Communication
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sub_status = self.create_subscription(
            String, '/e2/action_status', self._on_action_status, 10
        )
        self.cancel_client = self.create_client(
            Trigger, '/e2/cancel_navigation'
        )

        # 10Hz state machine timer
        self.timer = self.create_timer(0.1, self._state_step)

        if not self.waypoints:
            self.get_logger().error(
                'No waypoints loaded! Config missing or waypoints empty. '
                'Aborting patrol and publishing NO goal.'
            )
            self.state = 'DONE'
            self.init_failed = True
            return

        self.get_logger().info(
            f'Patrol configured: {len(self.waypoints)} waypoints, '
            f'loops={self.loops}, timeout={self.goal_timeout_sec}s, '
            f'dwell={self.dwell_sec}s, retry={self.retry_on_fail}'
        )

    def _load_yaml_config(self, path: str) -> Dict[str, Any]:
        """Load and parse waypoints YAML configuration file."""
        if not os.path.exists(path):
            self.get_logger().error(f'Waypoints file not found: {path}')
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data if isinstance(data, dict) else {}
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoints YAML: {e}')
            return {}

    def _on_action_status(self, msg: String) -> None:
        """Cache latest motion action status and increment seq."""
        self._status_seq += 1
        self._last_status = msg.data.strip().upper()

    def request_shutdown(self) -> None:
        """Signal node to stop and safely cancel navigation."""
        self._shutdown_requested = True

    def is_done(self) -> bool:
        """Return True if patrol finished or shutdown requested."""
        return (
            self.state == 'DONE'
            or self._shutdown_requested
            or self.init_failed
        )

    def cancel_navigation_sync(self, timeout_sec: float = 2.0) -> None:
        """Cancel navigation synchronously on /e2/cancel_navigation."""
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                '/e2/cancel_navigation service unavailable for cancel.'
            )
            return

        req = Trigger.Request()
        future = self.cancel_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=timeout_sec
        )
        if future.done():
            res = future.result()
            self.get_logger().info(
                f'Cancel navigation response: {res.message if res else "OK"}'
            )
        else:
            self.get_logger().warn(
                'Timed out waiting for /e2/cancel_navigation response.'
            )

    def _state_step(self) -> None:
        """Periodic state machine transition step."""
        if (
            self._shutdown_requested
            or self.state == 'DONE'
            or self.init_failed
        ):
            return

        now = self.get_clock().now()

        if self.state == 'SEND_GOAL':
            wp = self.waypoints[self.current_wp_idx]
            name = wp.get('name', f'wp_{self.current_wp_idx}')
            x = float(wp.get('x', 0.0))
            y = float(wp.get('y', 0.0))
            yaw = float(wp.get('yaw', 0.0))

            goal_msg = PoseStamped()
            goal_msg.header.stamp = now.to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            goal_msg.pose.orientation.x = qx
            goal_msg.pose.orientation.y = qy
            goal_msg.pose.orientation.z = qz
            goal_msg.pose.orientation.w = qw

            self.get_logger().info(
                f'[Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}] '
                f'Dispatching "{name}" -> '
                f'(x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})'
            )

            # P1-4: Reset old status before publishing new goal
            self._last_status = None
            self._goal_sent_status_seq = self._status_seq
            self.pub_goal.publish(goal_msg)

            self.goal_start_time = now
            self.wait_exec_start_time = now
            self.state = 'WAIT_EXEC'

        elif self.state == 'WAIT_EXEC':
            wp = self.waypoints[self.current_wp_idx]
            dt_exec = (now - self.wait_exec_start_time).nanoseconds / 1e9
            has_new_status = self._status_seq > self._goal_sent_status_seq

            if self._last_status == 'EXECUTING':
                self.state = 'WAIT_RESULT'
            elif (
                has_new_status
                and self._last_status == 'SUCCEEDED'
                and dt_exec > 1.5
            ):
                # Arrived immediately (close target)
                self.get_logger().info(
                    f'Waypoint "{wp.get("name")}" reached immediately!'
                )
                self.state = 'DWELL'
                self.dwell_start_time = now
                self.current_retry = 0
            elif dt_exec > 5.0:
                self.get_logger().warn(
                    f'Waypoint "{wp.get("name")}" did not enter EXECUTING '
                    f'within 5.0s (status={self._last_status}). '
                    'Handling failure.'
                )
                self._handle_failure()

        elif self.state == 'WAIT_RESULT':
            wp = self.waypoints[self.current_wp_idx]
            dt_goal = (now - self.goal_start_time).nanoseconds / 1e9

            if self._last_status == 'SUCCEEDED':
                self.get_logger().info(
                    f'Waypoint "{wp.get("name")}" reached successfully! '
                    f'Dwelling for {self.dwell_sec:.1f}s.'
                )
                self.state = 'DWELL'
                self.dwell_start_time = now
                self.current_retry = 0
            elif self._last_status in ('FAILED', 'CANCELED'):
                self.get_logger().warn(
                    f'Waypoint "{wp.get("name")}" received status: '
                    f'{self._last_status}.'
                )
                self._handle_failure()
            elif dt_goal > self.goal_timeout_sec:
                self.get_logger().warn(
                    f'Waypoint "{wp.get("name")}" timed out after '
                    f'{dt_goal:.1f}s (limit: {self.goal_timeout_sec:.1f}s). '
                    'Canceling navigation asynchronously...'
                )
                if self.cancel_client.wait_for_service(timeout_sec=0.2):
                    self._cancel_future = self.cancel_client.call_async(
                        Trigger.Request()
                    )
                    self.state = 'WAIT_CANCEL'
                    self._cancel_start_time = now
                else:
                    self.get_logger().warn(
                        '/e2/cancel_navigation service unavailable for cancel.'
                    )
                    self._handle_failure()

        elif self.state == 'WAIT_CANCEL':
            dt_cancel = (now - self._cancel_start_time).nanoseconds / 1e9
            if self._cancel_future is not None and self._cancel_future.done():
                res = self._cancel_future.result()
                msg = res.message if res else 'OK'
                self.get_logger().info(
                    f'Async cancel navigation response: {msg}'
                )
                self._handle_failure()
            elif dt_cancel > 1.0:
                self.get_logger().warn(
                    'Timed out waiting for async cancel response.'
                )
                self._handle_failure()

        elif self.state == 'DWELL':
            dt_dwell = (now - self.dwell_start_time).nanoseconds / 1e9
            if dt_dwell >= self.dwell_sec:
                self._advance_waypoint()

    def _handle_failure(self) -> None:
        """Handle waypoint failure with optional retry."""
        wp = self.waypoints[self.current_wp_idx]
        name = wp.get('name', f'wp_{self.current_wp_idx}')

        if self.current_retry < self.retry_on_fail:
            self.current_retry += 1
            self.get_logger().warn(
                f'Retrying waypoint "{name}" '
                f'(attempt {self.current_retry}/{self.retry_on_fail})...'
            )
            self.state = 'SEND_GOAL'
        else:
            self.get_logger().error(
                f'Waypoint "{name}" failed after {self.retry_on_fail} '
                'retries. Skipping to next.'
            )
            self.current_retry = 0
            self._advance_waypoint()

    def _advance_waypoint(self) -> None:
        """Advance to next waypoint or loop/complete."""
        self.current_wp_idx += 1
        if self.current_wp_idx >= len(self.waypoints):
            self.completed_loops += 1
            total_loops_str = (
                str(self.loops) if self.loops > 0 else 'infinite'
            )
            self.get_logger().info(
                f'Finished patrol loop '
                f'{self.completed_loops}/{total_loops_str}.'
            )
            if self.loops > 0 and self.completed_loops >= self.loops:
                self.state = 'DONE'
                self.get_logger().info(
                    'Patrol mission completed successfully!'
                )
            else:
                self.current_wp_idx = 0
                self.state = 'SEND_GOAL'
        else:
            self.state = 'SEND_GOAL'


def main(args: Optional[Any] = None) -> None:
    """Entry point with clean Ctrl+C safety shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = E2WaypointPatrolNode()

    # Fast exit if waypoints configuration failed to load
    if getattr(node, 'init_failed', False):
        node.get_logger().error(
            'Node initialization failed (missing/empty waypoints). '
            'Exiting immediately without canceling.'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    shutdown_flag = False

    def sigint_handler(signum: int, frame: Any) -> None:
        nonlocal shutdown_flag
        shutdown_flag = True
        node.request_shutdown()

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    try:
        while rclpy.ok() and not shutdown_flag and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        node.get_logger().error(f'Unexpected error during patrol: {e}')
    finally:
        if shutdown_flag:
            node.get_logger().info(
                'Interrupted by Ctrl+C: Safely canceling E2 navigation...'
            )
            node.cancel_navigation_sync(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
