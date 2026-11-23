"""ROS 2 Humble bridge node for PadBot E2 mobile chassis."""

import base64
import io
import json
import logging
import math
import os
import threading
import time
from typing import Any, List, Optional, Tuple

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from yunyan_e2_bridge.e2_api_client import E2ApiClient


def quaternion_to_yaw(
    qx: float, qy: float, qz: float, qw: float
) -> float:
    """Convert quaternion (x, y, z, w) to 2D yaw angle."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert 2D yaw angle to quaternion (qx, qy, qz, qw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def save_occupancy_grid_to_pgm_yaml(
    grid: OccupancyGrid,
    export_dir: str,
    base_name: str,
) -> Tuple[str, str]:
    """Export OccupancyGrid to ROS standard .pgm and .yaml files."""
    os.makedirs(export_dir, exist_ok=True)
    width = grid.info.width
    height = grid.info.height
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    oyaw = quaternion_to_yaw(
        grid.info.origin.orientation.x,
        grid.info.origin.orientation.y,
        grid.info.origin.orientation.z,
        grid.info.origin.orientation.w,
    )

    pgm_filename = f'{base_name}.pgm'
    yaml_filename = f'{base_name}.yaml'
    pgm_path = os.path.join(export_dir, pgm_filename)
    yaml_path = os.path.join(export_dir, yaml_filename)

    # Y-axis vertical flip: write rows from height - 1 down to 0
    raw_bytes = bytearray(width * height)
    for pgm_y in range(height):
        grid_y = height - 1 - pgm_y
        row_grid_offset = grid_y * width
        row_pgm_offset = pgm_y * width
        for x in range(width):
            v = grid.data[row_grid_offset + x]
            if v == -1:
                pixel = 205
            elif v == 0:
                pixel = 254
            elif v == 100:
                pixel = 0
            else:
                pixel = max(
                    0, min(254, int(round((100 - v) * 254.0 / 100.0)))
                )
            raw_bytes[row_pgm_offset + x] = pixel

    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(raw_bytes)

    yaml_content = (
        f'image: {pgm_filename}\n'
        f'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{ox:.4f}, {oy:.4f}, {oyaw:.4f}]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.25\n'
    )
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)

    return pgm_path, yaml_path


def _parse_occupancy_grid(
    raw_data: Any,
    frame_id: str = 'map',
    stamp: Optional[Any] = None,
) -> Optional[OccupancyGrid]:
    """Parse raw occupancy grid data supporting multiple formats."""
    if raw_data is None:
        return None

    # Handle JSON string
    if isinstance(raw_data, str):
        try:
            raw_data = json.loads(raw_data)
        except Exception:
            pass

    # If raw_data is dict, check if 'map' wrapper exists
    if isinstance(raw_data, dict):
        if 'map' in raw_data and isinstance(raw_data['map'], (dict, str)):
            if isinstance(raw_data['map'], dict) and (
                'width' in raw_data['map'] or 'data' in raw_data['map']
            ):
                raw_data = raw_data['map']
            elif isinstance(raw_data['map'], str):
                try:
                    loaded = json.loads(raw_data['map'])
                    if isinstance(loaded, dict):
                        raw_data = loaded
                except Exception:
                    pass

    # Format 1: JSON dict with width, height, resolution, origin, data/grid
    if (
        isinstance(raw_data, dict) and
        'width' in raw_data and
        'height' in raw_data
    ):
        raw_grid = raw_data.get('data', raw_data.get('grid', []))
        if isinstance(raw_grid, (list, tuple)):
            width = int(raw_data['width'])
            height = int(raw_data['height'])
            res = float(raw_data.get('resolution', 0.05))

            origin = raw_data.get('origin', [0.0, 0.0, 0.0])
            if isinstance(origin, dict):
                ox = float(origin.get('x', origin.get('X', 0.0)))
                oy = float(origin.get('y', origin.get('Y', 0.0)))
                oyaw = float(origin.get('yaw', origin.get('Yaw', 0.0)))
            elif isinstance(origin, (list, tuple)):
                ox = float(origin[0]) if len(origin) > 0 else 0.0
                oy = float(origin[1]) if len(origin) > 1 else 0.0
                oyaw = float(origin[2]) if len(origin) > 2 else 0.0
            else:
                ox, oy, oyaw = 0.0, 0.0, 0.0

            # Flatten 2D grid if nested
            if raw_grid and isinstance(raw_grid[0], (list, tuple)):
                flat_data = [int(v) for row in raw_grid for v in row]
            else:
                flat_data = [int(v) for v in raw_grid]

            target_len = width * height
            if len(flat_data) < target_len:
                flat_data.extend([-1] * (target_len - len(flat_data)))
            elif len(flat_data) > target_len:
                flat_data = flat_data[:target_len]

            clamped_data = [max(-1, min(100, v)) for v in flat_data]

            grid_msg = OccupancyGrid()
            grid_msg.header.frame_id = frame_id
            if stamp is not None:
                grid_msg.header.stamp = stamp
            grid_msg.info.map_load_time = grid_msg.header.stamp
            grid_msg.info.resolution = res
            grid_msg.info.width = width
            grid_msg.info.height = height
            grid_msg.info.origin.position.x = ox
            grid_msg.info.origin.position.y = oy
            grid_msg.info.origin.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(oyaw)
            grid_msg.info.origin.orientation.x = qx
            grid_msg.info.origin.orientation.y = qy
            grid_msg.info.origin.orientation.z = qz
            grid_msg.info.origin.orientation.w = qw
            grid_msg.data = clamped_data
            return grid_msg

    # Format 2: Base64-encoded grayscale/PNG image
    b64_candidate = None
    if isinstance(raw_data, str):
        b64_candidate = raw_data
    elif isinstance(raw_data, dict):
        for k in ('image', 'b64', 'data', 'map'):
            if isinstance(raw_data.get(k), str) and len(raw_data[k]) > 32:
                b64_candidate = raw_data[k]
                break

    if b64_candidate is not None:
        try:
            from PIL import Image
        except ImportError:
            logging.getLogger('e2_bridge_node').warning(
                'Missing Pillow, install via: sudo apt install python3-pil'
            )
            return None

        try:
            if ',' in b64_candidate:
                b64_candidate = b64_candidate.split(',', 1)[1]
            img_bytes = base64.b64decode(b64_candidate)

            img = Image.open(io.BytesIO(img_bytes)).convert('L')
            width, height = img.size
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            pixels = list(img.getdata())

            data: List[int] = []
            for p in pixels:
                if p == 255:
                    data.append(0)
                elif p == 0:
                    data.append(100)
                elif 125 <= p <= 210:
                    data.append(-1)
                else:
                    data.append(
                        max(0, min(100, int(round((255 - p) * 100.0 / 255.0))))
                    )

            res = 0.05
            ox, oy, oyaw = 0.0, 0.0, 0.0
            if isinstance(raw_data, dict):
                res = float(raw_data.get('resolution', 0.05))
                origin = raw_data.get('origin')
                if isinstance(origin, dict):
                    ox = float(origin.get('x', origin.get('X', 0.0)))
                    oy = float(origin.get('y', origin.get('Y', 0.0)))
                    oyaw = float(origin.get('yaw', origin.get('Yaw', 0.0)))
                elif isinstance(origin, (list, tuple)):
                    ox = float(origin[0]) if len(origin) > 0 else 0.0
                    oy = float(origin[1]) if len(origin) > 1 else 0.0
                    oyaw = float(origin[2]) if len(origin) > 2 else 0.0

            grid_msg = OccupancyGrid()
            grid_msg.header.frame_id = frame_id
            if stamp is not None:
                grid_msg.header.stamp = stamp
            grid_msg.info.map_load_time = grid_msg.header.stamp
            grid_msg.info.resolution = res
            grid_msg.info.width = width
            grid_msg.info.height = height
            grid_msg.info.origin.position.x = ox
            grid_msg.info.origin.position.y = oy
            grid_msg.info.origin.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(oyaw)
            grid_msg.info.origin.orientation.x = qx
            grid_msg.info.origin.orientation.y = qy
            grid_msg.info.origin.orientation.z = qz
            grid_msg.info.origin.orientation.w = qw
            grid_msg.data = data
            return grid_msg
        except Exception:
            pass

    return None


class E2BridgeNode(Node):
    """ROS 2 Node bridging ROS topics/services with E2 RESTful API."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize E2BridgeNode."""
        super().__init__('e2_bridge_node', *args, **kwargs)

        # Declare parameters
        self.declare_parameter('base_url', 'http://127.0.0.1:8080')
        self.declare_parameter('pose_poll_rate', 5.0)
        self.declare_parameter('action_poll_rate', 2.0)
        self.declare_parameter('map_poll_rate', 0.5)
        self.declare_parameter('max_linear_vel', 0.30)
        self.declare_parameter('max_angular_vel', 0.60)
        self.declare_parameter('teleop_step_dist', 0.15)
        self.declare_parameter('teleop_step_rad', 0.25)
        self.declare_parameter('teleop_min_interval', 0.25)
        self.declare_parameter('teleop_watchdog_sec', 1.0)
        self.declare_parameter('odom_frame', 'map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('default_map_name', 'e2_real_scene')
        self.declare_parameter('map_export_dir', '')

        # Callback Groups
        self.high_priority_cb_group = ReentrantCallbackGroup()
        self.slow_cb_group = MutuallyExclusiveCallbackGroup()

        # Get parameters
        base_url = str(self.get_parameter('base_url').value)
        pose_rate = float(self.get_parameter('pose_poll_rate').value)
        action_rate = float(self.get_parameter('action_poll_rate').value)
        map_rate = float(self.get_parameter('map_poll_rate').value)
        self.max_linear_vel = float(
            self.get_parameter('max_linear_vel').value
        )
        self.max_angular_vel = float(
            self.get_parameter('max_angular_vel').value
        )
        self.teleop_step_dist = float(
            self.get_parameter('teleop_step_dist').value
        )
        self.teleop_step_rad = float(
            self.get_parameter('teleop_step_rad').value
        )
        self.teleop_min_interval = float(
            self.get_parameter('teleop_min_interval').value
        )
        self.teleop_watchdog_sec = float(
            self.get_parameter('teleop_watchdog_sec').value
        )
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.default_map_name = str(
            self.get_parameter('default_map_name').value
        )
        self.map_export_dir = str(
            self.get_parameter('map_export_dir').value
        ).strip()

        # REST client
        self.client = E2ApiClient(base_url=base_url)
        self.get_logger().info(f'E2 bridge initialized for: {base_url}')

        # State tracking for edge-triggered stop, watchdog & map cache
        self._cmd_lock = threading.Lock()
        self._last_cmd_nonzero = False
        self._last_nonzero_cmd_time = 0.0
        self._last_teleop_send_time = 0.0
        self._warned_diagonal_cmd = False
        self._last_occupancy_grid: Optional[OccupancyGrid] = None
        self._warned_unknown_map_format = False

        # Subscriptions - High priority group
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._on_goal_pose,
            10,
            callback_group=self.high_priority_cb_group,
        )
        self.sub_init = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10,
            callback_group=self.high_priority_cb_group,
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._on_cmd_vel,
            10,
            callback_group=self.high_priority_cb_group,
        )

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/e2/odom', 10)
        self.pub_action_status = self.create_publisher(
            String, '/e2/action_status', 10
        )
        self.pub_plan = self.create_publisher(Path, '/e2/plan', 10)

        # Map publisher with Transient Local QoS
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_map = self.create_publisher(OccupancyGrid, '/e2/map', map_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Services - High priority: cancel navigation
        self.srv_cancel = self.create_service(
            Trigger,
            '/e2/cancel_navigation',
            self._srv_cancel_nav,
            callback_group=self.high_priority_cb_group,
        )

        # Services - Slow / heavy group
        self.srv_mapping = self.create_service(
            Trigger,
            '/e2/switch_to_mapping',
            self._srv_switch_to_mapping,
            callback_group=self.slow_cb_group,
        )
        self.srv_start_new_mapping = self.create_service(
            Trigger,
            '/e2/start_new_mapping',
            self._srv_start_new_mapping,
            callback_group=self.slow_cb_group,
        )
        self.srv_nav = self.create_service(
            Trigger,
            '/e2/switch_to_navigation',
            self._srv_switch_to_nav,
            callback_group=self.slow_cb_group,
        )
        self.srv_save_map = self.create_service(
            Trigger,
            '/e2/save_map',
            self._srv_save_map,
            callback_group=self.slow_cb_group,
        )
        self.srv_export_map = self.create_service(
            Trigger,
            '/e2/export_map',
            self._srv_export_map,
            callback_group=self.slow_cb_group,
        )

        # Timers - Slow / polling group
        pose_period = 1.0 / max(0.5, pose_rate)
        action_period = 1.0 / max(0.5, action_rate)
        map_period = 1.0 / max(0.1, map_rate)

        self.pose_timer = self.create_timer(
            pose_period,
            self._on_pose_poll_timer,
            callback_group=self.slow_cb_group,
        )
        self.action_timer = self.create_timer(
            action_period,
            self._on_action_poll_timer,
            callback_group=self.slow_cb_group,
        )
        self.map_timer = self.create_timer(
            map_period,
            self._on_map_poll_timer,
            callback_group=self.slow_cb_group,
        )

        # Watchdog Timer - High priority group (5 Hz = 0.2s)
        self.watchdog_timer = self.create_timer(
            0.2,
            self._on_teleop_watchdog_timer,
            callback_group=self.high_priority_cb_group,
        )

    def _parse_occupancy_grid(self, raw_data: Any) -> Optional[OccupancyGrid]:
        """Parse occupancy grid with node timestamp and logging."""
        stamp = self.get_clock().now().to_msg()
        grid = _parse_occupancy_grid(
            raw_data, frame_id=self.map_frame, stamp=stamp
        )
        if grid is None and raw_data is not None:
            if not self._warned_unknown_map_format:
                keys = (
                    list(raw_data.keys())
                    if isinstance(raw_data, dict)
                    else str(type(raw_data))
                )
                self.get_logger().warn(
                    f'Unrecognized occupancy grid map format: {keys}. '
                    'Skipping without blocking.'
                )
                self._warned_unknown_map_format = True
        return grid

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """Handle incoming navigation goal pose."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        self.get_logger().info(
            f'Received /goal_pose -> x={x:.2f}, y={y:.2f}, '
            f'yaw={math.degrees(yaw):.1f} deg'
        )
        success = self.client.send_goal(x, y, yaw)
        if success:
            self.get_logger().info('Goal successfully dispatched to E2.')
        else:
            self.get_logger().warn('Failed to dispatch goal to E2.')

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Handle 2D pose estimate for relocalization."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        self.get_logger().info(
            f'Received /initialpose -> x={x:.2f}, y={y:.2f}, '
            f'yaw={math.degrees(yaw):.1f} deg'
        )
        success = self.client.set_pose(x, y, yaw, mode='assist')
        if success:
            self.get_logger().info('Assisted pose set successfully.')
        else:
            self.get_logger().warn('Failed to set assisted pose.')

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Handle manual teleop cmd_vel with edge cancel and watchdog."""
        linear_val = msg.linear.x
        angular_val = msg.angular.z
        is_zero = (abs(linear_val) < 1e-4 and abs(angular_val) < 1e-4)

        should_cancel = False
        should_dispatch = False
        now = time.time()

        with self._cmd_lock:
            if is_zero:
                if self._last_cmd_nonzero:
                    self._last_cmd_nonzero = False
                    should_cancel = True
            else:
                self._last_cmd_nonzero = True
                self._last_nonzero_cmd_time = now
                if (
                    now - self._last_teleop_send_time
                ) >= self.teleop_min_interval:
                    self._last_teleop_send_time = now
                    should_dispatch = True

        if should_cancel:
            self.get_logger().info(
                'Teleop falling edge detected: canceling goal.'
            )
            self.client.cancel_goal()
            return

        if not should_dispatch:
            return

        # P1-3: Handle diagonal motion by executing dominant axis
        if abs(linear_val) >= 1e-4 and abs(angular_val) >= 1e-4:
            if not self._warned_diagonal_cmd:
                self.get_logger().info(
                    'E2 chassis separates rectilinear and rotation motion; '
                    'diagonal compound motion is not supported. '
                    'Executing dominant axis.'
                )
                self._warned_diagonal_cmd = True

            if abs(angular_val) > abs(linear_val):
                rad = math.copysign(self.teleop_step_rad, angular_val)
                vel = min(abs(angular_val), self.max_angular_vel)
                vel = max(vel, 0.1)
                self.client.move_rotation(rad, vel)
            else:
                dist = math.copysign(self.teleop_step_dist, linear_val)
                vel = min(abs(linear_val), self.max_linear_vel)
                vel = max(vel, 0.05)
                self.client.move_rectilinear(dist, vel)
            return

        if abs(linear_val) >= 1e-4:
            dist = math.copysign(self.teleop_step_dist, linear_val)
            vel = min(abs(linear_val), self.max_linear_vel)
            vel = max(vel, 0.05)
            self.client.move_rectilinear(dist, vel)
        elif abs(angular_val) >= 1e-4:
            rad = math.copysign(self.teleop_step_rad, angular_val)
            vel = min(abs(angular_val), self.max_angular_vel)
            vel = max(vel, 0.1)
            self.client.move_rotation(rad, vel)

    def _on_teleop_watchdog_timer(self) -> None:
        """Check teleop watchdog and auto-brake on command stream timeout."""
        now = time.time()
        should_cancel = False
        dt = 0.0
        with self._cmd_lock:
            if self._last_cmd_nonzero:
                dt = now - self._last_nonzero_cmd_time
                if dt > self.teleop_watchdog_sec:
                    self._last_cmd_nonzero = False
                    should_cancel = True

        if should_cancel:
            self.get_logger().warn(
                f'Teleop watchdog timeout ({dt:.2f}s > '
                f'{self.teleop_watchdog_sec:.2f}s): braking chassis.'
            )
            self.client.cancel_goal()

    def _on_pose_poll_timer(self) -> None:
        """Poll robot pose and publish odom and TF."""
        pose = self.client.get_pose()
        if pose is None:
            return

        x, y, yaw = pose
        now_msg = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def _on_action_poll_timer(self) -> None:
        """Poll motion action status and remaining path."""
        status_code = self.client.get_action_status()
        if status_code is not None:
            status_name = self.client.STATUS_NAMES.get(
                status_code, f'CODE_{status_code}'
            )
            msg = String()
            msg.data = status_name
            self.pub_action_status.publish(msg)

        points = self.client.get_remaining_path()
        if points is not None and isinstance(points, list):
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.odom_frame

            path_poses: List[PoseStamped] = []
            for pt in points:
                if isinstance(pt, dict):
                    px = float(pt.get('x', pt.get('X', 0.0)))
                    py = float(pt.get('y', pt.get('Y', 0.0)))
                elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    px = float(pt[0])
                    py = float(pt[1])
                else:
                    continue

                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = px
                ps.pose.position.y = py
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path_poses.append(ps)

            path_msg.poses = path_poses
            self.pub_plan.publish(path_msg)

    def _on_map_poll_timer(self) -> None:
        """Poll occupancy grid map and publish to /e2/map."""
        raw_map = self.client.get_occupancy_grid()
        if raw_map is None:
            return
        grid = self._parse_occupancy_grid(raw_map)
        if grid is not None:
            self._last_occupancy_grid = grid
            self.pub_map.publish(grid)

    def _srv_switch_to_mapping(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to switch E2 into continue mapping mode."""
        ok = self.client.set_slam_mode(0, cleanup_map=False)
        response.success = ok
        response.message = (
            'Switched to mapping mode' if ok else 'Failed to switch'
        )
        return response

    def _srv_start_new_mapping(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to start new mapping session (cleanupMap=True)."""
        ok = self.client.set_slam_mode(0, cleanup_map=True)
        self._last_occupancy_grid = None
        response.success = ok
        response.message = (
            'Started new mapping session (cleanupMap=True)'
            if ok else 'Failed to start new mapping session'
        )
        return response

    def _srv_switch_to_nav(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to switch E2 into navigation mode."""
        ok = self.client.set_slam_mode(1, cleanup_map=False)
        response.success = ok
        response.message = (
            'Switched to navigation mode' if ok else 'Failed to switch'
        )
        return response

    def _srv_cancel_nav(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to cancel navigation action."""
        ok = self.client.cancel_goal()
        response.success = ok
        response.message = (
            'Navigation canceled' if ok else 'Failed to cancel'
        )
        return response

    def _srv_save_map(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to save map on E2."""
        ok = self.client.save_map(self.default_map_name)
        response.success = ok
        response.message = (
            'Map saved successfully' if ok else 'Failed to save map'
        )
        return response

    def _srv_export_map(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service callback to export .map, .pgm, and .yaml files."""
        if self.map_export_dir:
            export_dir = os.path.expanduser(self.map_export_dir)
        else:
            export_dir = os.path.join(os.getcwd(), 'e2_maps')
        os.makedirs(export_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        base_name = f'{self.default_map_name}_{ts}'
        map_path = os.path.join(export_dir, f'{base_name}.map')

        # 1. Save on chassis
        ok_save = self.client.save_map(self.default_map_name)
        if not ok_save:
            self.get_logger().warn('Failed to save map on chassis')

        # 2. Download binary .map
        ok_download = self.client.download_map(map_path)

        # 3. Export raster .pgm + .yaml if cached
        grid = self._last_occupancy_grid
        if grid is None:
            raw_map = self.client.get_occupancy_grid()
            if raw_map is not None:
                grid = self._parse_occupancy_grid(raw_map)
                if grid is not None:
                    self._last_occupancy_grid = grid

        exported_files: List[str] = []
        if ok_download:
            exported_files.append(map_path)

        if grid is not None:
            pgm_path, yaml_path = save_occupancy_grid_to_pgm_yaml(
                grid, export_dir, base_name
            )
            exported_files.extend([pgm_path, yaml_path])

        if ok_download or grid is not None:
            response.success = True
            file_list_str = ', '.join(exported_files)
            response.message = (
                f'Successfully exported map files: {file_list_str}'
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                'Failed to export map '
                '(.map download failed and no raster grid)'
            )
            self.get_logger().error(response.message)

        return response


def main(args: Optional[Any] = None) -> None:
    """Entry point for E2BridgeNode with multi-threaded executor."""
    rclpy.init(args=args)
    node = E2BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
