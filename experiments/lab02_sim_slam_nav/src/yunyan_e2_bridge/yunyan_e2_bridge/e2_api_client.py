"""PadBot E2 RESTful API Client for ROS 2 bridge."""

import logging
from typing import Any, Dict, List, Optional, Tuple

import requests

logger = logging.getLogger('e2_api_client')


class E2ApiClient:
    """HTTP RESTful client for PadBot E2 navigation module."""

    STATUS_UNKNOWN = 0
    STATUS_SUCCEEDED = 1
    STATUS_CANCELED = 2
    STATUS_FAILED = 3
    STATUS_EXECUTING = 4

    STATUS_NAMES = {
        STATUS_UNKNOWN: 'UNKNOWN',
        STATUS_SUCCEEDED: 'SUCCEEDED',
        STATUS_CANCELED: 'CANCELED',
        STATUS_FAILED: 'FAILED',
        STATUS_EXECUTING: 'EXECUTING',
    }

    def __init__(
        self,
        base_url: str = 'http://127.0.0.1:8080',
        default_timeout: float = 2.0,
        poll_timeout: float = 0.5,
        map_timeout: float = 5.0,
    ) -> None:
        """Initialize E2 API client with base URL and session."""
        self.base_url = base_url.rstrip('/')
        self.default_timeout = default_timeout
        self.poll_timeout = poll_timeout
        self.map_timeout = map_timeout
        self.session = requests.Session()

    def set_base_url(self, base_url: str) -> None:
        """Update base URL."""
        self.base_url = base_url.rstrip('/')

    def get_slam_mode(self) -> Optional[int]:
        """Get SLAM mode (0: mapping, 1: navigation)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/mode'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                data = resp.json()
                if isinstance(data, dict):
                    mode_val = data.get('mode', 0)
                else:
                    mode_val = data
                return int(mode_val)
            logger.warning(f'get_slam_mode HTTP error: {resp.status_code}')
        except Exception as e:
            logger.warning(f'get_slam_mode failed: {e}')
        return None

    def set_slam_mode(self, mode: int, cleanup_map: bool = False) -> bool:
        """Set SLAM mode (0: mapping, 1: navigation)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/mode'
        payload = {'mode': str(mode), 'cleanupMap': cleanup_map}
        try:
            resp = self.session.put(
                url, json=payload, timeout=self.default_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'set_slam_mode failed: {e}')
            return False

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Get current pose (x, y, yaw)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/pose'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                data = resp.json()
                if not isinstance(data, dict):
                    return None
                x = float(data.get('x', data.get('X', 0.0)))
                y = float(data.get('y', data.get('Y', 0.0)))
                yaw = float(data.get('yaw', data.get('Yaw', 0.0)))
                return (x, y, yaw)
            logger.warning(f'get_pose HTTP error: {resp.status_code}')
        except Exception as e:
            logger.warning(f'get_pose failed: {e}')
        return None

    def set_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        mode: str = 'assist',
    ) -> bool:
        """Set pose or assist localization."""
        url = f'{self.base_url}/robot/api/v1.0/slam/pose'
        payload = {'mode': mode, 'x': x, 'y': y, 'yaw': yaw}
        try:
            resp = self.session.put(
                url, json=payload, timeout=self.default_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'set_pose failed: {e}')
            return False

    def save_map(self, map_name: str) -> bool:
        """Save current SLAM map with map_name."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map'
        payload = {'mapName': map_name}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.map_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'save_map failed: {e}')
            return False

    def download_map(self, save_path: str) -> bool:
        """Download binary .map file and save locally."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map'
        try:
            resp = self.session.get(url, timeout=self.map_timeout)
            if resp.status_code == 200:
                with open(save_path, 'wb') as f:
                    f.write(resp.content)
                return True
            logger.warning(f'download_map HTTP error: {resp.status_code}')
        except Exception as e:
            logger.warning(f'download_map failed: {e}')
        return False

    def get_occupancy_grid(self) -> Optional[Any]:
        """Get occupancy grid map data."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map/occupancy-grid'
        try:
            resp = self.session.get(url, timeout=self.default_timeout)
            if resp.status_code == 200:
                try:
                    return resp.json()
                except Exception:
                    return resp.text
            logger.warning(
                f'get_occupancy_grid HTTP error: {resp.status_code}'
            )
        except Exception as e:
            logger.warning(f'get_occupancy_grid failed: {e}')
        return None

    def get_map_info(self) -> Optional[Dict[str, Any]]:
        """Get current map information."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map/info'
        try:
            resp = self.session.get(url, timeout=self.default_timeout)
            if resp.status_code == 200:
                return resp.json()
            logger.warning(f'get_map_info HTTP error: {resp.status_code}')
        except Exception as e:
            logger.warning(f'get_map_info failed: {e}')
        return None

    def get_action_status(self) -> Optional[int]:
        """Get current motion action status."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                data = resp.json()
                if isinstance(data, dict):
                    status_raw = data.get(
                        'status', data.get('action', data.get('state', None))
                    )
                    if status_raw is None:
                        for v in data.values():
                            status_raw = v
                            break
                else:
                    status_raw = data

                if isinstance(status_raw, int):
                    return status_raw
                if isinstance(status_raw, str):
                    status_raw_upper = status_raw.upper()
                    for code, name in self.STATUS_NAMES.items():
                        if name == status_raw_upper:
                            return code
                    if status_raw.isdigit():
                        return int(status_raw)
            logger.warning(f'get_action_status HTTP error: {resp.status_code}')
        except Exception as e:
            logger.warning(f'get_action_status failed: {e}')
        return None

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        """Send navigation goal pose (x, y, yaw)."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        payload = {'x': x, 'y': y, 'yaw': yaw}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'send_goal failed: {e}')
            return False

    def cancel_goal(self) -> bool:
        """Cancel current navigation goal."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        try:
            resp = self.session.delete(url, timeout=self.default_timeout)
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'cancel_goal failed: {e}')
            return False

    def get_remaining_path(self) -> Optional[List[Dict[str, float]]]:
        """Get remaining path for current action."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action/path'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                data = resp.json()
                if isinstance(data, dict):
                    points = data.get('path', data.get('points', []))
                    if isinstance(points, list):
                        return points
                elif isinstance(data, list):
                    return data
            return None
        except Exception as e:
            logger.debug(f'get_remaining_path failed: {e}')
            return None

    def move_rectilinear(self, dist: float, linear_vel: float) -> bool:
        """Move rectilinear along heading."""
        url = f'{self.base_url}/robot/api/v1.0/motion/rectilinear'
        payload = {'dist': dist, 'linearVel': linear_vel}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'move_rectilinear failed: {e}')
            return False

    def move_rotation(self, radian: float, angular_vel: float) -> bool:
        """Move rotation around z axis."""
        url = f'{self.base_url}/robot/api/v1.0/motion/rotation'
        payload = {'radian': radian, 'angularVel': angular_vel}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f'move_rotation failed: {e}')
            return False
