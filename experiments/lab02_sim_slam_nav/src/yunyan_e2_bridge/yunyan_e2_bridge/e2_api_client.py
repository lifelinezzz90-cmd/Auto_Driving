"""PadBot E2 RESTful API Client for ROS 2 bridge."""

import json
import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import requests

logger = logging.getLogger('e2_api_client')


def _unwrap_data(payload: Any) -> Any:
    """Unwrap payload if nested under 'data' dict key."""
    if isinstance(payload, dict) and isinstance(payload.get('data'), dict):
        return payload['data']
    return payload


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
        """Initialize E2 API client with base URL and thread-local session."""
        self.base_url = base_url.rstrip('/')
        self.default_timeout = default_timeout
        self.poll_timeout = poll_timeout
        self.map_timeout = map_timeout
        self._local = threading.local()
        self._log_lock = threading.Lock()
        self._error_throttle: Dict[str, float] = {}
        self._has_error: Dict[str, bool] = {}

    @property
    def session(self) -> requests.Session:
        """Return thread-local requests Session instance."""
        if not hasattr(self._local, 'session'):
            self._local.session = requests.Session()
        return self._local.session

    def _log_error(self, endpoint: str, msg: str) -> None:
        """Throttle error logging to at most once per 5 seconds."""
        now = time.time()
        with self._log_lock:
            last = self._error_throttle.get(endpoint, 0.0)
            should_log = (now - last) >= 5.0
            if should_log:
                self._error_throttle[endpoint] = now
            self._has_error[endpoint] = True
        if should_log:
            logger.warning(f'[{endpoint}] {msg}')

    def _log_success(self, endpoint: str) -> None:
        """Log recovery message if endpoint was previously in error state."""
        with self._log_lock:
            was_in_error = self._has_error.get(endpoint, False)
            if was_in_error:
                self._has_error[endpoint] = False
        if was_in_error:
            logger.info(f'[{endpoint}] Interface recovered successfully.')

    def set_base_url(self, base_url: str) -> None:
        """Update base URL."""
        self.base_url = base_url.rstrip('/')

    def get_slam_mode(self) -> Optional[int]:
        """Get SLAM mode (0: mapping, 1: navigation)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/mode'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                self._log_success('get_slam_mode')
                data = _unwrap_data(resp.json())
                if isinstance(data, dict):
                    if 'mode' in data:
                        try:
                            return int(data['mode'])
                        except (ValueError, TypeError):
                            return None
                    return None
                if isinstance(data, int) and not isinstance(data, bool):
                    return data
                if isinstance(data, str) and data.strip().isdigit():
                    return int(data.strip())
                return None
            self._log_error('get_slam_mode', f'HTTP error: {resp.status_code}')
        except Exception as e:
            self._log_error('get_slam_mode', f'failed: {e}')
        return None

    def set_slam_mode(self, mode: int, cleanup_map: bool = False) -> bool:
        """Set SLAM mode (0: mapping, 1: navigation)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/mode'
        payload = {'mode': str(mode), 'cleanupMap': cleanup_map}
        try:
            resp = self.session.put(
                url, json=payload, timeout=self.default_timeout
            )
            if resp.status_code == 200:
                self._log_success('set_slam_mode')
                return True
            self._log_error(
                'set_slam_mode',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('set_slam_mode', f'failed: {e}')
            return False

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Get current pose (x, y, yaw)."""
        url = f'{self.base_url}/robot/api/v1.0/slam/pose'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                self._log_success('get_pose')
                data = _unwrap_data(resp.json())
                if not isinstance(data, dict):
                    return None
                raw_x = data.get('x', data.get('X'))
                raw_y = data.get('y', data.get('Y'))
                if raw_x is None or raw_y is None:
                    return None
                try:
                    x = float(raw_x)
                    y = float(raw_y)
                    yaw = float(data.get('yaw', data.get('Yaw', 0.0)))
                    return (x, y, yaw)
                except (ValueError, TypeError):
                    return None
            self._log_error('get_pose', f'HTTP error: {resp.status_code}')
        except Exception as e:
            self._log_error('get_pose', f'failed: {e}')
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
            if resp.status_code == 200:
                self._log_success('set_pose')
                return True
            self._log_error(
                'set_pose',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('set_pose', f'failed: {e}')
            return False

    def save_map(self, map_name: str) -> bool:
        """Save current SLAM map with map_name."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map'
        payload = {'mapName': map_name}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.map_timeout
            )
            if resp.status_code == 200:
                self._log_success('save_map')
                return True
            self._log_error(
                'save_map',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('save_map', f'failed: {e}')
            return False

    def download_map(self, save_path: str) -> bool:
        """Download binary .map file and save locally."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map'
        try:
            resp = self.session.get(url, timeout=self.map_timeout)
            if resp.status_code == 200:
                self._log_success('download_map')
                with open(save_path, 'wb') as f:
                    f.write(resp.content)
                return True
            self._log_error(
                'download_map', f'HTTP error: {resp.status_code}'
            )
        except Exception as e:
            self._log_error('download_map', f'failed: {e}')
        return False

    def get_occupancy_grid(self) -> Optional[Any]:
        """Get occupancy grid map data."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map/occupancy-grid'
        try:
            resp = self.session.get(url, timeout=self.default_timeout)
            if resp.status_code == 200:
                self._log_success('get_occupancy_grid')
                try:
                    return resp.json()
                except Exception:
                    return resp.text
            self._log_error(
                'get_occupancy_grid', f'HTTP error: {resp.status_code}'
            )
        except Exception as e:
            self._log_error('get_occupancy_grid', f'failed: {e}')
        return None

    def get_map_info(self) -> Optional[Dict[str, Any]]:
        """Get current map information."""
        url = f'{self.base_url}/robot/api/v1.0/slam/map/info'
        try:
            resp = self.session.get(url, timeout=self.default_timeout)
            if resp.status_code == 200:
                self._log_success('get_map_info')
                data = _unwrap_data(resp.json())
                return data if isinstance(data, dict) else None
            self._log_error('get_map_info', f'HTTP error: {resp.status_code}')
        except Exception as e:
            self._log_error('get_map_info', f'failed: {e}')
        return None

    def get_action_status(self) -> Optional[int]:
        """Get current motion action status."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                self._log_success('get_action_status')
                try:
                    data = resp.json()
                except Exception:
                    data = resp.text
                data = _unwrap_data(data)
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

                if isinstance(status_raw, int) and not isinstance(
                    status_raw, bool
                ):
                    return status_raw
                if isinstance(status_raw, str):
                    status_raw_str = status_raw.strip().upper()
                    for code, name in self.STATUS_NAMES.items():
                        if name == status_raw_str:
                            return code
                    if status_raw_str.isdigit():
                        return int(status_raw_str)

                logger.debug(
                    f'get_action_status: unrecognized value {status_raw}'
                )
                return None
            self._log_error(
                'get_action_status', f'HTTP error: {resp.status_code}'
            )
        except Exception as e:
            self._log_error('get_action_status', f'failed: {e}')
        return None

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        """Send navigation goal pose (x, y, yaw)."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        payload = {'x': x, 'y': y, 'yaw': yaw}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            if resp.status_code == 200:
                self._log_success('send_goal')
                return True
            self._log_error(
                'send_goal',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('send_goal', f'failed: {e}')
            return False

    def cancel_goal(self) -> bool:
        """Cancel current navigation goal with short timeout (1.0s)."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action'
        try:
            resp = self.session.delete(url, timeout=1.0)
            if resp.status_code == 200:
                self._log_success('cancel_goal')
                return True
            self._log_error(
                'cancel_goal',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('cancel_goal', f'failed: {e}')
            return False

    def get_remaining_path(self) -> Optional[List[Dict[str, float]]]:
        """Get remaining path for current action."""
        url = f'{self.base_url}/robot/api/v1.0/motion/action/path'
        try:
            resp = self.session.get(url, timeout=self.poll_timeout)
            if resp.status_code == 200:
                self._log_success('get_remaining_path')
                try:
                    data = resp.json()
                except Exception:
                    try:
                        data = json.loads(resp.text)
                    except Exception:
                        return None
                data = _unwrap_data(data)
                if isinstance(data, dict):
                    points = data.get('path', data.get('points', []))
                    if isinstance(points, str):
                        try:
                            points = json.loads(points)
                        except Exception:
                            points = []
                    if isinstance(points, list):
                        return points
                elif isinstance(data, list):
                    return data
                elif isinstance(data, str):
                    try:
                        points = json.loads(data)
                        if isinstance(points, list):
                            return points
                    except Exception:
                        pass
            return None
        except Exception as e:
            logger.debug(f'get_remaining_path failed: {e}')
            return None

    def move_rectilinear(
        self,
        dist: float,
        linear_vel: float,
        max_linear_vel: float = 0.6,
    ) -> bool:
        """Move rectilinear along heading with velocity clamp."""
        url = f'{self.base_url}/robot/api/v1.0/motion/rectilinear'
        vel = min(abs(linear_vel), max_linear_vel)
        vel = max(vel, 0.05)
        payload = {'dist': dist, 'linearVel': vel}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            if resp.status_code == 200:
                self._log_success('move_rectilinear')
                return True
            self._log_error(
                'move_rectilinear',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('move_rectilinear', f'failed: {e}')
            return False

    def move_rotation(
        self,
        radian: float,
        angular_vel: float,
        max_angular_vel: float = 1.0,
    ) -> bool:
        """Move rotation around z axis with velocity clamp."""
        url = f'{self.base_url}/robot/api/v1.0/motion/rotation'
        vel = min(abs(angular_vel), max_angular_vel)
        vel = max(vel, 0.1)
        payload = {'radian': radian, 'angularVel': vel}
        try:
            resp = self.session.post(
                url, json=payload, timeout=self.default_timeout
            )
            if resp.status_code == 200:
                self._log_success('move_rotation')
                return True
            self._log_error(
                'move_rotation',
                f'HTTP {resp.status_code}: {resp.text[:200]}'
            )
            return False
        except Exception as e:
            self._log_error('move_rotation', f'failed: {e}')
            return False
