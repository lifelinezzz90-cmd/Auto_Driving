"""Mock HTTP RESTful server simulating PadBot E2 chassis navigation API."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import sys
import threading
import time
from typing import Any, Dict, List


class MockE2State:
    """Thread-safe state container for mock E2 robot."""

    MAP_WIDTH = 160
    MAP_HEIGHT = 160
    MAP_RES = 0.05
    MAP_ORIGIN_X = -4.0
    MAP_ORIGIN_Y = -4.0

    def __init__(self) -> None:
        """Initialize mock robot state."""
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.slam_mode = 1  # 0: mapping, 1: navigation
        # Status: 1: SUCCEEDED (idle), 2: CANCELED, 4: EXECUTING
        self.action_status = 1
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.path: List[Dict[str, float]] = []
        self.speed = 0.3  # m/s
        self.rot_speed = 0.6  # rad/s
        self.running = True

        # Precompute realistic 8m x 8m room map
        self.ground_truth: List[int] = [0] * (
            self.MAP_WIDTH * self.MAP_HEIGHT
        )
        for y in range(self.MAP_HEIGHT):
            wy = self.MAP_ORIGIN_Y + (y + 0.5) * self.MAP_RES
            for x in range(self.MAP_WIDTH):
                wx = self.MAP_ORIGIN_X + (x + 0.5) * self.MAP_RES
                idx = y * self.MAP_WIDTH + x
                # Outer walls
                if (
                    x <= 2 or x >= (self.MAP_WIDTH - 3) or
                    y <= 2 or y >= (self.MAP_HEIGHT - 3)
                ):
                    self.ground_truth[idx] = 100
                # Pillar 1 at (1.5, 1.0) with radius 0.45m
                elif math.hypot(wx - 1.5, wy - 1.0) <= 0.45:
                    self.ground_truth[idx] = 100
                # Pillar 2 at (-1.5, -1.0) with radius 0.45m
                elif math.hypot(wx - (-1.5), wy - (-1.0)) <= 0.45:
                    self.ground_truth[idx] = 100
                else:
                    self.ground_truth[idx] = 0

        self.explored = bytearray(self.MAP_WIDTH * self.MAP_HEIGHT)
        self.update_exploration()

    def reset_exploration(self) -> None:
        """Reset explored area in mapping mode."""
        self.explored = bytearray(self.MAP_WIDTH * self.MAP_HEIGHT)
        self.update_exploration()

    def update_exploration(self, radius: float = 2.5) -> None:
        """Update explored cells around current robot pose."""
        radius_cells = int(radius / self.MAP_RES)
        cx = int((self.x - self.MAP_ORIGIN_X) / self.MAP_RES)
        cy = int((self.y - self.MAP_ORIGIN_Y) / self.MAP_RES)
        min_x = max(0, cx - radius_cells)
        max_x = min(self.MAP_WIDTH, cx + radius_cells + 1)
        min_y = max(0, cy - radius_cells)
        max_y = min(self.MAP_HEIGHT, cy + radius_cells + 1)
        r2 = radius * radius

        for y in range(min_y, max_y):
            wy = self.MAP_ORIGIN_Y + (y + 0.5) * self.MAP_RES
            for x in range(min_x, max_x):
                wx = self.MAP_ORIGIN_X + (x + 0.5) * self.MAP_RES
                if (wx - self.x) ** 2 + (wy - self.y) ** 2 <= r2:
                    self.explored[y * self.MAP_WIDTH + x] = 1

    def get_occupancy_grid_dict(self) -> Dict[str, Any]:
        """Return occupancy grid map dictionary."""
        with self.lock:
            if self.slam_mode == 0:
                # Mapping mode: dynamic exploration
                self.update_exploration()
                grid_data = [
                    self.ground_truth[i] if self.explored[i] else -1
                    for i in range(self.MAP_WIDTH * self.MAP_HEIGHT)
                ]
            else:
                # Navigation mode: full map
                grid_data = list(self.ground_truth)

            return {
                'width': self.MAP_WIDTH,
                'height': self.MAP_HEIGHT,
                'resolution': self.MAP_RES,
                'origin': {
                    'x': self.MAP_ORIGIN_X,
                    'y': self.MAP_ORIGIN_Y,
                    'yaw': 0.0,
                },
                'data': grid_data,
            }

    def update_motion(self, dt: float) -> None:
        """Update simulated motion trajectory towards goal."""
        with self.lock:
            if self.action_status != 4:
                return

            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dist = math.hypot(dx, dy)

            if dist > 0.03:
                step = min(dist, self.speed * dt)
                heading = math.atan2(dy, dx)
                self.x += step * math.cos(heading)
                self.y += step * math.sin(heading)
                if self.slam_mode == 0:
                    self.update_exploration()

                # Smoothly rotate towards heading
                dyaw = (heading - self.yaw + math.pi) % (2.0 * math.pi) \
                    - math.pi
                if abs(dyaw) > 0.05:
                    self.yaw += math.copysign(
                        min(abs(dyaw), self.rot_speed * dt), dyaw
                    )
                    self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) \
                        - math.pi

                # Update remaining path
                steps_count = min(10, max(2, int(dist / 0.1)))
                new_path: List[Dict[str, float]] = []
                for i in range(steps_count + 1):
                    alpha = i / float(steps_count)
                    px = self.x + alpha * dx
                    py = self.y + alpha * dy
                    new_path.append({'x': round(px, 3), 'y': round(py, 3)})
                self.path = new_path
            else:
                # Close to goal, rotate to target yaw
                dyaw = (self.goal_yaw - self.yaw + math.pi) % (2.0 * math.pi) \
                    - math.pi
                if abs(dyaw) > 0.05:
                    self.yaw += math.copysign(
                        min(abs(dyaw), self.rot_speed * dt), dyaw
                    )
                    self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) \
                        - math.pi
                else:
                    self.x = self.goal_x
                    self.y = self.goal_y
                    self.yaw = self.goal_yaw
                    self.action_status = 1  # SUCCEEDED
                    self.path = []


GLOBAL_STATE = MockE2State()


class MockE2RequestHandler(BaseHTTPRequestHandler):
    """HTTP Request handler implementing PadBot E2 RESTful endpoints."""

    def log_message(self, format: str, *args: Any) -> None:
        """Suppress default access logging to keep terminal clean."""
        pass

    def _send_json(self, data: Any, status: int = 200) -> None:
        """Send JSON response."""
        body = json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        """Read and parse JSON body."""
        content_len = int(self.headers.get('Content-Length', 0))
        if content_len > 0:
            raw = self.rfile.read(content_len).decode('utf-8')
            try:
                return json.loads(raw)
            except Exception:
                return {}
        return {}

    def do_GET(self) -> None:
        """Handle GET requests."""
        path = self.path.split('?')[0]

        if path == '/robot/api/v1.0/slam/mode':
            with GLOBAL_STATE.lock:
                mode_str = str(GLOBAL_STATE.slam_mode)
            self._send_json({'mode': mode_str})

        elif path == '/robot/api/v1.0/slam/pose':
            with GLOBAL_STATE.lock:
                x = GLOBAL_STATE.x
                y = GLOBAL_STATE.y
                yaw = GLOBAL_STATE.yaw
            self._send_json({'x': x, 'y': y, 'yaw': yaw})

        elif path == '/robot/api/v1.0/slam/map':
            dummy_map = b'PADBOT_E2_MOCK_MAP_BINARY_STREAM_DATA\x00\x01\x02'
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(dummy_map)))
            self.end_headers()
            self.wfile.write(dummy_map)

        elif path == '/robot/api/v1.0/slam/map/info':
            self._send_json({'mapName': 'mock_e2_map'})

        elif path == '/robot/api/v1.0/slam/map/occupancy-grid':
            grid_resp = GLOBAL_STATE.get_occupancy_grid_dict()
            self._send_json(grid_resp)

        elif path == '/robot/api/v1.0/motion/action':
            with GLOBAL_STATE.lock:
                status = GLOBAL_STATE.action_status
            self._send_json({'status': status})

        elif path == '/robot/api/v1.0/motion/action/path':
            with GLOBAL_STATE.lock:
                current_path = list(GLOBAL_STATE.path)
            self._send_json({'path': current_path})

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self) -> None:
        """Handle POST requests."""
        path = self.path.split('?')[0]
        data = self._read_json()

        if path == '/robot/api/v1.0/motion/action':
            gx = float(data.get('x', 0.0))
            gy = float(data.get('y', 0.0))
            gyaw = float(data.get('yaw', 0.0))
            with GLOBAL_STATE.lock:
                GLOBAL_STATE.goal_x = gx
                GLOBAL_STATE.goal_y = gy
                GLOBAL_STATE.goal_yaw = gyaw
                GLOBAL_STATE.action_status = 4  # EXECUTING
                GLOBAL_STATE.path = [{'x': gx, 'y': gy}]
            self._send_json({'status': 200, 'message': 'goal accepted'})

        elif path == '/robot/api/v1.0/motion/rectilinear':
            dist = float(data.get('dist', 0.0))
            with GLOBAL_STATE.lock:
                GLOBAL_STATE.x += dist * math.cos(GLOBAL_STATE.yaw)
                GLOBAL_STATE.y += dist * math.sin(GLOBAL_STATE.yaw)
                if GLOBAL_STATE.slam_mode == 0:
                    GLOBAL_STATE.update_exploration()
            self._send_json({'status': 200, 'message': 'rectilinear executed'})

        elif path == '/robot/api/v1.0/motion/rotation':
            radian = float(data.get('radian', 0.0))
            with GLOBAL_STATE.lock:
                GLOBAL_STATE.yaw += radian
                GLOBAL_STATE.yaw = (GLOBAL_STATE.yaw + math.pi) \
                    % (2.0 * math.pi) - math.pi
            self._send_json({'status': 200, 'message': 'rotation executed'})

        elif path == '/robot/api/v1.0/slam/map':
            self._send_json({'status': 200, 'message': 'map saved'})

        else:
            self.send_response(404)
            self.end_headers()

    def do_PUT(self) -> None:
        """Handle PUT requests."""
        path = self.path.split('?')[0]
        data = self._read_json()

        if path == '/robot/api/v1.0/slam/mode':
            mode = int(data.get('mode', 1))
            cleanup = data.get('cleanupMap', False)
            is_cleanup = False
            if isinstance(cleanup, bool):
                is_cleanup = cleanup
            elif isinstance(cleanup, str):
                is_cleanup = cleanup.lower() in ('true', '1')
            elif isinstance(cleanup, (int, float)):
                is_cleanup = bool(cleanup)

            with GLOBAL_STATE.lock:
                GLOBAL_STATE.slam_mode = mode
                if mode == 0 and is_cleanup:
                    GLOBAL_STATE.reset_exploration()
                elif mode == 0:
                    GLOBAL_STATE.update_exploration()
            self._send_json({'status': 200, 'mode': mode})

        elif path == '/robot/api/v1.0/slam/pose':
            with GLOBAL_STATE.lock:
                GLOBAL_STATE.x = float(data.get('x', GLOBAL_STATE.x))
                GLOBAL_STATE.y = float(data.get('y', GLOBAL_STATE.y))
                GLOBAL_STATE.yaw = float(data.get('yaw', GLOBAL_STATE.yaw))
            self._send_json({'status': 200, 'message': 'pose updated'})

        else:
            self.send_response(404)
            self.end_headers()

    def do_DELETE(self) -> None:
        """Handle DELETE requests."""
        path = self.path.split('?')[0]

        if path == '/robot/api/v1.0/motion/action':
            with GLOBAL_STATE.lock:
                GLOBAL_STATE.action_status = 2  # CANCELED
                GLOBAL_STATE.path = []
            self._send_json({'status': 200, 'message': 'action canceled'})

        else:
            self.send_response(404)
            self.end_headers()


def motion_loop() -> None:
    """Background motion loop at 10Hz."""
    last_time = time.time()
    while GLOBAL_STATE.running:
        time.sleep(0.1)
        now = time.time()
        dt = now - last_time
        last_time = now
        GLOBAL_STATE.update_motion(dt)


def main() -> None:
    """Launch mock server."""
    parser = argparse.ArgumentParser(description='Mock PadBot E2 Server')
    parser.add_argument('--host', default='127.0.0.1', help='Host IP')
    parser.add_argument('--port', type=int, default=8080, help='Port')
    args, _ = parser.parse_known_args(sys.argv[1:])

    motion_thread = threading.Thread(target=motion_loop, daemon=True)
    motion_thread.start()

    server_address = (args.host, args.port)
    httpd = ThreadingHTTPServer(server_address, MockE2RequestHandler)
    print(f'[MockE2] Listening on http://{args.host}:{args.port}...')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        GLOBAL_STATE.running = False
        httpd.server_close()


if __name__ == '__main__':
    main()
