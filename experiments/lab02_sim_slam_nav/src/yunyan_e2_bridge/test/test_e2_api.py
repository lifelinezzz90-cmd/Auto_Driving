"""Automated integration test for PadBot E2 API client and mock server."""

import base64
from http.server import ThreadingHTTPServer
import io
import math
import os
import tempfile
import threading
import time
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import Twist
from PIL import Image
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from yunyan_e2_bridge.e2_api_client import E2ApiClient
from yunyan_e2_bridge.e2_bridge_node import (
    _parse_occupancy_grid,
    E2BridgeNode,
    save_occupancy_grid_to_pgm_yaml,
)
from yunyan_e2_bridge.e2_mock_server import (
    GLOBAL_STATE,
    MockE2RequestHandler,
    motion_loop,
)
from yunyan_e2_bridge.e2_probe import run_probe
from yunyan_e2_bridge.e2_waypoint_patrol import (
    E2WaypointPatrolNode,
    yaw_to_quaternion,
)


@pytest.fixture(scope='module')
def mock_server():
    """Start mock E2 server on port 8899 for tests."""
    server_address = ('127.0.0.1', 8899)
    httpd = ThreadingHTTPServer(server_address, MockE2RequestHandler)
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    motion_thread = threading.Thread(target=motion_loop, daemon=True)
    motion_thread.start()

    time.sleep(0.1)
    yield 'http://127.0.0.1:8899'

    GLOBAL_STATE.running = False
    httpd.server_close()


def test_slam_mode(mock_server):
    """Test get and set SLAM mode including cleanupMap."""
    client = E2ApiClient(base_url=mock_server)
    assert client.set_slam_mode(0, cleanup_map=False) is True
    assert client.get_slam_mode() == 0

    assert client.set_slam_mode(0, cleanup_map=True) is True
    assert client.get_slam_mode() == 0

    assert client.set_slam_mode(1) is True
    assert client.get_slam_mode() == 1


def test_pose_get_and_set(mock_server):
    """Test get and set robot pose."""
    client = E2ApiClient(base_url=mock_server)
    assert client.set_pose(1.5, -2.0, 0.785) is True

    pose = client.get_pose()
    assert pose is not None
    x, y, yaw = pose
    assert math.isclose(x, 1.5, abs_tol=1e-3)
    assert math.isclose(y, -2.0, abs_tol=1e-3)
    assert math.isclose(yaw, 0.785, abs_tol=1e-3)


def test_map_apis(mock_server):
    """Test map saving, info, occupancy grid, and download."""
    client = E2ApiClient(base_url=mock_server)
    assert client.save_map('test_map') is True

    info = client.get_map_info()
    assert isinstance(info, dict)
    assert 'mapName' in info

    grid = client.get_occupancy_grid()
    assert isinstance(grid, dict)
    assert 'width' in grid or 'map' in grid

    with tempfile.NamedTemporaryFile(delete=False) as f:
        tmp_name = f.name
    try:
        assert client.download_map(tmp_name) is True
        assert os.path.getsize(tmp_name) > 0
    finally:
        if os.path.exists(tmp_name):
            os.remove(tmp_name)


def test_occupancy_grid_parsing_and_mock(mock_server):
    """Test mock dynamic grid and 3-format parsing."""
    client = E2ApiClient(base_url=mock_server)

    # 1. Format 1: Live mock server grid dictionary
    raw_grid = client.get_occupancy_grid()
    assert isinstance(raw_grid, dict)
    assert raw_grid.get('width') == 160
    assert raw_grid.get('height') == 160
    assert len(raw_grid.get('data', [])) == 25600

    parsed_grid = _parse_occupancy_grid(raw_grid)
    assert parsed_grid is not None
    assert parsed_grid.info.width == 160
    assert parsed_grid.info.height == 160
    assert math.isclose(parsed_grid.info.resolution, 0.05, abs_tol=1e-4)
    assert len(parsed_grid.data) == 25600

    # 2. Format 2: Base64-encoded PNG image
    test_img = Image.new('L', (12, 12), color=255)
    buf = io.BytesIO()
    test_img.save(buf, format='PNG')
    b64_data = base64.b64encode(buf.getvalue()).decode('ascii')

    b64_dict = {'image': b64_data, 'resolution': 0.05}
    b64_parsed = _parse_occupancy_grid(b64_dict)
    assert b64_parsed is not None
    assert b64_parsed.info.width == 12
    assert b64_parsed.info.height == 12
    assert len(b64_parsed.data) == 144
    assert b64_parsed.data[0] == 0  # 255 free space mapped to 0

    # 3. Format 3: Unknown format fallback
    invalid_dict = {'unknown_prop': 'garbage', 'foo': 123}
    assert _parse_occupancy_grid(invalid_dict) is None
    assert _parse_occupancy_grid(None) is None


def test_map_export_pgm_and_yaml(tmp_path, mock_server):
    """Test pure Python P5 PGM and YAML export with Y-axis flip."""
    client = E2ApiClient(base_url=mock_server)
    raw_grid = client.get_occupancy_grid()
    grid_msg = _parse_occupancy_grid(raw_grid)
    assert grid_msg is not None

    export_dir = str(tmp_path)
    base_name = 'test_real_scene'
    pgm_path, yaml_path = save_occupancy_grid_to_pgm_yaml(
        grid_msg, export_dir, base_name
    )

    assert os.path.exists(pgm_path)
    assert os.path.exists(yaml_path)

    # Validate PGM header & binary body
    with open(pgm_path, 'rb') as f:
        content = f.read()
    assert content.startswith(b'P5\n160 160\n255\n')
    header_end = content.find(b'\n255\n') + len(b'\n255\n')
    body = content[header_end:]
    assert len(body) == 160 * 160

    # Validate YAML content
    with open(yaml_path, 'r', encoding='utf-8') as f:
        yaml_text = f.read()
    assert 'image: test_real_scene.pgm' in yaml_text
    assert 'mode: trinary' in yaml_text
    assert 'resolution: 0.05' in yaml_text
    assert 'negate: 0' in yaml_text
    assert 'occupied_thresh: 0.65' in yaml_text
    assert 'free_thresh: 0.25' in yaml_text


def test_waypoint_patrol_config_and_helpers(tmp_path):
    """Test waypoint patrol YAML loading and math helper."""
    qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2.0)
    assert math.isclose(qw, math.cos(math.pi / 4.0), abs_tol=1e-4)
    assert math.isclose(qz, math.sin(math.pi / 4.0), abs_tol=1e-4)

    test_yaml = tmp_path / 'custom_waypoints.yaml'
    test_yaml.write_text(
        'loops: 2\n'
        'goal_timeout_sec: 30.0\n'
        'dwell_sec: 1.5\n'
        'retry_on_fail: 2\n'
        'waypoints:\n'
        '  - name: test_pt\n'
        '    x: 2.0\n'
        '    y: 3.0\n'
        '    yaw: 1.57\n'
    )

    loader = E2WaypointPatrolNode._load_yaml_config
    cfg = loader(None, str(test_yaml))
    assert cfg.get('loops') == 2
    assert cfg.get('goal_timeout_sec') == 30.0
    assert cfg.get('dwell_sec') == 1.5
    assert cfg.get('retry_on_fail') == 2
    assert len(cfg.get('waypoints', [])) == 1
    assert cfg['waypoints'][0]['name'] == 'test_pt'


def test_navigation_action_and_cancel(mock_server):
    """Test dispatching goal, checking status, path, and canceling."""
    client = E2ApiClient(base_url=mock_server)
    assert client.send_goal(3.0, 4.0, 0.0) is True

    status = client.get_action_status()
    assert status == E2ApiClient.STATUS_EXECUTING

    path = client.get_remaining_path()
    assert isinstance(path, list)
    assert len(path) > 0

    assert client.cancel_goal() is True
    status = client.get_action_status()
    assert status == E2ApiClient.STATUS_CANCELED


def test_teleop_motion(mock_server):
    """Test rectilinear and rotation motion."""
    client = E2ApiClient(base_url=mock_server)
    client.set_pose(0.0, 0.0, 0.0)

    assert client.move_rectilinear(0.5, 0.2) is True
    pose = client.get_pose()
    assert pose is not None
    assert math.isclose(pose[0], 0.5, abs_tol=1e-2)

    assert client.move_rotation(1.57, 0.5) is True
    pose = client.get_pose()
    assert pose is not None
    assert math.isclose(pose[2], 1.57, abs_tol=1e-2)


def test_probe_utility(mock_server):
    """Test e2_probe runs against mock server with 200 OK."""
    ok = run_probe(mock_server)
    assert ok is True
    assert os.path.exists('/tmp/e2_probe_dump.json')


def test_waypoint_patrol_missing_waypoints_abort(tmp_path):
    """Test patrol node aborts and never falls back to default waypoint."""
    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    # 1. Non-existent file aborts without publishing goal
    node_missing = E2WaypointPatrolNode(
        parameter_overrides=[
            Parameter(
                'waypoints_file',
                Parameter.Type.STRING,
                '/tmp/non_existent_waypoints_12345.yaml',
            )
        ]
    )
    assert node_missing.init_failed is True
    assert node_missing.state == 'DONE'
    assert len(node_missing.waypoints) == 0
    assert node_missing.is_done() is True
    node_missing._state_step()
    assert node_missing.state == 'DONE'
    node_missing.destroy_node()

    # 2. Existing file with empty waypoints list
    empty_file = tmp_path / 'empty_wp.yaml'
    empty_file.write_text('loops: 1\nwaypoints: []\n')
    node_empty = E2WaypointPatrolNode(
        parameter_overrides=[
            Parameter(
                'waypoints_file',
                Parameter.Type.STRING,
                str(empty_file),
            )
        ]
    )
    assert node_empty.init_failed is True
    assert node_empty.state == 'DONE'
    assert len(node_empty.waypoints) == 0
    node_empty.destroy_node()


def test_api_client_data_unwrap_and_validation():
    """Test _unwrap_data, missing x/y, and boolean rejection."""
    client = E2ApiClient(base_url='http://127.0.0.1:9999')

    # 1. Test get_slam_mode with nested data
    mock_resp = MagicMock()
    mock_resp.status_code = 200
    mock_resp.json.return_value = {'data': {'mode': 1}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_slam_mode() == 1

    # Missing mode key returns None, NOT 0
    mock_resp.json.return_value = {'data': {'other_key': 123}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_slam_mode() is None

    # 2. Test get_pose with nested data and missing coordinates
    mock_resp.json.return_value = {'data': {'x': 2.5, 'y': -3.0, 'yaw': 1.2}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        pose = client.get_pose()
        assert pose == (2.5, -3.0, 1.2)

    # Missing x or y MUST return None, NOT (0.0, 0.0, 0.0)
    mock_resp.json.return_value = {'data': {'x': 2.5}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_pose() is None

    mock_resp.json.return_value = {'data': {'yaw': 1.2}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_pose() is None

    # Non-convertible float returns None
    mock_resp.json.return_value = {'data': {'x': 'invalid', 'y': 1.0}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_pose() is None

    # 3. Test get_action_status excludes bool
    mock_resp.json.return_value = {'data': {'status': True}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_action_status() is None

    mock_resp.json.return_value = {'data': {'status': False}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_action_status() is None

    # Valid status string
    mock_resp.json.return_value = {'data': {'status': 'SUCCEEDED'}}
    with patch.object(client.session, 'get', return_value=mock_resp):
        assert client.get_action_status() == E2ApiClient.STATUS_SUCCEEDED

    # 4. Test get_remaining_path with nested points / path
    mock_resp.json.return_value = {
        'data': {'points': [{'x': 1.0, 'y': 2.0}]}
    }
    with patch.object(client.session, 'get', return_value=mock_resp):
        path = client.get_remaining_path()
        assert path == [{'x': 1.0, 'y': 2.0}]

    mock_resp.json.return_value = {
        'data': {'path': '[{"x": 3.0, "y": 4.0}]'}
    }
    with patch.object(client.session, 'get', return_value=mock_resp):
        path = client.get_remaining_path()
        assert path == [{'x': 3.0, 'y': 4.0}]


def test_cmd_vel_watchdog_timeout():
    """Test cmd_vel watchdog triggers cancel_goal upon timeout."""
    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    node = E2BridgeNode(
        parameter_overrides=[
            Parameter('teleop_watchdog_sec', Parameter.Type.DOUBLE, 0.1),
            Parameter('publish_tf', Parameter.Type.BOOL, False),
        ]
    )
    node.client = MagicMock(spec=E2ApiClient)

    # Simulate non-zero cmd_vel
    twist = Twist()
    twist.linear.x = 0.2
    node._on_cmd_vel(twist)

    with node._cmd_lock:
        assert node._last_cmd_nonzero is True

    # Sleep past watchdog interval
    time.sleep(0.15)
    node._on_teleop_watchdog_timer()

    with node._cmd_lock:
        assert node._last_cmd_nonzero is False

    node.client.cancel_goal.assert_called_once()
    node.destroy_node()


def test_motion_velocity_clamping():
    """Test low-level velocity limit clamps in E2ApiClient."""
    client = E2ApiClient(base_url='http://127.0.0.1:9999')
    mock_resp = MagicMock()
    mock_resp.status_code = 200

    with patch.object(client.session, 'post', return_value=mock_resp) as p:
        # Request 1.5 m/s, clamped to max_linear_vel (0.6 m/s)
        client.move_rectilinear(
            dist=1.0, linear_vel=1.5, max_linear_vel=0.6
        )
        _, kwargs = p.call_args
        assert kwargs['json']['linearVel'] == 0.6

        # Request 2.0 rad/s, clamped to max_angular_vel (1.0 rad/s)
        client.move_rotation(
            radian=1.57, angular_vel=2.0, max_angular_vel=1.0
        )
        _, kwargs = p.call_args
        assert kwargs['json']['angularVel'] == 1.0


def test_base64_grid_origin_parsing():
    """Test base64 grid parsing preserves custom origin dict/list."""
    test_img = Image.new('L', (4, 4), color=255)
    buf = io.BytesIO()
    test_img.save(buf, format='PNG')
    b64_data = base64.b64encode(buf.getvalue()).decode('ascii')

    raw_dict = {
        'image': b64_data,
        'resolution': 0.05,
        'origin': {'x': -1.5, 'y': -2.5, 'yaw': 0.785},
    }
    grid = _parse_occupancy_grid(raw_dict)
    assert grid is not None
    assert math.isclose(grid.info.origin.position.x, -1.5, abs_tol=1e-3)
    assert math.isclose(grid.info.origin.position.y, -2.5, abs_tol=1e-3)


def test_api_client_error_throttling_and_422():
    """Test 422 status logging and 5.0s per-interface log throttling."""
    client = E2ApiClient(base_url='http://127.0.0.1:9999')
    mock_resp = MagicMock()
    mock_resp.status_code = 422
    mock_resp.text = '{"detail": "Unprocessable Entity"}'

    with patch.object(client.session, 'post', return_value=mock_resp):
        with patch(
            'yunyan_e2_bridge.e2_api_client.logger.warning'
        ) as mock_warn:
            # First failure logs warning
            ok = client.send_goal(1.0, 2.0, 0.0)
            assert ok is False
            mock_warn.assert_called_once()
            assert '422' in mock_warn.call_args[0][0]

            # Immediate second failure is throttled (no duplicate log)
            mock_warn.reset_mock()
            ok = client.send_goal(1.0, 2.0, 0.0)
            assert ok is False
            mock_warn.assert_not_called()
