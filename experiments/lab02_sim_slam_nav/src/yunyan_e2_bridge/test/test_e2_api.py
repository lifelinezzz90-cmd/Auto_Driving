"""Automated integration test for PadBot E2 API client and mock server."""

import base64
from http.server import ThreadingHTTPServer
import io
import math
import os
import tempfile
import threading
import time

from PIL import Image
import pytest
from yunyan_e2_bridge.e2_api_client import E2ApiClient
from yunyan_e2_bridge.e2_bridge_node import (
    _parse_occupancy_grid,
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

    # Instantiation check for loader without starting ROS node loop
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
