"""PadBot E2 real machine RESTful interface probe and diagnostic tool."""

import argparse
import json
import sys
import time
from typing import Any, Dict, List

import requests


def run_probe(base_url: str) -> bool:
    """Probe all essential E2 RESTful API endpoints."""
    base_url = base_url.rstrip('/')
    endpoints = [
        ('/robot/api/v1.0/slam/mode', 'GET'),
        ('/robot/api/v1.0/slam/pose', 'GET'),
        ('/robot/api/v1.0/slam/map/info', 'GET'),
        ('/robot/api/v1.0/slam/map/occupancy-grid', 'GET'),
        ('/robot/api/v1.0/motion/action', 'GET'),
        ('/robot/api/v1.0/motion/action/path', 'GET'),
    ]

    print('=' * 70)
    print(f'[E2 Probe] Target base URL: {base_url}')
    print('=' * 70)

    dump_data: List[Dict[str, Any]] = []
    all_success = True
    session = requests.Session()

    for path, method in endpoints:
        url = f'{base_url}{path}'
        start_t = time.perf_counter()
        try:
            resp = session.request(method, url, timeout=2.0)
            elapsed_ms = (time.perf_counter() - start_t) * 1000.0
            status = resp.status_code

            try:
                content_json = resp.json()
                raw_preview = json.dumps(content_json, ensure_ascii=False)
            except Exception:
                content_json = None
                raw_preview = resp.text[:100]

            is_ok = (status == 200)
            if not is_ok:
                all_success = False

            flag = '✔ [OK]  ' if is_ok else '✘ [FAIL]'
            print(f'{flag} {method:4s} {path:<38s} | {status} | '
                  f'{elapsed_ms:6.1f} ms | {raw_preview[:40]}...')

            dump_data.append({
                'path': path,
                'method': method,
                'status_code': status,
                'latency_ms': round(elapsed_ms, 2),
                'response': (
                    content_json if content_json is not None
                    else resp.text[:200]
                ),
            })
        except Exception as e:
            all_success = False
            elapsed_ms = (time.perf_counter() - start_t) * 1000.0
            print(f'✘ [FAIL] {method:4s} {path:<38s} | ERR | '
                  f'{elapsed_ms:6.1f} ms | {str(e)}')
            dump_data.append({
                'path': path,
                'method': method,
                'status_code': -1,
                'error': str(e),
            })

    print('=' * 70)
    dump_path = '/tmp/e2_probe_dump.json'
    try:
        with open(dump_path, 'w', encoding='utf-8') as f:
            json.dump(dump_data, f, indent=2, ensure_ascii=False)
        print(f'[E2 Probe] Raw probe dump saved to: {dump_path}')
    except Exception as e:
        print(f'[E2 Probe] Failed to save dump: {e}')

    return all_success


def main() -> None:
    """Entry point for e2_probe CLI."""
    parser = argparse.ArgumentParser(description='PadBot E2 API Probe Tool')
    parser.add_argument(
        '--url',
        default='http://127.0.0.1:8080',
        help='E2 base URL (e.g. http://192.168.11.1:8080)',
    )
    args, _ = parser.parse_known_args(sys.argv[1:])

    success = run_probe(args.url)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
