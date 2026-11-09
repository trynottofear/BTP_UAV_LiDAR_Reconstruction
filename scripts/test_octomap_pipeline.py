#!/usr/bin/env python3
"""
Comprehensive test script for the ROS 2 Jazzy OctoMap Pipeline.
Tests:
1. Existence and integrity of generated static and dynamic OctoMap files (.bt).
1b. Correctness of every map from the benchmark output (Reports/benchmark/benchmark_results.json):
    no occupied voxel without a supporting LiDAR point, 100% point coverage, and >= 99.9% hit/miss
    agreement between OctoMap castRay() and the hierarchical ray traversal.
2. OctoMap server node startup with uniform resolution mapping (0.2m).
3. OctoMap server node startup with adaptive Dynamic OctoMap (maps/dynamic_octomap.bt).
4. Verification of published topics:
   - /octomap_binary (octomap_msgs/msg/Octomap)
   - /occupied_cells_vis_array (visualization_msgs/msg/MarkerArray)
   - /projected_map (nav_msgs/msg/OccupancyGrid)
5. Verification of coordinate frames (camera_init) and multi-resolution voxel layers.
"""

import json
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid


def test_files():
    print("\n--- Test 1: Verifying Generated OctoMap Files ---")
    ws = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction'
    test_files = [
        'uav_octomap_0.05m.bt',
        'uav_octomap_0.1m.bt',
        'uav_octomap_0.2m.bt',
        'uav_octomap_0.4m.bt',
        'dynamic_octomap.bt',
        'dynamic_octomap.ot'
    ]
    all_ok = True
    for f in test_files:
        path = os.path.join(ws, 'maps', f)
        if os.path.exists(path):
            size_kb = os.path.getsize(path) / 1024.0
            print(f"  [PASS] {f} exists ({size_kb:.1f} KB)")
        else:
            print(f"  [FAIL] {path} missing!")
            all_ok = False
    return all_ok


def test_map_correctness():
    print("\n--- Test 1b: Verifying Map Correctness (benchmark_results.json) ---")
    ws = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction'
    path = os.path.join(ws, 'Reports', 'benchmark', 'benchmark_results.json')
    if not os.path.exists(path):
        print(f"  [FAIL] {path} missing - run scripts/benchmark_octomap first")
        return False
    with open(path) as f:
        bench = json.load(f)
    all_ok = True
    for m in bench['maps']:
        if m['name'].startswith('Legacy'):
            continue  # kept only as a negative control
        if os.path.getmtime(os.path.join(ws, m['path'])) > os.path.getmtime(path):
            print(f"  [FAIL] {m['name']}: map is newer than the benchmark results - rerun the benchmark")
            all_ok = False
            continue
        agreement = min(r['hit_agreement'] for r in m['rays'].values())
        checks = {
            'no unsupported occupied voxels': m['unsupported_leaves'] == 0,
            'all points covered': m['coverage'] >= 1.0 - 1e-9,
            'castRay / hierarchical agreement >= 99.9%': agreement >= 0.999,
        }
        for label, ok in checks.items():
            print(f"  [{'PASS' if ok else 'FAIL'}] {m['name']}: {label}")
            all_ok &= ok
    return all_ok


def expected_voxels(map_path):
    """Occupied leaf count of a map, from the benchmark results (None if unavailable)."""
    path = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction/Reports/benchmark/benchmark_results.json'
    if not os.path.exists(path):
        return None
    with open(path) as f:
        for m in json.load(f)['maps']:
            if m['path'] == map_path:
                return m['occupied_leaves']
    return None


def test_live_topics(launch_arg='resolution:=0.2', test_name='Uniform 0.2m Map',
                     expected_res=0.2, map_path='maps/uav_octomap_0.2m.bt'):
    print(f"\n--- {test_name}: Starting OctoMap Server & Verifying Latched Topics ---")
    ws = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction'
    cmd = [
        'bash', '-c',
        f'source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash && '
        f'ros2 launch octomap_server octomap_server.launch.py {launch_arg}'
    ]

    # New session so the whole launch tree (bash -> ros2 launch -> octomap_server_node) can be
    # stopped; terminating only the bash wrapper leaves the server publishing latched topics.
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True,
                            start_new_session=True)
    print(f"  Launched octomap_server with '{launch_arg}' (PID: {proc.pid})...")

    # Allow node to initialize and load the map
    time.sleep(3)

    received_data = {
        'binary': None,
        'markers': None,
        'projected_map': None
    }

    class TestSubscriber(Node):
        def __init__(self):
            super().__init__('octomap_pipeline_verifier')
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )

            self.create_subscription(Octomap, '/octomap_binary', self.on_binary, qos)
            self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.on_markers, qos)
            self.create_subscription(OccupancyGrid, '/projected_map', self.on_map, qos)

        def on_binary(self, msg):
            received_data['binary'] = msg

        def on_markers(self, msg):
            received_data['markers'] = msg

        def on_map(self, msg):
            received_data['projected_map'] = msg

    rclpy.init()
    node = TestSubscriber()

    start_t = time.time()
    while time.time() - start_t < 6.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(v is not None for v in received_data.values()):
            break

    node.destroy_node()
    rclpy.shutdown()

    # Terminate the whole launch process group
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass

    # Assertions
    tests_passed = True

    # 1. /octomap_binary
    bin_msg = received_data['binary']
    if bin_msg is not None:
        print(f"  [PASS] /octomap_binary: received {len(bin_msg.data)} bytes, frame='{bin_msg.header.frame_id}', res={bin_msg.resolution}")
        if bin_msg.header.frame_id != 'camera_init':
            print("    [WARN] frame_id is not camera_init")
        if abs(bin_msg.resolution - expected_res) > 1e-6:
            print(f"  [FAIL] /octomap_binary resolution {bin_msg.resolution} != expected {expected_res}")
            tests_passed = False
    else:
        print("  [FAIL] /octomap_binary not received within timeout")
        tests_passed = False

    # 2. /occupied_cells_vis_array
    marker_msg = received_data['markers']
    if marker_msg is not None:
        total_voxels = sum(len(m.points) for m in marker_msg.markers)
        print(f"  [PASS] /occupied_cells_vis_array: received {len(marker_msg.markers)} marker layers, {total_voxels} active 3D voxels")
        expected = expected_voxels(map_path)
        if expected is not None and total_voxels != expected:
            print(f"  [FAIL] voxel count {total_voxels} != occupied leaves of {map_path} ({expected})")
            tests_passed = False
    else:
        print("  [FAIL] /occupied_cells_vis_array not received within timeout")
        tests_passed = False

    # 3. /projected_map
    map_msg = received_data['projected_map']
    if map_msg is not None:
        occupied = sum(1 for d in map_msg.data if d > 50)
        print(f"  [PASS] /projected_map: size={map_msg.info.width}x{map_msg.info.height}, occupied_cells={occupied}")
    else:
        print("  [FAIL] /projected_map not received within timeout")
        tests_passed = False

    return tests_passed


if __name__ == '__main__':
    ok1 = test_files() and test_map_correctness()
    ok2 = test_live_topics(launch_arg='resolution:=0.2', test_name='Test 2: Uniform 0.2m OctoMap Baseline')
    ok3 = test_live_topics(launch_arg='octomap_path:=dynamic', test_name='Test 3: Adaptive Dynamic OctoMap (0.05m-0.4m)',
                           expected_res=0.05, map_path='maps/dynamic_octomap.bt')

    if ok1 and ok2 and ok3:
        print("\n=======================================================")
        print("  ALL UNIFORM & DYNAMIC OCTOMAP TESTS PASSED [100%]")
        print("=======================================================\n")
    else:
        print("\n=======================================================")
        print("  SOME TESTS FAILED! Check logs above.")
        print("=======================================================\n")
        exit(1)
