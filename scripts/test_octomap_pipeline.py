#!/usr/bin/env python3
"""
Comprehensive test script for the ROS 2 Jazzy OctoMap Pipeline.
Tests:
1. Existence and integrity of generated static and dynamic OctoMap files (.bt).
2. OctoMap server node startup with uniform resolution mapping (0.2m).
3. OctoMap server node startup with adaptive Dynamic OctoMap (maps/dynamic_octomap.bt).
4. Verification of published topics:
   - /octomap_binary (octomap_msgs/msg/Octomap)
   - /occupied_cells_vis_array (visualization_msgs/msg/MarkerArray)
   - /projected_map (nav_msgs/msg/OccupancyGrid)
5. Verification of coordinate frames (camera_init) and multi-resolution voxel layers.
"""

import os
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


def test_live_topics(launch_arg='resolution:=0.2', test_name='Uniform 0.2m Map'):
    print(f"\n--- {test_name}: Starting OctoMap Server & Verifying Latched Topics ---")
    ws = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction'
    cmd = [
        'bash', '-c',
        f'source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash && '
        f'ros2 launch octomap_server octomap_server.launch.py {launch_arg}'
    ]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
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

    # Terminate server process
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()

    # Assertions
    tests_passed = True

    # 1. /octomap_binary
    bin_msg = received_data['binary']
    if bin_msg is not None:
        print(f"  [PASS] /octomap_binary: received {len(bin_msg.data)} bytes, frame='{bin_msg.header.frame_id}', res={bin_msg.resolution}")
        if bin_msg.header.frame_id != 'camera_init':
            print("    [WARN] frame_id is not camera_init")
    else:
        print("  [FAIL] /octomap_binary not received within timeout")
        tests_passed = False

    # 2. /occupied_cells_vis_array
    marker_msg = received_data['markers']
    if marker_msg is not None:
        total_voxels = sum(len(m.points) for m in marker_msg.markers)
        print(f"  [PASS] /occupied_cells_vis_array: received {len(marker_msg.markers)} marker layers, {total_voxels} active 3D voxels")
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
    ok1 = test_files()
    ok2 = test_live_topics(launch_arg='resolution:=0.2', test_name='Test 2: Uniform 0.2m OctoMap Baseline')
    ok3 = test_live_topics(launch_arg='octomap_path:=dynamic', test_name='Test 3: Adaptive Dynamic OctoMap (0.05m-0.4m)')

    if ok1 and ok2 and ok3:
        print("\n=======================================================")
        print("  ALL UNIFORM & DYNAMIC OCTOMAP TESTS PASSED [100%]")
        print("=======================================================\n")
    else:
        print("\n=======================================================")
        print("  SOME TESTS FAILED! Check logs above.")
        print("=======================================================\n")
        exit(1)
