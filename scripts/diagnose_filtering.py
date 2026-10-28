"""Detailed diagnostic: check what FAST-LIO's avia_handler filtering does to our data."""
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
import sys
import math

BAG_PATH = Path(sys.argv[1])

ros2_store = get_typestore(Stores.ROS2_JAZZY)

ros2_msgdef = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: livox_ros_driver2/msg/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""

ros2_types = get_types_from_msg(ros2_msgdef, "livox_ros_driver2/msg/CustomMsg")
ros2_store.register(ros2_types)

reader = Reader(BAG_PATH)
reader.open()

lidar_conn = next(c for c in reader.connections if c.topic == "/livox/lidar")

N_SCANS = 6
BLIND = 4.0
POINT_FILTER_NUM = 3

print(f"Bag: {BAG_PATH}")
print(f"Filters: N_SCANS={N_SCANS}, blind={BLIND}, point_filter_num={POINT_FILTER_NUM}")
print(f"{'='*80}\n")

msg_idx = 0
MAX_MSGS = 5

for conn, timestamp, rawdata in reader.messages(connections=[lidar_conn]):
    if msg_idx >= MAX_MSGS:
        break

    msg = ros2_store.deserialize_cdr(rawdata, conn.msgtype)
    pts = msg.points
    total = len(pts)

    # Count filter stages
    nonzero = 0
    pass_line = 0
    pass_tag = 0
    pass_blind = 0
    line_hist = {}
    tag_hist = {}

    for p in pts:
        if p.x != 0.0 or p.y != 0.0 or p.z != 0.0:
            nonzero += 1

        # line check
        if p.line < N_SCANS:
            pass_line += 1
        line_hist[p.line] = line_hist.get(p.line, 0) + 1

        # tag check
        tag_bits = p.tag & 0x30
        if tag_bits == 0x10 or tag_bits == 0x00:
            pass_tag += 1
        tag_hist[p.tag] = tag_hist.get(p.tag, 0) + 1

        # blind check
        dist_sq = p.x * p.x + p.y * p.y + p.z * p.z
        if dist_sq > BLIND * BLIND:
            pass_blind += 1

    stamp_sec = msg.header.stamp.sec
    stamp_nsec = msg.header.stamp.nanosec

    print(f"msg {msg_idx}: stamp={stamp_sec}.{stamp_nsec:09d}")
    print(f"  point_num={msg.point_num}, total_points={total}")
    print(f"  nonzero_xyz:   {nonzero}")
    print(f"  pass line<{N_SCANS}: {pass_line}")
    print(f"  pass tag:      {pass_tag}")
    print(f"  pass blind>{BLIND}m: {pass_blind}")
    print(f"  line histogram: {dict(sorted(line_hist.items()))}")

    # Show tag values sorted by frequency (top 5)
    tag_sorted = sorted(tag_hist.items(), key=lambda x: -x[1])[:5]
    print(f"  tag histogram (top5): {tag_sorted}")

    # Sample some points that pass blind
    samples = []
    for p in pts:
        dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        if dist > BLIND:
            samples.append((dist, p.x, p.y, p.z, p.line, p.tag, p.offset_time))
            if len(samples) >= 3:
                break
    if samples:
        print(f"  sample points beyond blind:")
        for d, x, y, z, ln, tg, ot in samples:
            print(f"    dist={d:.2f}m x={x:.3f} y={y:.3f} z={z:.3f} line={ln} tag={tg} offset_time={ot}")
    else:
        print(f"  NO points beyond blind distance!")
    print()

    msg_idx += 1

reader.close()
print(f"Checked {msg_idx} messages.")
