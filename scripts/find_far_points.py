"""Find which messages have points beyond blind=4.0m."""
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
import sys
import math

BAG_PATH = Path(sys.argv[1])
BLIND = 4.0

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

print(f"Checking ALL {201} LiDAR messages for points beyond {BLIND}m...")
print(f"{'msg':>4}  {'nonzero':>8}  {'beyond_4m':>10}  {'max_dist':>10}  stamp")
print("-"*70)

msg_idx = 0
for conn, timestamp, rawdata in reader.messages(connections=[lidar_conn]):
    msg = ros2_store.deserialize_cdr(rawdata, conn.msgtype)
    
    nonzero = 0
    beyond_blind = 0
    max_dist = 0.0

    for p in msg.points:
        dist_sq = p.x * p.x + p.y * p.y + p.z * p.z
        if dist_sq > 0:
            nonzero += 1
        dist = math.sqrt(dist_sq)
        if dist > max_dist:
            max_dist = dist
        if dist > BLIND:
            beyond_blind += 1

    stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
    print(f"{msg_idx:4d}  {nonzero:8d}  {beyond_blind:10d}  {max_dist:10.3f}  {stamp}")
    msg_idx += 1

reader.close()
