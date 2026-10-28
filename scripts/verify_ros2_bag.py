"""Verify that the converted ROS2 bag has valid (nonzero) LiDAR XYZ data."""
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
import sys
import struct

BAG_PATH = Path(sys.argv[1])

# Set up ROS2 type store with Livox CustomMsg
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

ros2_types = get_types_from_msg(
    ros2_msgdef,
    "livox_ros_driver2/msg/CustomMsg"
)
ros2_store.register(ros2_types)

reader = Reader(BAG_PATH)
reader.open()

lidar_conn = next(c for c in reader.connections if c.topic == "/livox/lidar")

print(f"Bag: {BAG_PATH}")
print(f"Checking LiDAR messages on /livox/lidar ...\n")

msg_idx = 0
MAX_MSGS = 10  # check first 10 LiDAR messages

for conn, timestamp, rawdata in reader.messages(connections=[lidar_conn]):
    if msg_idx >= MAX_MSGS:
        break

    msg = ros2_store.deserialize_cdr(rawdata, conn.msgtype)

    total_pts = len(msg.points)
    nonzero = 0
    first_nonzero = None

    for i, p in enumerate(msg.points):
        if p.x != 0.0 or p.y != 0.0 or p.z != 0.0:
            nonzero += 1
            if first_nonzero is None:
                first_nonzero = (i, p.x, p.y, p.z)

    print(f"  msg {msg_idx}: point_num={msg.point_num}, "
          f"total_points={total_pts}, nonzero_xyz={nonzero}")

    if first_nonzero:
        idx, x, y, z = first_nonzero
        print(f"    first nonzero point[{idx}]: "
              f"x={x:.4f}, y={y:.4f}, z={z:.4f}")
    else:
        print(f"    ALL POINTS ARE ZERO")

    msg_idx += 1

print(f"\nChecked {msg_idx} LiDAR messages.")
reader.close()
