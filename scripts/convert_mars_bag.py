from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
import sys

INPUT = Path(sys.argv[1])
OUTPUT = Path(sys.argv[2])
SKIP_LIDAR = 169

# ROS1 type store
ros1_store = get_typestore(Stores.ROS1_NOETIC)

# ROS2 Jazzy type store
ros2_store = get_typestore(Stores.ROS2_JAZZY)

# Read ROS1 bag
reader = Reader(INPUT)
reader.open()

lidar_conn = next(c for c in reader.connections if c.topic == "/livox/lidar")
imu_conn = next(c for c in reader.connections if c.topic == "/livox/imu")

# Register Livox ROS1 custom message from the bag definition
ros1_types = get_types_from_msg(lidar_conn.msgdef[1], lidar_conn.msgtype)
ros1_store.register(ros1_types)

# ROS2 Livox CustomMsg definition
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

# Create ROS2 bag writer
writer = Writer(OUTPUT, version=8)
writer.open()

out_lidar = writer.add_connection(
    "/livox/lidar",
    "livox_ros_driver2/msg/CustomMsg",
    typestore=ros2_store,
)

out_imu = writer.add_connection(
    "/livox/imu",
    "sensor_msgs/msg/Imu",
    typestore=ros2_store,
)

# ROS2 message classes
Header = ros2_store.types["std_msgs/msg/Header"]
Time = ros2_store.types["builtin_interfaces/msg/Time"]
Quaternion = ros2_store.types["geometry_msgs/msg/Quaternion"]
Vector3 = ros2_store.types["geometry_msgs/msg/Vector3"]

CustomMsg = ros2_store.types["livox_ros_driver2/msg/CustomMsg"]
PointType = ros2_store.types["livox_ros_driver2/msg/CustomPoint"]
Imu = ros2_store.types["sensor_msgs/msg/Imu"]

first_timestamp = None
lidar_count = 0
imu_count = 0
lidar_seen = 0

for conn, timestamp, rawdata in reader.messages(
    connections=[lidar_conn, imu_conn]
):
    # =========================================================
    # LiDAR
    # =========================================================
    if conn.topic == "/livox/lidar":
        lidar_seen += 1

        # Skip invalid startup LiDAR scans.
        if lidar_seen <= SKIP_LIDAR:
            continue

        if first_timestamp is None:
            first_timestamp = timestamp

        src = ros1_store.deserialize_ros1(rawdata, conn.msgtype)

        points = []

        for p in src.points:
            q = PointType(
                offset_time=p.offset_time,
                x=p.x,
                y=p.y,
                z=p.z,
                reflectivity=p.reflectivity,
                tag=p.tag,
                line=p.line,
            )
            points.append(q)

        dst = CustomMsg(
            header=Header(
                stamp=Time(
                    sec=src.header.stamp.sec,
                    nanosec=src.header.stamp.nanosec,
                ),
                frame_id=src.header.frame_id,
            ),
            timebase=src.timebase,
            point_num=src.point_num,
            lidar_id=src.lidar_id,
            rsvd=src.rsvd,
            points=points,
        )

        serialized = ros2_store.serialize_cdr(
            dst,
            "livox_ros_driver2/msg/CustomMsg"
        )

        writer.write(out_lidar, timestamp, serialized)
        lidar_count += 1

    # =========================================================
    # IMU
    # =========================================================
    elif conn.topic == "/livox/imu":
        if first_timestamp is None or timestamp < first_timestamp:
            continue

        src = ros1_store.deserialize_ros1(rawdata, conn.msgtype)

        dst = Imu(
            header=Header(
                stamp=Time(
                    sec=src.header.stamp.sec,
                    nanosec=src.header.stamp.nanosec,
                ),
                frame_id=src.header.frame_id,
            ),
            orientation=Quaternion(
                x=src.orientation.x,
                y=src.orientation.y,
                z=src.orientation.z,
                w=src.orientation.w,
            ),
            orientation_covariance=src.orientation_covariance,
            angular_velocity=Vector3(
                x=src.angular_velocity.x,
                y=src.angular_velocity.y,
                z=src.angular_velocity.z,
            ),
            angular_velocity_covariance=src.angular_velocity_covariance,
            linear_acceleration=Vector3(
                x=src.linear_acceleration.x,
                y=src.linear_acceleration.y,
                z=src.linear_acceleration.z,
            ),
            linear_acceleration_covariance=src.linear_acceleration_covariance,
        )

        serialized = ros2_store.serialize_cdr(
            dst,
            "sensor_msgs/msg/Imu"
        )

        writer.write(out_imu, timestamp, serialized)
        imu_count += 1

writer.close()
reader.close()

print("Conversion complete.")
print(f"LiDAR messages: {lidar_count}")
print(f"IMU messages: {imu_count}")
print(f"Output: {OUTPUT}")

