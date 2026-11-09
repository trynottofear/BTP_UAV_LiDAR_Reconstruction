import os
import sys
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        default_pcd = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'src', 'FAST_LIO_ROS2', 'PCD', 'scans.pcd'
        )

        self.declare_parameter('pcd_path', default_pcd)
        self.declare_parameter('topic', '/cloud_registered')
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('latched', True)

        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value
        latched = self.get_parameter('latched').get_parameter_value().bool_value

        self.get_logger().info(f'Loading PCD: {pcd_path}')
        if not os.path.exists(pcd_path):
            self.get_logger().error(f'File not found: {pcd_path}')
            sys.exit(1)

        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32)
        self.get_logger().info(f'Loaded {len(points)} points from {pcd_path}.')

        if latched:
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST
            )
        else:
            qos = QoSProfile(depth=10)

        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.get_logger().info(f'Publishing to {topic} [frame: {self.frame_id}, rate: {rate}Hz, latched: {latched}]')

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        self.msg = point_cloud2.create_cloud_xyz32(header, points.tolist())

        # Publish immediately for latched subscriber
        self.publisher.publish(self.msg)

        if rate > 0.0:
            self.timer = self.create_timer(1.0 / rate, self.publish_cloud)

    def publish_cloud(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = PCDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
