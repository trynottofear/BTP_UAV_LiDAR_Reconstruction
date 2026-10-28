import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        pcd_path = (
            '/home/shaurya/BTP_UAV_LiDAR_Reconstruction/'
            'src/FAST_LIO_ROS2/PCD/scans.pcd'
        )

        self.get_logger().info(f'Loading {pcd_path}...')

        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32)

        self.get_logger().info(f'Loaded {len(points)} points.')

        self.publisher = self.create_publisher(
            PointCloud2,
            '/fast_lio/map',
            10
        )

        header = Header()
        header.frame_id = 'map'

        self.msg = point_cloud2.create_cloud_xyz32(
            header,
            points.tolist()
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_cloud
        )

    def publish_cloud(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()

    node = PCDPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
