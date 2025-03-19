import rclpy
from rclpy.node import Node
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')

        # Create a publisher for PointCloud2
        self.publisher_ = self.create_publisher(PointCloud2, '/simulated_cloud', 10)
        
        # Publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_point_cloud)

    def create_point_cloud(self):
        """Generate a synthetic point cloud (e.g., cube)."""
        num_points = 1000  # Number of points in the cloud
        points = np.random.uniform(-1, 1, (num_points, 3))  # Generate points in a cube [-1,1]

        # Convert to byte format
        cloud_data = []
        for p in points:
            cloud_data.append(struct.pack('fff', p[0], p[1], p[2]))  # Pack x, y, z

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Set frame ID
        msg.height = 1  # Unordered point cloud
        msg.width = num_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * num_points
        msg.data = b''.join(cloud_data)
        msg.is_dense = True

        return msg

    def publish_point_cloud(self):
        """Publish the synthetic point cloud."""
        cloud_msg = self.create_point_cloud()
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f'Published {cloud_msg.width} points')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
