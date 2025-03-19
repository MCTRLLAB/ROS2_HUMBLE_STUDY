import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Create a publisher for the occupancy grid
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(2.0, self.publish_map)

    def publish_map(self):
        """Publish a simple 5x5 occupancy grid."""
        msg = OccupancyGrid()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Map Metadata
        msg.info = MapMetaData()
        msg.info.resolution = 0.05  # Each grid cell is 5cm
        msg.info.width = 5  # Grid width (5 cells)
        msg.info.height = 5  # Grid height (5 cells)
        msg.info.origin = Pose()
        msg.info.origin.position = Point(x=-0.25, y=-0.25, z=0.0)  # Bottom-left corner
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Define map occupancy grid (5x5)
        msg.data = [
            0,  0,  0,  0,  0,  # Free space
            0,  100, 100, 100,  0,  # Wall
            0,  100, -1, 100,  0,  # Unknown area
            0,  100, 100, 100,  0,  # Wall
            0,  0,  0,  0,  0   # Free space
        ]

        # Publish the map
        self.publisher_.publish(msg)
        self.get_logger().info("Published a simple 5x5 occupancy grid map.")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
