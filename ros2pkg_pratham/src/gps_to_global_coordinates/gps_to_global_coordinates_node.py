import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import tf2_ros

class GPSToGlobalCoordinates(Node):
    def __init__(self):
        super().__init__('gps_to_global_coordinates')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps',
            self.gps_callback,
            10)
        self.global_coordinates_publisher = self.create_publisher(Point, 'global_coordinates', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def gps_callback(self, msg):
        # Simple transformation from GPS data (e.g., lat/lon to X/Y)
        # In a real-world scenario, you would use something like geodesy to perform this transformation
        global_point = Point()
        global_point.x = msg.latitude  # Placeholder transformation
        global_point.y = msg.longitude

        # Publish transformed global coordinates
        self.global_coordinates_publisher.publish(global_point)

def main(args=None):
    rclpy.init(args=args)
    node = GPSToGlobalCoordinates()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

