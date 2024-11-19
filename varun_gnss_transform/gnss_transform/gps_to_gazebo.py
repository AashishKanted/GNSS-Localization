import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import math

class GPSToGazebo(Node):
    def __init__(self):
        super().__init__('gps_to_gazebo')
        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/gazebo/global_pose', 10)

        # Reference point in GPS coordinates (lat, lon, alt) corresponding to Gazebo origin
        self.reference_lat = 37.4275  # Example latitude
        self.reference_lon = -122.1697  # Example longitude
        self.reference_alt = 0.0  # Altitude

    def gps_callback(self, msg):
        # Convert GPS coordinates to Gazebo coordinates
        x, y = self.gps_to_gazebo(msg.latitude, msg.longitude)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = msg.altitude - self.reference_alt

        # Assuming no rotation for simplicity
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(pose_msg)

    def gps_to_gazebo(self, lat, lon):
        # Calculate the difference in coordinates
        delta_lat = math.radians(lat - self.reference_lat)
        delta_lon = math.radians(lon - self.reference_lon)

        # Approximate conversion using Earth's radius
        earth_radius = 6371000  # Meters
        x = earth_radius * delta_lon * math.cos(math.radians(self.reference_lat))
        y = earth_radius * delta_lat
        return x, y

def main(args=None):
    rclpy.init(args=args)
    node = GPSToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
