import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math


class GPS2XY(Node):
    def __init__(self):
        super().__init__('gps_to_xy')

        self.lat0 = math.radians(12.99151)
        self.lon0 = math.radians(80.23362)

        self.theta = math.radians(30.0)

        self.R = 6378137.0
        self.sub = self.create_subscription(NavSatFix,'/sim/fix_gps',self.gps_callback,10)

    def gps_callback(self, msg: NavSatFix):
        print("hello")
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        dlat = lat - self.lat0
        dlon = lon - self.lon0

        east = self.R * dlon * math.cos(self.lat0)
        north = self.R * dlat

        x = east * math.sin(self.theta) + north * math.cos(self.theta)
        y = -east * math.cos(self.theta) + north * math.sin(self.theta)

        self.get_logger().info(
            f"lat={msg.latitude:.6f}, lon={msg.longitude:.6f} -> x={x:.2f}, y={y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPS2XY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
