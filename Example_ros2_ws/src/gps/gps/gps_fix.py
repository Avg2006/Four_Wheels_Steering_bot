import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix, NavSatStatus


class PointToNavSatFix(Node):
    def __init__(self):
        super().__init__('point_to_navsatfix')

        self.subscription = self.create_subscription(Point,'/sim/gps',self.point_callback,10)

        self.publisher = self.create_publisher(NavSatFix, '/sim/fix_gps', 10)

    def point_callback(self, msg: Point):
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = "gps_link"
        navsat_msg.latitude = msg.x
        navsat_msg.longitude = msg.y
        navsat_msg.altitude = 0.0
        navsat_msg.status.status = NavSatStatus.STATUS_FIX
        navsat_msg.status.service = NavSatStatus.SERVICE_GPS
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(navsat_msg)
        self.get_logger().info(
            f"Published NavSatFix: lat={msg.x}, lon={msg.y}, alt={msg.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointToNavSatFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
