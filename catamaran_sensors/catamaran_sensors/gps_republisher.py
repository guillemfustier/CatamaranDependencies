#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data


class GPSRepublisher(Node):

    def __init__(self):
        super().__init__('gps_republisher')

        # Suscriptor con QoS de sensor
        self.subscription = self.create_subscription(
            NavSatFix,
            '/catamaran/mavlink/gps',
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/catamaran/republished/gps',
            10
        )

        self.get_logger().info("GPS Republisher Node Started")

    def gps_callback(self, msg: NavSatFix):
        output_msg = Float64MultiArray()
        output_msg.data = [
            msg.latitude,
            msg.longitude,
            msg.altitude
        ]

        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
