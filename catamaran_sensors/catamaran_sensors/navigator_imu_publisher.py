#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from pymavlink import mavutil
import tf_transformations
import math

class NavigatorImuPublisher(Node):
    def __init__(self):
        super().__init__('navigator_imu_publisher')

        # Declare parameters for IP and port
        self.declare_parameter('mavlink_ip', '0.0.0.0')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('imu_frame_id', 'imu_link')

        # Get parameter values
        mavlink_ip = self.get_parameter('mavlink_ip').get_parameter_value().string_value
        mavlink_port = self.get_parameter('mavlink_port').get_parameter_value().integer_value
        self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

        # Connect to MAVLink
        mavlink_address = f'udp:{mavlink_ip}:{mavlink_port}'
        self.get_logger().info(f'Connecting to MAVLink at {mavlink_address}')
        self.mav = mavutil.mavlink_connection(mavlink_address)

        # Create publishers
        # Publishing just the orientation as requested
        self.orientation_pub = self.create_publisher(Quaternion, 'navigator/orientation', 10)
        
        # Also publishing standard Imu message for RViz compatibility (as user mentioned "integrate all in rviz")
        self.imu_pub = self.create_publisher(Imu, 'navigator/imu', 10)

        # Create a timer to read MAVLink
        self.timer = self.create_timer(0.01, self.read_mavlink)

    def read_mavlink(self):
        """Reads MAVLink data and publishes to ROS 2"""
        while True:
            # Non-blocking receive to process all available messages
            msg = self.mav.recv_match(blocking=False)
            if not msg:
                break

            if msg.get_type() == 'ATTITUDE':
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw

                # Convert to quaternion
                quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

                # Create Quaternion message
                orientation_msg = Quaternion()
                orientation_msg.x = quat[0]
                orientation_msg.y = quat[1]
                orientation_msg.z = quat[2]
                orientation_msg.w = quat[3]
                
                self.orientation_pub.publish(orientation_msg)

                # Create Imu message for standard visualization
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.imu_frame_id
                imu_msg.orientation = orientation_msg
                
                # MAVLink ATTITUDE usually doesn't have covariance, set to 0 (unknown) or small value
                imu_msg.orientation_covariance = [0.0] * 9
                imu_msg.angular_velocity_covariance = [0.0] * 9
                imu_msg.linear_acceleration_covariance = [0.0] * 9
                
                # Fill angular velocity if available (ATTITUDE has rollspeed, pitchspeed, yawspeed)
                imu_msg.angular_velocity.x = msg.rollspeed
                imu_msg.angular_velocity.y = msg.pitchspeed
                imu_msg.angular_velocity.z = msg.yawspeed

                self.imu_pub.publish(imu_msg)

                # self.get_logger().debug(f"Orientation: {roll}, {pitch}, {yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
