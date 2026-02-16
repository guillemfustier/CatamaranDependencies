#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from pymavlink import mavutil
import tf_transformations

class MAVLinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')

        # Declarar parámetros para IP y puerto
        self.declare_parameter('mavlink_ip', '0.0.0.0')  # Valor por defecto
        self.declare_parameter('mavlink_port', 14550)    # Valor por defecto

        # Obtener valores de los parámetros
        mavlink_ip = self.get_parameter('mavlink_ip').get_parameter_value().string_value
        mavlink_port = self.get_parameter('mavlink_port').get_parameter_value().integer_value

        # Conectar con MAVLink (usando los parámetros)
        mavlink_address = f'udp:{mavlink_ip}:{mavlink_port}'
        self.get_logger().info(f'Conectando a MAVLink en {mavlink_address}')
        self.mav = mavutil.mavlink_connection(mavlink_address)

        # Crear publicadores en ROS 2
        self.orientation_pub = self.create_publisher(Quaternion, 'mavlink/orientation', 10)

        # Crear un timer para leer MAVLink
        self.timer = self.create_timer(0.05, self.read_mavlink)

    def read_mavlink(self):
        """Lee los datos de MAVLink y los publica en ROS 2"""
        while True:
            msg = self.mav.recv_match(blocking=True, timeout=1)
            #msg = self.mav.recv_match(blocking=False, timeout=0) #
            if msg:
                if msg.get_type() == 'ATTITUDE':
                    roll = msg.roll
                    pitch = msg.pitch
                    yaw = msg.yaw

                    # Convertir a cuaternión con el nuevo orden
                    quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

                    # Publicar en ROS 2
                    orientation_msg = Quaternion()
                    orientation_msg.x = quat[0]
                    orientation_msg.y = quat[1]
                    orientation_msg.z = quat[2]
                    orientation_msg.w = quat[3]
                    
                    self.orientation_pub.publish(orientation_msg)


                    self.get_logger().info(f"Orientación: {roll}, {pitch}, {yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
