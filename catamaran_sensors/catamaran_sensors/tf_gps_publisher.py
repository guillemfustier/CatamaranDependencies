#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geopy.distance import geodesic
import numpy as np

class GPS2TF(Node):
    def __init__(self):
        super().__init__('gps_to_tf')

        # Suscriptores a GPS y orientación
        self.gps_sub = self.create_subscription(NavSatFix, '/mavlink/gps', self.gps_callback, 10)
        self.orientation_sub = self.create_subscription(Quaternion, '/mavlink/orientation', self.orientation_callback, 10)

        # Transform Broadcaster para publicar la TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Variables para almacenar datos
        self.origin = None
        self.gps_data = []
        self.orientation = Quaternion()

        # Obtener el origen desde parámetros de ROS
        self.declare_parameter('origin_lat', None)
        self.declare_parameter('origin_lon', None)

        lat = self.get_parameter('origin_lat').value
        lon = self.get_parameter('origin_lon').value

        if lat is not None and lon is not None:
            self.origin = (lat, lon)
            self.get_logger().info(f'Usando origen definido en parámetros: {self.origin}')
        else:
            self.get_logger().info('Esperando los primeros 5 valores para calcular el origen...')

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        # Si el origen no está definido, acumular datos y calcular el promedio
        if self.origin is None:
            self.gps_data.append((lat, lon))

            if len(self.gps_data) >= 5:
                avg_lat = np.mean([p[0] for p in self.gps_data])
                avg_lon = np.mean([p[1] for p in self.gps_data])
                self.origin = (avg_lat, avg_lon)
                self.get_logger().info(f'Origen calculado: {self.origin}')
            return

        # Convertir coordenadas GPS a NED (x, y)
        x, y = self.geodetic2ned(lat, lon, self.origin)
        z = -0.2

        # Publicar la TF desde "map" a "blueboat_base_link"
        self.publish_tf(x, y, z, self.orientation)

    def orientation_callback(self, msg):
        self.orientation = msg  # Guardar la orientación actual

    def geodetic2ned(self, lat, lon, origin):
        # Convierte coordenadas geodésicas (lat, lon) a coordenadas NED (x, y)
        ned = geodesic((origin[0], origin[1]), (lat, lon)).meters
        return float(ned), float(0)  # Convertimos a float explícitamente

    def publish_tf(self, x, y, z, orientation):
        """Publica la transformada TF desde 'map' a 'catamaran_base_link'."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "catamaran_base_link"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = -0.2
        t.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'TF publicada: map -> catamaran_base_link | x: {x}, y: {y}, z: {z}')

def main(args=None):
    rclpy.init(args=args)
    node = GPS2TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
