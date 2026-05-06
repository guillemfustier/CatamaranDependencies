#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class SBSImageNode(Node):

    def __init__(self):
        super().__init__('sbs_image_node')

        self.img1 = None  # front_camera
        self.img2 = None  # front_camera2

        # Subscripciones
        self.sub1 = self.create_subscription(
            CompressedImage,
            '/girona500/front_camera/camera/image_color/compressed',
            self.callback1,
            10)

        self.sub2 = self.create_subscription(
            CompressedImage,
            '/girona500/front_camera2/camera/image_color/compressed',
            self.callback2,
            10)

        # Publisher
        self.pub = self.create_publisher(
            CompressedImage,
            '/sbs/image/compressed',
            10)

    def callback1(self, msg):
        self.img1 = msg
        self.process()

    def callback2(self, msg):
        self.img2 = msg
        self.process()

    def process(self):
        if self.img1 is None or self.img2 is None:
            return

        try:
            # Convertir compressed → cv2
            np_arr1 = np.frombuffer(self.img1.data, np.uint8)
            np_arr2 = np.frombuffer(self.img2.data, np.uint8)

            img1 = cv2.imdecode(np_arr1, cv2.IMREAD_COLOR)
            img2 = cv2.imdecode(np_arr2, cv2.IMREAD_COLOR)

            # Redimensionar si necesario
            h = min(img1.shape[0], img2.shape[0])
            img1 = cv2.resize(img1, (int(img1.shape[1] * h / img1.shape[0]), h))
            img2 = cv2.resize(img2, (int(img2.shape[1] * h / img2.shape[0]), h))

            # SBS: img2 izquierda, img1 derecha
            sbs = np.hstack((img2, img1))

            # Codificar como compressed
            _, buffer = cv2.imencode('.jpg', sbs)

            out_msg = CompressedImage()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.format = "jpeg"
            out_msg.data = buffer.tobytes()

            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error procesando imágenes: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SBSImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
