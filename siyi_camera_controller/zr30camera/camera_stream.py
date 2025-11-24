import cv2
import rclpy
import numpy as np 

# Se elimina CvBridge ya que usaremos codificación manual jpg
# from cv_bridge import CvBridge 
from rclpy.node import Node
# Importamos el mensaje de imagen comprimida
from sensor_msgs.msg import CompressedImage

_CAM_STREAM_URI = "rtsp://192.168.144.25:8554/main.264"
_CAM_NODE_NAME = "camera_node"
# Es convención que los topics comprimidos terminen en /compressed, 
# pero puedes dejarlo igual si lo prefieres.
_CAM_PUB_TOPIC = "ZR30/camera_stream/compressed" 
_CAM_FRAME_ID = "ZR30_Camera_Capture"
_QUEUE_SIZE = 100
_PUBLISH_PERIOD_SEC = 0.01
_JPEG_QUALITY = 60  # Ajustar calidad (0-100) según ancho de banda deseado

class CameraStreamNode(Node):
    def __init__(self, capture: cv2.VideoCapture, node_name: str =_CAM_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.capture = capture
        # self.bridge = CvBridge() # No necesario para CompressedImage con este método manual
        
        # Definir el publisher con tipo CompressedImage
        self.publisher = self.create_publisher(CompressedImage, _CAM_PUB_TOPIC, _QUEUE_SIZE)

        # Definir frecuencia de publicación y callback
        self.timer_ = self.create_timer(pub_period, self.capture_image_callback)
        self.i = 0

    def capture_image_callback(self) -> None:
        """
        Captures an image from the camera via RTSP, compresses it to JPEG, 
        and publishes it as a ROS CompressedImage message.
        """
        ret, frame = self.capture.read()
        
        if ret:
            # Comprimir la imagen a formato JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY]
            result, encimg = cv2.imencode('.jpg', frame, encode_param)

            if result:
                msg = CompressedImage()
                msg.header.frame_id = _CAM_FRAME_ID
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(encimg).tobytes()

                # Nota: CompressedImage no requiere msg.height, msg.width ni msg.step
                # ya que esa información viaja dentro de la cabecera del propio JPEG.

                self.publisher.publish(msg)
                self.i += 1
        else:
            self.get_logger().warn("No se pudo leer el frame del RTSP stream")


def main(args=None):
    # Inicializar captura
    capture = cv2.VideoCapture(_CAM_STREAM_URI)
    
    # Verificar si se abrió correctamente (opcional pero recomendado)
    if not capture.isOpened():
        print(f"Error al abrir el stream RTSP: {_CAM_STREAM_URI}")
        return

    rclpy.init(args=args)
    camera_publisher = CameraStreamNode(capture=capture)

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()
        capture.release()

if __name__ == "__main__":
    main()