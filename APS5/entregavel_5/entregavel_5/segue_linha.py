import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2
import numpy as np

class SegmentaLinhaNode(Node):

    def __init__(self):
        super().__init__('segmenta_linha_node')
        self.running = True
        self.bridge = CvBridge()
        
        # Subscriber para imagem comprimida
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/color/image_raw/compressed',  # Ajuste o tópico conforme necessário
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Variáveis para armazenar a posição da linha
        self.x = -1
        self.y = -1
        self.w = None

    def image_callback(self, msg):
        if not self.running:
            return
        
        # Converte a imagem ROS para OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Processamento da imagem para segmentar a linha amarela
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Definir intervalo de cores para detectar a linha amarela (ajustar conforme necessário)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
        # Encontrar contornos na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Encontra o maior contorno
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] > 0:
                # Calcula o centro do contorno
                self.x = int(M["m10"] / M["m00"])
                self.y = int(M["m01"] / M["m00"])
            else:
                self.x, self.y = -1, -1
        else:
            # Se não houver contorno, define posição como (-1, -1)
            self.x, self.y = -1, -1

        # Desenha o contorno e o centro na imagem
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
        if self.x != -1 and self.y != -1:
            cv2.circle(cv_image, (self.x, self.y), 5, (0, 0, 255), -1)

        # Exibe a imagem
        cv2.imshow("Segmenta Linha", cv_image)
        cv2.waitKey(1)
        
        # Calcula a distância do centro da linha para o centro da imagem
        self.w = cv_image.shape[1] // 2  # Largura da imagem
        if self.x <= -1:
            distancia = self.x - self.w
            if distancia < -320 or distancia > 320:
                self.get_logger().info("Nenhuma linha detectada")
            else:
                self.get_logger().info(f"Distância do centro da linha ao centro da imagem: {distancia}")
        elif self.x > -1:
            distancia = self.w - self.x
            if distancia < -320 or distancia > 320:
                self.get_logger().info("Nenhuma linha detectada")
            else:
                self.get_logger().info(f"Distância do centro da linha ao centro da imagem: {distancia}")

        else:
            self.get_logger().info("Nenhuma linha detectada")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentaLinhaNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()