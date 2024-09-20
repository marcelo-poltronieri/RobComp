import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

"""
ros2 launch my_package first_node.launch.py
"""

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.vel_pub = self.create_publisher(String, 'publisher', 10)

        self.timer = self.create_timer(0.25, self.control)
        self.contador = 0
    
    def control(self):
        self.string = String()
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9        
        print(f"Ola, sao {current_time} e estou publicando pela {self.contador} vez")
        self.string.data = f"{current_time} {self.contador}"
        self.vel_pub.publish(self.string)
        self.contador += 1

    


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
