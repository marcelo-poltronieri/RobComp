import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from my_package.odom import Odom
# Adicione aqui os imports necessários

class Quadrado_Node(Node, Odom): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'quadrado')
        Odom.__init__(self)
        time.sleep(2)

        self.timer = self.create_timer(0.25, self.control)
        self.tempo_final = self.get_clock().now().to_msg().sec + 4


        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'girar':self.girar,
            'stop':self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def stop(self):
        self.twist = Twist()
    
    def andar(self):
        self.tempo_sec = self.get_clock().now().to_msg().sec

        self.twist.linear.x = 0.2
        print(f'tempo atual: {self.tempo_sec}')
        print(f'tempo final: {self.tempo_final}')

        if (self.tempo_final - self.tempo_sec) <= 1:
            self.robot_state = 'girar'
            self.goal_yaw = (self.yaw + np.pi / 2) % (2 * np.pi)
            print(f'{np.rad2deg(self.goal_yaw)}') # Calula o angulo final


    def girar(self):
        self.twist.angular.z = 0.2
        print(f'Angulo atual: {np.rad2deg(self.yaw)}')
        print(f'Angulo final: {np.rad2deg(self.goal_yaw)}')

        if (self.goal_yaw - self.yaw) % (2* np.pi) <= np.deg2rad(3):
            self.robot_state = 'andar'
            self.tempo_final = self.get_clock().now().to_msg().sec + 4


    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Quadrado_Node() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()