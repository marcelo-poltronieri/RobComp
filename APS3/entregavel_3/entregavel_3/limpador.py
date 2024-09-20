import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from my_package.odom import Odom

from my_package.laser import *

class FirstNode(Node, Laser,Odom):

    def __init__(self):
        Node.__init__(self, 'limpador_node')
        Laser.__init__(self)
        Odom.__init__(self)

        self.twist = Twist()
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'
        self.state_machine = {
            'forward': self.forward,
            'turn': self.turn
        }

    def custom_laser(self):
        self.lower_right = self.laser_msg[180-self.openning:270+self.openning]
        
    def control(self):
        print(f'O estado atual do robo é: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.vel_pub.publish(self.twist)
        print(self.front)

    def forward(self):
        self.twist.linear.x = 0.5

        if min(self.front) < 0.5:
            self.twist = Twist()
            self.robot_state = 'turn'
            #self.self_front_min = min(self.front)
            self.goal_yaw = self.yaw + 3*np.pi/4

    def turn(self):
        self.twist.angular.z = 0.3

        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        if abs(erro) <= np.deg2rad(2): #min(self.lower_right) < 0.5:#1.1*self.self_front_min:
            self.twist = Twist()
            self.robot_state = 'forward'

def main(args=None):
    rclpy.init(args=args)
    first_node = FirstNode()

    rclpy.spin(first_node)

    first_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
