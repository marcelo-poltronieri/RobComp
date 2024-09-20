import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.laser import Laser
from sensor_msgs.msg import LaserScan
import numpy as np
from math import *
import time



class Indeciso(Node, Laser):

    def __init__(self):
        Node.__init__(self, 'indeciso')
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'
        self.state_machine = {
            'stop': self.stop,
            'forward': self.forward,
            'backward': self.backward
        }

        self.twist = Twist()


        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def stop(self):
        if np.min(self.front) > 1.05:
            self.robot_state = 'forward'
        elif np.min(self.front) < 0.95:
            self.robot_state = 'backward'
        else:
            self.robot_state = 'stop'
            self.twist.linear.x = 0.0

    def forward(self):
        self.twist.linear.x = 0.2
        if np.min(self.front) > 1.05:
            self.robot_state = 'forward'
        elif np.min(self.front) < 0.95:
            self.robot_state = 'backward'

    def backward(self):
        self.twist.linear.x = -0.2

        if np.min(self.front) < 0.95:
            self.robot_state = 'backward'

        elif np.min(self.front) > 1.05:
            self.robot_state = 'forward'
        else:
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'


    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Indeciso()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()