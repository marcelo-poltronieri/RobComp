import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.timer = self.create_timer(0.25, self.control)

        self.split = []

        self.publisher_sub = self.create_subscription(
            String,
            'publisher',
            self.publisher_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def publisher_callback(self, msg: String):
        mensagem = msg.data
        self.split = mensagem.split()
        

    def control(self):
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9        
        self.delay = current_time - float(self.split[0])
        print(f'Ola, estou recebendo a mensagem: {self.split[1]} que demorou {self.delay} segundos para ser recebidas')


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
