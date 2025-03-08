import rclpy, rclpy.subscription
from rclpy.node import Node
from std_msgs.msg import String, Int8

class Subscriber(Node):
    def __init__(self) -> None:
        super().__init__('My_subscriber')
        
        self.int: Int8 =  Int8()
        self.str: String = String()
        
        self.int_sub: rclpy.subscription = self.create_subscription(Int8, 
                                                                    '/test/int',
                                                                    self.int_callback,
                                                                    10)
        
        self.str_sub: rclpy.subscription = self.create_subscription(String,
                                                                    '/test/str',
                                                                    self.str_callback,
                                                                    10)
    def int_callback(self, msg) -> None:
        self.int = msg
        self.get_logger().info(f'{msg.data}')
    
    def str_callback(self, msg) -> None:
        self.str = msg
        self.get_logger().info(f'{msg.data}')

def main(args=None) -> None:
    rclpy.init(args=args)
    
    subscriber: Subscriber = Subscriber()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()