import rclpy, rclpy.publisher, rclpy.timer
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self) -> None:
        super().__init__('My_Publisher')
        
        self.str_msg: String = String()
        self.str_pub: rclpy.publisher = self.create_publisher(
            String,
            '/test/str',
            10
        )
        self.frequency: float = 1.0 # Hz
        self.timer: rclpy.timer = self.create_timer(
            1 / self.frequency,
            self.timer_callback
        )
        self.i: int = 0
        
    def timer_callback(self) -> None:
        self.str_msg.data = f'{self.i}: Hello World!'
        self.str_pub.publish(self.str_msg)
        self.get_logger().info('Sending message...')
        self.i += 1
        
def main(args = None) -> None:
    rclpy.init(args = args)
    
    publisher: Publisher = Publisher()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()