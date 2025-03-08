import rclpy, rclpy.publisher, rclpy.timer
from rclpy.node import Node
from std_msgs.msg import Int8

class my_second_publisher(Node):
    def __init__(self) -> None:
        super().__init__('my_second_publisher')
        
        self.int_msg: Int8 = Int8()
        self.int_pub: rclpy.publisher = self.create_publisher(
            Int8,
            '/test/int',
            10
        )
        self.frequency: float = 10.0 # Hz
        self.timer: rclpy.timer = self.create_timer(
            1 / self.frequency,
            self.timer_callback
        )
        self.i: int = 0
        
    def timer_callback(self) -> None:
        self.int_msg.data = 10
        self.int_pub.publish(self.int_msg)
        self.get_logger().info('Sending message...')
        self.i += 1
        
def main(args = None) -> None:
    rclpy.init(args = args)
    
    publisher: my_second_publisher = my_second_publisher()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()