import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')
        
        
        self.declare_parameters(
            namespace='DEL URDF VRO',
            parameters=[
                ('wheel_radius', 0), #NO TENGO ESTE
                ('lx', 0),  #NI ESTE
                ('ly', 0),  # NI ESTE
            ]
        )
        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_speeds_pub = self.create_publisher(
            JointState, '/wheel_speeds', 10)
        
        self.get_logger().info("Nodo de cinemática mecanum inicializado")

    def mecanum_inverse_kinematics(self, vx, vy, omega):
        w1 = (vx - vy - (self.lx + self.ly) * omega) / self.r
        w2 = (vx + vy + (self.lx + self.ly) * omega) / self.r
        w3 = (vx + vy - (self.lx + self.ly) * omega) / self.r
        w4 = (vx - vy + (self.lx + self.ly) * omega) / self.r
        return [w1, w2, w3, w4]

    def mecanum_direct_kinematics(self, w1, w2, w3, w4):
        vx = (self.r / 4) * (w1 + w2 + w3 + w4)
        vy = (self.r / 4) * (-w1 + w2 + w3 - w4)
        omega = (self.r / 4) * (-w1 + w2 - w3 + w4) / (self.lx + self.ly)
        return vx, vy, omega

    def cmd_vel_callback(self, msg):
        w1, w2, w3, w4 = self.mecanum_inverse_kinematics(
            msg.linear.x, msg.linear.y, msg.angular.z)
        
        wheel_speeds_msg = JointState()
        wheel_speeds_msg.name = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint']
        wheel_speeds_msg.velocity = [w1, w2, w3, w4]
        self.wheel_speeds_pub.publish(wheel_speeds_msg)

def main():
    rclpy.init()
    node = MecanumKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()