import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, pi
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

class ManipulatorKinematics(Node):
    def __init__(self):
        super().__init__('manipulator_kinematics')
        

        self.dh_params = [
            {'theta_offset': 0, 'd': 'l1', 'a': 0, 'alpha': pi/2},
            {'theta_offset': 0, 'd': 0, 'a': 'l2', 'alpha': 0},
            {'theta_offset': 0, 'd': 0, 'a': 'l3', 'alpha': 0},
            {'theta_offset': pi/2, 'd': 0, 'a': 0, 'alpha': pi/2},
            {'theta_offset': 0, 'd': 'l4 + l5', 'a': 0, 'alpha': 0}
        ]
        
        #FALTAN LOS VALORES DE LAS DISTANCIAS DE LOS LINKS
        self.l1 = 0.3
        self.l2 = 0.5
        self.l3 = 0.4
        self.l4_plus_l5 = 0.2
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        
        self.get_logger().info("Nodo de cinemática del manipulador inicializado")

    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_positions):
        T = np.eye(4)
        
        for i in range(5):
            params = self.dh_params[i]
            
            theta = joint_positions[i] + params['theta_offset']
            
            d = self.l1 if params['d'] == 'l1' else (
                self.l4_plus_l5 if params['d'] == 'l4 + l5' else params['d']
            )
            a = self.l2 if params['a'] == 'l2' else (
                self.l3 if params['a'] == 'l3' else params['a']
            )
            alpha = params['alpha']
            
            Ti = self.dh_matrix(theta, d, a, alpha)
            T = np.dot(T, Ti)
        
        position = T[:3, 3]
    
        rotation = T[:3, :3]
        
        return position, rotation

    def joint_callback(self, msg):

        if len(msg.position) >= 5:
            position, rotation = self.forward_kinematics(msg.position)
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'end_effector'
            t.transform.translation.x = float(position[0])
            t.transform.translation.y = float(position[1])
            t.transform.translation.z = float(position[2])
            
            from tf_transformations import quaternion_from_matrix
            q = quaternion_from_matrix(
                np.vstack([np.hstack([rotation, [[0],[0],[0]]), [0,0,0,1]])
            )
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f"End-Effector en: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

def main():
    rclpy.init()
    node = ManipulatorKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()