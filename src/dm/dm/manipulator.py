import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from ros_gz_interfaces.msg import Actuators # For Gazebo sync
from pymata4 import pymata4
import time
import math

class RobotManipulador(Node):
    def __init__(self):
        super().__init__('robot_manipulador_arduino_controller')

        # --- Arduino Setup ---
        try:
            self.board = pymata4.Pymata4()
            self.servo_pins = [3, 5, 6, 9, 10] # Assuming 5 DOF for Arduino
            self.current_angles_deg = [90] * len(self.servo_pins) # Store last commanded degrees
            for pin in self.servo_pins:
                self.board.set_pin_mode_servo(pin)
            self.get_logger().info('Arduino board initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Arduino board: {e}')
            self.board = None # Ensure board is None if init fails

        # --- ROS Setup ---
        self.num_joints = len(self.servo_pins)

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_commands', # Expects degrees
            self.command_callback,
            10
        )

        # Publisher for Gazebo joint commands (optional synchronization)
        self.gz_command_publisher = self.create_publisher(
            Actuators,
            '/cmd_man', # ROS topic that bridges to Gazebo
            10
        )
        self.get_logger().info(f'Manipulator Arduino controller node ({self.num_joints}-DOF) initiated.')
        self.get_logger().info(f'Subscribing to Int32MultiArray on /servo_commands (expects {self.num_joints} angles in degrees).')
        if self.board:
            self.get_logger().info('Ready to send commands to Arduino.')
        if self.gz_command_publisher:
            self.get_logger().info('Also publishing to /cmd_man for Gazebo synchronization.')


    def command_callback(self, msg: Int32MultiArray):
        if len(msg.data) != self.num_joints:
            self.get_logger().warn(
                f'Expected {self.num_joints} angle values, received {len(msg.data)}. Ignoring command.'
            )
            return

        target_positions_rad_for_gz = [0.0] * self.num_joints

        for i in range(self.num_joints):
            angle_deg = msg.data[i]
            clamped_angle_deg = max(0, min(180, angle_deg)) # Standard servo range

            # --- Command Physical Servo via Arduino ---
            if self.board:
                try:
                    self.board.servo_write(self.servo_pins[i], clamped_angle_deg)
                    self.current_angles_deg[i] = clamped_angle_deg
                except Exception as e:
                    self.get_logger().error(f"Error writing to servo pin {self.servo_pins[i]}: {e}")
            
            # --- Prepare command for Gazebo (optional) ---
            target_positions_rad_for_gz[i] = math.radians(clamped_angle_deg)
            # Add a small delay if needed between servo commands
            # time.sleep(0.015) 


        if self.board:
            self.get_logger().info(f'Physical servos commanded to (degrees): {self.current_angles_deg}')

        # --- Publish to Gazebo (optional) ---
        actuators_msg = Actuators()
        actuators_msg.position = target_positions_rad_for_gz
        self.gz_command_publisher.publish(actuators_msg)
        self.get_logger().info(f'Published joint commands for Gazebo (radians): {actuators_msg.position}')

    def destroy_node(self):
        if self.board:
            self.get_logger().info('Shutting down Arduino board.')
            self.board.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotManipulador()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The destroy_node method will handle board shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()