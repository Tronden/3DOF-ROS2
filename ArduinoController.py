import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            'angles',
            self.angle_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Modify as per your COM port
        self.get_logger().info("Arduino Controller initialized")

    def angle_callback(self, msg):
        angles_str = ','.join([f"{x:.2f}" for x in msg.data]) + '\n'
        self.serial_port.write(angles_str.encode('utf-8'))
        self.get_logger().info(f'Sent to Arduino: {angles_str}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
