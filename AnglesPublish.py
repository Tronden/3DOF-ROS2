import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np  # Assuming some processing that results in angle data

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'angles', 10)
        self.timer = self.create_timer(1.0, self.publish_angles)
        self.get_logger().info("Angle Publisher initialized")

    def publish_angles(self):
        msg = Float64MultiArray()
        # Placeholder: generate angles based on your application logic
        msg.data = np.random.uniform(-60, 60, 3).tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing angles: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = AnglePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
