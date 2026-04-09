import rclpy
from rclpy.node import Node

class ValidationPrintingNode(Node):
    def __init__(self):
        super().__init__('validation_printing_node')
        self.get_logger().info('Hello world')

def main(args=None):
    rclpy.init(args=args)
    node = ValidationPrintingNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
