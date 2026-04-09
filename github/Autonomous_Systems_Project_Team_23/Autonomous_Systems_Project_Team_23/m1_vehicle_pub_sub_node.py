import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VehiclePubSubNode(Node):
    def __init__(self):
        super().__init__('vehicle_pub_sub_node')
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriber for Odometry (position)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        # Timer to publish commands every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        msg.angular.z = 0.2 # Steer
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to /cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}')

    def odom_callback(self, msg):
        self.get_logger().info(f'Subscribed to /odom: Position X={msg.pose.pose.position.x}')

def main(args=None):
    rclpy.init(args=args)
    node = VehiclePubSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
