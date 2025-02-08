import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Relay(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    rclpy.shutdown()

