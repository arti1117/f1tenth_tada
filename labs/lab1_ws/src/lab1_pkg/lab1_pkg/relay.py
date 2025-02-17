#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )

        # Publisher to 'drive_relay' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

        self.get_logger().info('Ackermann Relay Node started.')

    def listener_callback(self, msg):
        # Modify the received message
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3.0
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3.0

        # Publish the modified message
        self.publisher_.publish(new_msg)

        # Log the transformation
        self.get_logger().info(f'Relayed: speed={new_msg.drive.speed}, steering_angle={new_msg.drive.steering_angle}')


def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()