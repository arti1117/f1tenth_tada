# lab1_pkg/relay.py
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive_relay', 10)
        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/drive', self.callback, 10)

    def callback(self, msg):
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3.0
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3.0
        self.drive_pub.publish(new_msg)
        self.get_logger().info(f'Relay: speed={new_msg.drive.speed:.2f}, steering={new_msg.drive.steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
