# lab1_pkg/talker.py
import sys
sys.path.append('/home/kipp/ackermann_ws/install/ackermann_msgs/lib/python3.8/site-packages')
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('v', 1.0)
        self.declare_parameter('d', 0.0)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.publish_drive)

    def publish_drive(self):
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d
        self.drive_pub.publish(msg)
        self.get_logger().info(f'Publishing: speed={v:.2f}, steering={d:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
