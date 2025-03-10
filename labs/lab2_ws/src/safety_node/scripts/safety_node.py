#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Vehicle state variables
        self.speed = 0.
        self.closet_obstacle_distance = float('inf')
        self.prev_ranges = None # Store previous range measurements
        self.prev_scan_time = None # Store previous scan timestamp
        self.collision_threshold = 0.5 # Threshold for iTTC warning

        self.get_logger().info('Safety Node Initialized')


    def odom_callback(self, odom_msg):
        """
        Process Odometry messages to get current vehicle speed.
        """
        self.speed = odom_msg.twist.twist.linear.x

        self.get_logger().info(f'Current Speed: {self.speed:.2f} m/s')


    def scan_callback(self, scan_msg):
        """
        Process LaserScan message to determine closest obstacle distance.
        """
        ranges = np.array(scan_msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, float('inf')) # Handle NaNs and Infs

        self.closest_obstacle_distance = np.min(ranges) # Find the minimum range value

        self.get_logger().info(f'Furthest Obstacle Distance: {np.max(ranges):.2f} meters')
        self.get_logger().info(f'Closest Obstacle Distance: {self.closest_obstacle_distance:.2f} meters')

        # angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        # Compute range rate using velocity projection
        range_rates = self.speed * np.cos(angles)
        scan_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # Compute range rate using finite difference if previous scan is available
        if self.prev_ranges is not None and self.prev_scan_time is not None:
            delta_t = scan_time - self.prev_scan_time
            
            if delta_t > 0:
                finite_diff_rates = -(ranges - self.prev_ranges) / delta_t
                range_rates = np.where(finite_diff_rates < 0, finite_diff_rates, range_rates) # Use meaningful rate

        # Compute iTTC: iTTC = r / max(-r_dot, 0)
        range_rates[range_rates >= 0] = float('-inf') # Ensure valid calculations
        ittc_values = ranges / np.maximum(-range_rates, 1e-6) # Avoid division by zero

        self.get_logger().info(f'Range Rate Max Value: {np.max(range_rates)} / Range Rate Min Value: {np.min(range_rates)}')  
        self.get_logger().info(f'iTTC Max Value: {np.max(ittc_values)} / iTTC Min Value: {np.min(ittc_values)}')       

        # Check for imminent collisions
        if np.any(ittc_values < self.collision_threshold):
            self.get_logger().warn("COLLISION WARNIN: iTTC below threshold! Braking...")
            # self.publish_drive_command(0.0, 0.0) # Hald vehicle
        
        # Store current scan for next iteration
        self.prev_ranges = ranges
        self.prev_scan_time = scan_time


    def publish_drive_command(self, speed, steering_angle):
        """
        Publishes AckermannDriveStamped message to control the vehicle.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_publisher.publish(drive_msg)

        self.get_logger().info(f'Published: Speed={speed:.2f}, Steering Angle={steering_angle:.2f}')


# Destory the node explicitly
# (optional - otherwise it will be done automatically
# when the garbage collector destroys the node object)
def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()

    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
