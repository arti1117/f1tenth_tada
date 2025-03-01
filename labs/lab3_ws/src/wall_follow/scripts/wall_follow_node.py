#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.kp = 1.
        self.kd = 1.
        self.ki = 1.

        self.integral = 0.0
        self.prev_error = None
        self.error = None

        self.prev_ranges = None
        self.prev_scan_time = None

        self.get_logger().info("WallFollow Node Initialized")

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        alpha = np.arctan()
        #TODO: implement
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        # Dt+1 - Dt
        return 

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = self.kp * error + self.ki + self.kd
        # TODO: Use kp, ki & kd to implement a PID controller


        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0
        drive_msg.drive.steering_angle = angle
        
        # TODO: fill in drive message and publish
        self.drive_publisher.publish(drive_msg)

        self.get_logger().info(f"Published: Speed=, Steering Angle=")

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, float('inf')) # Handle NaNs and Infs

        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))


        error = self.get_error(ranges, None) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID

        # Store current scan for next iteration
        self.prev_ranges = ranges
        self.prev_scan_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9



def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()

    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        wall_follow_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()