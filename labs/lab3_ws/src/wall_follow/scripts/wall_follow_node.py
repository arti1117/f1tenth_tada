#! /usr/bin/env python3

import time
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

        # Create subscribers and publishers
        self.subscription = self.create_subscription(
            LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)

        # Set PID gains
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

        # Store any necessary values
        self.desired_distance = 1.0  # Desired distance to the wall
        self.angle_a = 45  # Angle for first LiDAR point
        self.angle_b = 90  # Angle for second LiDAR point
        self.max_speed = 2.0  # Maximum speed
        self.min_speed = 0.5  # Minimum speed

        self.get_logger().info("WallFollow Node Initialized")


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle (meters)
        """
        angle_index = int((np.radians(angle) - range_data.angle_min) / range_data.angle_increment)
        if 0 <= angle_index < len(range_data.ranges):
            range_value = range_data.ranges[angle_index]
            return range_value if np.isfinite(range_value) else 0.0
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
        dist_a = self.get_range(range_data, self.angle_a)
        dist_b = self.get_range(range_data, self.angle_b)

        theta = np.radians(self.angle_a - self.angle_b)
        if dist_b > 0:
            alpha = np.arctan((dist_a * np.cos(theta) - dist_b) / (dist_a * np.sin(theta)))
            Dt = dist_b * np.cos(alpha)
            error = dist - Dt
            return error
        return 0.0


    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        current_time = time.time()
        dt = current_time - self.prev_time if self.prev_time else 1e-6
        self.prev_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        angle = np.clip(angle, -0.3, 0.3)

        speed = max(self.min_speed, min(velocity, self.max_speed - abs(angle) * 5)) # Adjust speed based on angle

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.publisher.publish(drive_msg)

        self.get_logger().info(f"Published: Speed={speed:.2f}, Steering Anglel={angle:.2f}")


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance)
        velocity = max(self.min_speed, self.max_speed - abs(error) * 2)  # Adjust velocity based on error
        self.pid_control(error, velocity)


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