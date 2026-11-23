#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import os


class LidarLoggerNode(Node):

    def __init__(self):
        super().__init__('lidar_logger_node')

        # Subscribe to LiDAR /scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # CSV file
        self.file_path = 'lidar_data.csv'

        self.file = open(
            self.file_path,
            'w',
            newline=''
        )

        self.writer = csv.writer(self.file)

        # CSV header
        self.writer.writerow([
            'angle_radians',
            'distance_meters'
        ])

        self.get_logger().info(
            'LiDAR Logger started!'
        )

        self.get_logger().info(
            f'Saving data to: {os.path.abspath(self.file_path)}'
        )

    def scan_callback(self, msg):

        # Number of LiDAR measurements
        num_readings = len(msg.ranges)

        for i in range(num_readings):

            # Calculate angle
            angle = (
                msg.angle_min +
                i * msg.angle_increment
            )

            # Get distance
            distance = msg.ranges[i]

            # Check whether measurement is valid
            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            # Write data to CSV
            self.writer.writerow([
                angle,
                distance
            ])

        # Force data to be written to file
        self.file.flush()

        self.get_logger().info(
            f'Saved {num_readings} LiDAR readings'
        )

    def destroy_node(self):

        # Close CSV file
        if hasattr(self, 'file'):
            self.file.close()

        super().destroy_node()


def main(args=None):

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = LidarLoggerNode()

    try:

        # Keep node running
        rclpy.spin(node)

    except KeyboardInterrupt:

        print('\nStopping LiDAR logger...')

    finally:

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

