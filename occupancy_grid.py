#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np
import math
import matplotlib

# Use a non-GUI backend.
# This is important for headless robots such as VeeRobot.
matplotlib.use('Agg')

import matplotlib.pyplot as plt


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid_node')

        # ==========================================
        # MAP PARAMETERS
        # ==========================================

        self.map_size = 10.0       # meters
        self.resolution = 0.05     # meters/cell

        self.grid_size = int(
            self.map_size / self.resolution
        )

        # ==========================================
        # OCCUPANCY GRID
        # ==========================================

        # -1   = Unknown
        #  0   = Free
        # 100  = Occupied

        self.grid = np.full(
            (self.grid_size, self.grid_size),
            -1,
            dtype=np.int8
        )

        # ==========================================
        # ROBOT POSITION
        # ==========================================

        # Robot is assumed to be at the center
        # of the map.

        self.robot_x = self.map_size / 2.0
        self.robot_y = self.map_size / 2.0

        # Convert robot position to grid coordinates
        self.robot_row, self.robot_col = self.world_to_grid(
            self.robot_x,
            self.robot_y
        )

        # ==========================================
        # SUBSCRIBE TO LIDAR
        # ==========================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ==========================================
        # SAVE MAP TIMER
        # ==========================================

        # Save the map every 5 seconds.

        self.save_timer = self.create_timer(
            5.0,
            self.save_grid_image
        )

        # ==========================================
        # INFORMATION
        # ==========================================

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'Occupancy Grid Node Started'
        )

        self.get_logger().info(
            f'Map size: {self.map_size} m x {self.map_size} m'
        )

        self.get_logger().info(
            f'Resolution: {self.resolution} m/cell'
        )

        self.get_logger().info(
            f'Grid size: {self.grid_size} x {self.grid_size}'
        )

        self.get_logger().info(
            'Subscribing to /scan'
        )

        self.get_logger().info(
            'Map will be saved every 5 seconds.'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ==================================================
    # WORLD → GRID
    # ==================================================

    def world_to_grid(self, x, y):

        """
        Convert world coordinates in meters
        to occupancy-grid row and column.
        """

        column = int(x / self.resolution)
        row = int(y / self.resolution)

        return row, column

    # ==================================================
    # CHECK GRID CELL
    # ==================================================

    def is_inside_grid(self, row, column):

        return (
            0 <= row < self.grid_size
            and
            0 <= column < self.grid_size
        )

    # ==================================================
    # RAY TRACE
    # ==================================================

    def raytrace(self, start_row, start_col, end_row, end_col):

        """
        Bresenham line algorithm.

        Marks cells between robot and obstacle as FREE.
        """

        x0 = start_col
        y0 = start_row

        x1 = end_col
        y1 = end_row

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        error = dx - dy

        while True:

            # Stop before the obstacle cell.
            # The obstacle itself will be marked occupied.

            if x0 == x1 and y0 == y1:
                break

            if self.is_inside_grid(y0, x0):

                # Mark as free
                self.grid[y0, x0] = 0

            e2 = 2 * error

            if e2 > -dy:
                error -= dy
                x0 += sx

            if e2 < dx:
                error += dx
                y0 += sy

    # ==================================================
    # LIDAR CALLBACK
    # ==================================================

    def scan_callback(self, msg):

        for i, distance in enumerate(msg.ranges):

            # ==========================================
            # CHECK INVALID MEASUREMENT
            # ==========================================

            if math.isnan(distance):
                continue

            if math.isinf(distance):
                continue

            if distance < msg.range_min:
                continue

            # ==========================================
            # LIMIT TO MAP SIZE
            # ==========================================

            # If the LiDAR measurement is beyond
            # the map, we cannot represent the hit.

            if distance > msg.range_max:
                continue

            # ==========================================
            # ANGLE
            # ==========================================

            angle = (
                msg.angle_min
                +
                i * msg.angle_increment
            )

            # ==========================================
            # POLAR → CARTESIAN
            # ==========================================

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # ==========================================
            # SHIFT ROBOT TO MAP CENTER
            # ==========================================

            map_x = self.robot_x + x
            map_y = self.robot_y + y

            # ==========================================
            # WORLD → GRID
            # ==========================================

            end_row, end_col = self.world_to_grid(
                map_x,
                map_y
            )

            # ==========================================
            # CHECK MAP BOUNDARY
            # ==========================================

            if not self.is_inside_grid(
                end_row,
                end_col
            ):
                continue

            # ==========================================
            # MARK FREE SPACE
            # ==========================================

            self.raytrace(
                self.robot_row,
                self.robot_col,
                end_row,
                end_col
            )

            # ==========================================
            # MARK OBSTACLE
            # ==========================================

            self.grid[
                end_row,
                end_col
            ] = 100

    # ==================================================
    # SAVE GRID IMAGE
    # ==================================================

    def save_grid_image(self):

        """
        Save occupancy grid as PNG.

        Color representation:

        Black  = Occupied
        White  = Free
        Gray   = Unknown
        Red    = Robot
        """

        filename = '/home/veerobot/occupancy_grid.png'

        # ==========================================
        # CREATE FIGURE
        # ==========================================

        fig, ax = plt.subplots(
            figsize=(8, 8)
        )

        # ==========================================
        # DISPLAY GRID
        # ==========================================

        ax.imshow(
            self.grid,
            cmap='gray',
            origin='lower',
            extent=[
                0,
                self.map_size,
                0,
                self.map_size
            ],
            vmin=-1,
            vmax=100
        )

        # ==========================================
        # ROBOT POSITION
        # ==========================================

        ax.plot(
            self.robot_x,
            self.robot_y,
            'r^',
            markersize=12
        )

        # ==========================================
        # AXIS
        # ==========================================

        ax.set_xlabel(
            'X (meters)'
        )

        ax.set_ylabel(
            'Y (meters)'
        )

        ax.set_title(
            'VeeRobot LiDAR Occupancy Grid'
        )

        # ==========================================
        # GRID LINES
        # ==========================================

        ax.grid(
            True,
            color='blue',
            alpha=0.2,
            linewidth=0.5
        )

        # ==========================================
        # KEEP ASPECT RATIO
        # ==========================================

        ax.set_aspect(
            'equal'
        )

        # ==========================================
        # SAVE IMAGE
        # ==========================================

        plt.savefig(
            filename,
            dpi=200,
            bbox_inches='tight'
        )

        plt.close(fig)

        self.get_logger().info(
            f'Occupancy image saved: {filename}'
        )

    # ==================================================
    # SAVE NUMERICAL GRID
    # ==================================================

    def save_grid_numpy(self):

        filename = '/home/veerobot/occupancy_grid.npy'

        np.save(
            filename,
            self.grid
        )

        self.get_logger().info(
            f'Occupancy grid saved: {filename}'
        )


# ======================================================
# MAIN
# ======================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = OccupancyGridNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Stopping occupancy grid node...'
        )

    finally:

        # ==========================================
        # FINAL SAVE
        # ==========================================

        node.save_grid_image()

        node.save_grid_numpy()

        node.destroy_node()

        rclpy.try_shutdown()


# ======================================================
# PROGRAM ENTRY
# ======================================================

if __name__ == '__main__':
    main()

