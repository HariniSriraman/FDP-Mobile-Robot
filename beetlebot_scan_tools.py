#!/usr/bin/env python3
"""BeetleBot lab /scan CSV logger and 10x10m static occupancy snapshot.
Runs without NumPy/matplotlib; grid is saved as PGM image. Stationary scanner only.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def ray(x0, y0, x1, y1):
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx: err += dx; y0 += sy


class ScanTools(Node):
    def __init__(self, a):
        super().__init__('beetlebot_exam_scan_tools')
        self.a = a
        self.count = 0
        self.grid_size = int(round(a.map_size / a.resolution))
        self.grid = [-1] * (self.grid_size * self.grid_size)
        self.file = None
        self.writer = None
        self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        self.path = Path(a.output).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if a.mode == 'log':
            self.file = self.path.open('w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['timestamp_s','beam_index','angle_rad','distance_m'])
        self.get_logger().info('Subscribed to /scan with sensor-data QoS; mode=%s output=%s' % (a.mode,self.path))

    def callback(self, msg):
        self.count += 1
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            theta = msg.angle_min + i * msg.angle_increment
            if self.a.mode == 'log':
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.writer.writerow((stamp, i, theta, r))
            else:
                # Static scan snapshot only. Never use this as a live world-fixed SLAM map.
                xc, yc = self.grid_size//2, self.grid_size//2
                x = xc + int(round((r*math.cos(theta))/self.a.resolution))
                y = yc + int(round((r*math.sin(theta))/self.a.resolution))
                if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
                    continue
                cells = list(ray(xc, yc, x, y))
                for u, v in cells[:-1]:
                    k = v*self.grid_size+u
                    if self.grid[k] != 100: self.grid[k] = 0
                self.grid[y*self.grid_size+x] = 100
        if self.file and self.count % 10 == 0:
            self.file.flush()
        if self.count % 20 == 0:
            self.get_logger().info('Received %d /scan messages' % self.count)

    def finish(self):
        if self.file:
            self.file.flush()
            self.file.close()
            return
        # Binary portable grey map: black occupied, white free, grey unknown.
        # Image pixels are upside-down from ROS coordinates; viewer orientation is illustrative.
        with self.path.open('wb') as h:
            h.write(('P5\n%d %d\n255\n' % (self.grid_size,self.grid_size)).encode())
            h.write(bytes(0 if val == 100 else 255 if val == 0 else 128
                          for val in self.grid))
        self.get_logger().info('Saved %dx%d PGM (unknown=grey, free=white, occupied=black)' %
                               (self.grid_size, self.grid_size))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=('log','grid'))
    p.add_argument('--seconds', type=float, default=15.0)
    p.add_argument('--output', default=None)
    p.add_argument('--map-size', type=float, default=10.0)
    p.add_argument('--resolution', type=float, default=0.05)
    a = p.parse_args()
    if a.output is None:
        a.output = '~/beetlebot_lab5/lidar_data.csv' if a.mode == 'log' else '~/beetlebot_lab5/occupancy_grid.pgm'
    if a.seconds <= 0 or a.map_size <= 0 or a.resolution <= 0:
        p.error('seconds, map-size and resolution must be positive')
    rclpy.init()
    node = ScanTools(a)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < a.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.get_logger().info('Done: %d scan messages, %s' % (node.count,node.path))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
