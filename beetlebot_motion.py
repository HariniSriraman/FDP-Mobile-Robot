#!/usr/bin/env python3
"""BEETLEBOT/LYRA exam motion patterns and distance-limited LiDAR avoidance.
Source-derived interface: /cmd_vel_nav, /scan, /odometry/filtered, /lyra/armed.
NEW implementation: stop-on-failure, odometry goals, scan frame correction.
Dry-run by default. --motion explicitly permits nonzero Twist publication.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sector(scan, center_deg, half_deg=25):
    """Minimum finite valid range in a centered angular sector, or None."""
    center = math.radians(center_deg)
    half = math.radians(half_deg)
    hits = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        beam = scan.angle_min + i * scan.angle_increment
        if abs(wrap(beam - center)) <= half:
            hits.append(float(r))
    return min(hits) if hits else None


class Controller(Node):
    def __init__(self, a):
        super().__init__('beetlebot_exam_controller')
        self.a = a
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/lyra/armed', self.on_arm, qos_profile_sensor_data)
        self.create_timer(0.10, self.tick)
        self.pose = None
        self.yaw = None
        self.scan = None
        self.armed = False
        self.t_odom = 0.0
        self.t_scan = 0.0
        self.t_armed = 0.0
        self.started = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.reason = ''
        self.phase = 0
        self.phase_started = 0.0
        self.phase_pose = None
        self.phase_yaw = None
        self.turn_progress = 0.0
        self.prev_yaw = None
        self.avoid_state = 'FORWARD'
        self.block_since = None
        self.stall_since = self.started
        self.total_path = 0.0
        self.last_path_pose = None
        self.start_pose = None
        self.start_yaw = None
        self.last_progress = 0.0
        self.actions = self.make_actions()
        self.get_logger().info('MODE=%s MOTION=%s; distance=%s; lidar_yaw=%.0f' %
                               (a.mode, a.motion, a.goal_mode, a.lidar_yaw_deg))
        if not a.motion:
            self.get_logger().warn('DRY RUN: commands printed but nonzero velocity NEVER published')
        else:
            self.get_logger().warn('MOTION ENABLED: verify arm, front LiDAR angle, clear path and emergency stop')

    def make_actions(self):
        a = self.a
        f = lambda x: ('drive', x)
        t = lambda x: ('turn', x)
        if a.mode == 'forward': return [f(a.distance)]
        if a.mode == 'backward': return [f(-a.distance)]
        if a.mode == 'left': return [t(a.angle)]
        if a.mode == 'right': return [t(-a.angle)]
        if a.mode == 'uturn': return [t(180)]
        if a.mode == 'square': return [z for _ in range(4) for z in (f(a.side), t(a.turn_dir * 90))]
        if a.mode == 'rectangle':
            return [z for leg in (a.length, a.width, a.length, a.width)
                    for z in (f(leg), t(a.turn_dir * 90))]
        if a.mode == 'triangle': return [z for _ in range(3) for z in (f(a.side), t(a.turn_dir * 120))]
        if a.mode == 'zigzag':
            return [f(a.side), t(45), f(a.side), t(-90), f(a.side),
                    t(90), f(a.side), t(-90), f(a.side)]
        if a.mode == 'circle': return [('arc', a.turn_dir * 360)]
        if a.mode == 'figure8': return [('arc', 360), ('arc', -360)]
        return []

    def on_arm(self, msg):
        self.armed = bool(msg.data)
        self.t_armed = time.monotonic()

    def on_scan(self, msg):
        self.scan = msg
        self.t_scan = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        xy = (float(p.x), float(p.y))
        y = yaw_from_quat(msg.pose.pose.orientation)
        if self.last_path_pose is not None:
            step = math.dist(xy, self.last_path_pose)
            # Reject large localization jumps; do not count them as robot travel.
            if 0.003 < step < 0.20 and self.a.motion:
                self.total_path += step
        self.last_path_pose = xy
        if self.prev_yaw is not None and self.phase_started and self.a.mode != 'avoid':
            if self.phase < len(self.actions) and self.actions[self.phase][0] in ('turn', 'arc'):
                self.turn_progress += wrap(y - self.prev_yaw)
        self.prev_yaw = y
        self.pose = xy
        self.yaw = y
        self.t_odom = time.monotonic()
        if self.start_pose is None:
            self.start_pose, self.start_yaw = xy, y
            self.stall_since = time.monotonic()

    def stop(self, why):
        if not self.done:
            self.reason = why
            self.get_logger().info('STOP: ' + why)
        self.done = True
        self.send(0.0, 0.0)

    def send(self, v, w):
        requested = (v, w)
        # Avoidance and shapes have fixed, modest speeds; no bypass to /cmd_vel.
        if not self.a.motion:
            v, w = 0.0, 0.0
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        if time.monotonic() - self.last_log > 1.0:
            self.get_logger().info('phase=%d requested=(%.2f m/s, %.2f rad/s) sent=(%.2f,%.2f) path=%.2f m' %
                                   (self.phase, requested[0], requested[1], v, w, self.total_path))
            self.last_log = time.monotonic()

    def safety(self, now):
        if now - self.started > self.a.timeout:
            return 'overall timeout exceeded'
        if not self.pose or now - self.t_odom > 0.8:
            return 'odometry missing/stale'
        if self.a.motion and not self.armed:
            return 'robot not confirmed ARMED recently'
        if not self.a.no_scan_safety and (not self.scan or now - self.t_scan > 0.8):
            return 'LiDAR /scan missing/stale'
        return None

    def front(self):
        return sector(self.scan, -self.a.lidar_yaw_deg) if self.scan else None

    def scan_clear(self, for_turn=False, reverse=False):
        if self.a.no_scan_safety:
            return True
        # Standstill turns should not occur near any valid detected obstacle.
        if for_turn:
            vals = [r for r in self.scan.ranges
                    if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max]
            return bool(vals) and min(vals) > self.a.turn_clearance
        d = sector(self.scan, 180 - self.a.lidar_yaw_deg) if reverse else self.front()
        return d is not None and d > self.a.stop_distance

    def phase_init(self, now):
        self.phase_started = now
        self.phase_pose = self.pose
        self.phase_yaw = self.yaw
        self.turn_progress = 0.0
        self.get_logger().info('START SEGMENT %d/%d: %s' %
                               (self.phase + 1, len(self.actions), str(self.actions[self.phase])))

    def next_phase(self):
        self.send(0, 0)
        self.phase += 1
        self.phase_started = 0.0
        self.turn_progress = 0.0
        if self.phase >= len(self.actions):
            self.stop('all %d segments completed' % len(self.actions))

    def do_shape(self, now):
        if self.phase >= len(self.actions):
            self.stop('pattern complete')
            return
        if not self.phase_started:
            self.phase_init(now)
        kind, goal = self.actions[self.phase]
        if now - self.phase_started > self.a.segment_timeout:
            self.stop('segment timeout (odometry not advancing?)')
            return
        if kind == 'drive':
            dx = self.pose[0] - self.phase_pose[0]
            dy = self.pose[1] - self.phase_pose[1]
            signed = dx * math.cos(self.phase_yaw) + dy * math.sin(self.phase_yaw)
            if (1 if goal >= 0 else -1) * signed >= abs(goal) - 0.015:
                self.next_phase()
            elif self.scan_clear(reverse=(goal < 0)):
                self.send(math.copysign(self.a.speed, goal), 0.0)
            else:
                self.stop('obstacle in movement path or no valid front ranges')
        else:
            desired = math.radians(goal)
            if abs(self.turn_progress) >= abs(desired) - math.radians(3):
                self.next_phase()
            elif not self.scan_clear(for_turn=True):
                self.stop('not enough all-around clearance for turning')
            else:
                w = math.copysign(self.a.turn_speed, desired)
                if kind == 'arc':
                    self.send(self.a.arc_speed, w)
                else:
                    self.send(0.0, w)

    def progress(self):
        if self.a.goal_mode == 'path':
            return self.total_path
        dx = self.pose[0] - self.start_pose[0]
        dy = self.pose[1] - self.start_pose[1]
        return dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)

    def do_avoid(self, now):
        p = self.progress()
        if p >= self.a.distance - 0.02:
            self.stop('goal reached: %.2f m (%s)' % (p, self.a.goal_mode))
            return
        if p > self.last_progress + 0.04:
            self.stall_since = now
            self.last_progress = p
        if now - self.stall_since > self.a.stall_timeout:
            self.stop('no goal progress - obstacle trap/stall')
            return
        front = self.front()
        left = sector(self.scan, 90 - self.a.lidar_yaw_deg)
        right = sector(self.scan, -90 - self.a.lidar_yaw_deg)
        if front is None:
            self.stop('no VALID front-facing LiDAR ranges')
            return
        if front <= self.a.emergency_distance:
            self.stop('EMERGENCY: front obstacle %.2f m' % front)
            return
        if front > self.a.avoid_distance:
            self.block_since = None
            self.send(self.a.speed, 0)
            return
        if self.block_since is None:
            self.block_since = now
        if now - self.block_since > self.a.max_turn_time:
            self.stop('blocked too long - no safe path found')
            return
        l = left if left is not None else 0.0
        r = right if right is not None else 0.0
        if max(l, r) < self.a.side_clearance:
            self.send(0.0, 0.0)
            return
        self.send(0.0, self.a.turn_speed * (1 if l > r else -1))

    def tick(self):
        now = time.monotonic()
        if self.done:
            self.send(0, 0)
            return
        bad = self.safety(now)
        if bad:
            # Allow 2 seconds on startup for first odometry and scan samples.
            if now - self.started > 2.0:
                self.stop(bad)
            else:
                self.send(0, 0)
            return
        if self.a.mode == 'avoid':
            self.do_avoid(now)
        else:
            self.do_shape(now)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['forward','backward','left','right','uturn','square',
                    'rectangle','triangle','zigzag','circle','figure8','avoid'])
    p.add_argument('--motion', action='store_true', help='explicitly enable nonzero motor commands')
    p.add_argument('--distance', type=float, default=3.0, help='drive/avoid target metres')
    p.add_argument('--side', type=float, default=0.4, help='square/triangle side; zigzag leg metres')
    p.add_argument('--length', type=float, default=0.6)
    p.add_argument('--width', type=float, default=0.35)
    p.add_argument('--angle', type=float, default=90.0, help='left/right degrees')
    p.add_argument('--turn-dir', type=int, choices=(-1,1), default=1, help='+1 CCW (left), -1 CW')
    p.add_argument('--speed', type=float, default=0.10)
    p.add_argument('--turn-speed', type=float, default=0.30)
    p.add_argument('--arc-speed', type=float, default=0.05)
    p.add_argument('--distance-mode', dest='goal_mode', choices=('path','forward'), default='path',
                   help='avoid: cumulative path length OR net progress along start heading')
    p.add_argument('--lidar-yaw-deg', type=float, default=180.0,
                   help='LiDAR frame yaw relative to robot; source documents suggest ~180')
    p.add_argument('--avoid-distance', type=float, default=0.65)
    p.add_argument('--emergency-distance', type=float, default=0.30)
    p.add_argument('--side-clearance', type=float, default=0.55)
    p.add_argument('--stop-distance', type=float, default=0.50)
    p.add_argument('--turn-clearance', type=float, default=0.23)
    p.add_argument('--stall-timeout', type=float, default=18.0)
    p.add_argument('--max-turn-time', type=float, default=10.0)
    p.add_argument('--segment-timeout', type=float, default=30.0)
    p.add_argument('--timeout', type=float, default=120.0)
    p.add_argument('--no-scan-safety', action='store_true',
                   help='DANGEROUS: explicit lab-supervised exception for movement with no /scan')
    a = p.parse_args()
    if a.distance <= 0 or a.side <= 0 or a.speed <= 0 or a.turn_speed <= 0:
        p.error('positive distance, side and speeds are required')
    rclpy.init()
    node = Controller(a)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        node.stop('Ctrl+C')
    finally:
        for _ in range(10):
            node.send(0, 0)
            time.sleep(0.04)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
