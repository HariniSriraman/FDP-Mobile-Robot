import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.declare_parameter('speed', 0.15)      # m/s
        self.declare_parameter('duration', 2.0)    # s
        self.speed = float(self.get_parameter('speed').value)
        self.duration = float(self.get_parameter('duration').value)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.start_time = time.time()
        self.create_timer(0.05, self.tick)        # 20 Hz beats the 500 ms disarm
        self.get_logger().info(f'Forward {self.speed} m/s for {self.duration} s')

    def tick(self):
        t = Twist()
        if time.time() - self.start_time < self.duration:
            t.linear.x = self.speed
            self.pub.publish(t)
        else:
            self.pub.publish(Twist())
            self.get_logger().info('Done. Stopped.')
            raise SystemExit

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.02)


def main():
    rclpy.init()
    node = MoveForward()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
