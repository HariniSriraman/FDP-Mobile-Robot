#!/usr/bin/env python3
"""Camera practical: GREEN GO / RED STOP or BLACK ARROW LEFT/RIGHT.
Requires ROS 2 Jazzy, python3-cv2 and python3-numpy. Uses /camera/image_raw/compressed.
LiDAR safety and armed status are mandatory before motor movement.
Arrows trigger ONE odometry-measured 90-degree turn per new arrow presentation.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
try:
    import numpy as np
    import cv2
except ImportError as e:
    raise SystemExit('Missing cv2/numpy. Test: python3 -c "import cv2,numpy"') from e


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Vision(Node):
    def __init__(self, a):
        super().__init__('beetlebot_exam_vision')
        self.a = a
        self.cmd = self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.status = self.create_publisher(String,'/beetlebot_exam/vision_state',10)
        self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.on_image,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/filtered',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Bool,'/lyra/armed',self.on_armed,qos_profile_sensor_data)
        self.create_timer(0.1,self.tick)
        self.armed = False
        self.image_time = 0.0
        self.scan_time = 0.0
        self.odom_time = 0.0
        self.scan = None
        self.state = 'NONE'
        self.candidate = 'NONE'
        self.confirm = 0
        self.yaw = None
        self.last_yaw = None
        self.turn_progress = 0.0
        self.turn_direction = 0
        self.latch = False
        self.started = time.monotonic()
        self.last_log = 0.0
        self.get_logger().info('VISION %s; motion %s; camera compressed; front lidar yaw %s' %
                               (a.mode,a.motion,a.lidar_yaw_deg))

    def on_armed(self,m): self.armed = bool(m.data)
    def on_scan(self,m): self.scan=m; self.scan_time=time.monotonic()
    def on_odom(self,m):
        cur = yaw(m.pose.pose.orientation)
        if self.last_yaw is not None and self.turn_direction:
            self.turn_progress += wrap(cur-self.last_yaw)
        self.last_yaw = cur
        self.yaw = cur
        self.odom_time = time.monotonic()

    def on_image(self,m):
        try:
            frame=cv2.imdecode(np.frombuffer(m.data,dtype=np.uint8),cv2.IMREAD_COLOR)
            if frame is None: return
            if self.a.mode == 'traffic':
                hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                mask_g=cv2.inRange(hsv,(35,70,55),(85,255,255))
                mask_r=cv2.bitwise_or(cv2.inRange(hsv,(0,75,60),(12,255,255)),
                                      cv2.inRange(hsv,(170,75,60),(179,255,255)))
                fraction=frame.shape[0]*frame.shape[1]*self.a.color_fraction
                red=cv2.countNonZero(mask_r)>fraction
                green=cv2.countNonZero(mask_g)>fraction
                result='RED' if red else 'GREEN' if green else 'NONE'
            else:
                # Silhouette heuristic: black arrow on white paper, centered, horizontal.
                im=cv2.resize(frame,(320,240))
                gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
                _,bw=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
                n,labels,stats,cent=cv2.connectedComponentsWithStats(bw)
                result='NONE'
                if n>1:
                    areas=stats[1:,cv2.CC_STAT_AREA]
                    label=int(np.argmax(areas))+1
                    x,y,w,h,area=stats[label].tolist()
                    if area >= self.a.min_arrow_area and w > 1.3*h and x>2 and x+w<318:
                        silhouette=(labels[y:y+h,x:x+w]==label)
                        cut=max(1,w//3)
                        left=int(np.count_nonzero(silhouette[:,:cut]))
                        right=int(np.count_nonzero(silhouette[:,-cut:]))
                        if left>right*1.25: result='LEFT'
                        elif right>left*1.25: result='RIGHT'
            self.image_time=time.monotonic()
            if result == self.candidate: self.confirm+=1
            else: self.candidate=result; self.confirm=1
            if self.confirm>=3:
                if self.state != result:
                    self.get_logger().info('CAMERA DETECTED: %s (dry-run allowed)' % result)
                self.state=result
                self.status.publish(String(data=result))
                if result=='NONE' and not self.turn_direction:
                    self.latch=False
        except Exception as exc:
            self.get_logger().error('Image decode/detect failed: %s' % exc)
            self.state='NONE'

    def front(self):
        if self.scan is None: return None
        hits=[]
        center=math.radians(-self.a.lidar_yaw_deg)
        for i,r in enumerate(self.scan.ranges):
            theta=self.scan.angle_min+i*self.scan.angle_increment
            if (math.isfinite(r) and self.scan.range_min<=r<=self.scan.range_max
                    and abs(wrap(theta-center)) < math.radians(25)):
                hits.append(r)
        return min(hits) if hits else None

    def publish(self,v=0.0,w=0.0):
        t=Twist()
        if self.a.motion: t.linear.x=v; t.angular.z=w
        self.cmd.publish(t)

    def tick(self):
        now=time.monotonic()
        if now-self.started > self.a.timeout:
            self.publish(); return
        # Fail closed for stale image, LiDAR, odometry, or disarmed robot.
        if not self.armed or now-self.image_time>0.8 or now-self.scan_time>0.8 or (self.a.mode=='arrow' and now-self.odom_time>0.8):
            self.publish(); return
        d=self.front()
        if d is None or d<=self.a.stop_distance:
            self.publish(); return
        if self.a.mode=='arrow':
            around=[r for r in self.scan.ranges if math.isfinite(r)
                    and self.scan.range_min <= r <= self.scan.range_max]
            if not around or min(around) <= 0.23:
                self.publish(); return
        if self.a.mode=='traffic':
            self.publish(self.a.speed if self.state=='GREEN' else 0.0,0.0)
        else:
            if self.turn_direction:
                if self.turn_direction*self.turn_progress >= math.radians(self.a.angle)-math.radians(3):
                    self.turn_direction=0; self.publish()
                else:
                    self.publish(0.0,self.turn_direction*self.a.turn_speed)
            elif not self.latch and self.state in ('LEFT','RIGHT'):
                self.turn_direction=1 if self.state=='LEFT' else -1
                self.turn_progress=0.0
                self.latch=True
                self.publish(0.0,self.turn_direction*self.a.turn_speed)
            else: self.publish()
        if now-self.last_log>1:
            self.get_logger().info('vision=%s front=%s turn=%s' % (self.state,round(d,2),self.turn_direction))
            self.last_log=now


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=('traffic','arrow'))
    p.add_argument('--motion',action='store_true')
    p.add_argument('--timeout',type=float,default=120)
    p.add_argument('--speed',type=float,default=0.10)
    p.add_argument('--turn-speed',type=float,default=0.25)
    p.add_argument('--angle',type=float,default=90)
    p.add_argument('--stop-distance',type=float,default=0.5)
    p.add_argument('--lidar-yaw-deg',type=float,default=180)
    p.add_argument('--color-fraction',type=float,default=0.025)
    p.add_argument('--min-arrow-area',type=int,default=800)
    a=p.parse_args()
    rclpy.init(); n=Vision(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        for _ in range(10): n.publish(); time.sleep(0.04)
        n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
