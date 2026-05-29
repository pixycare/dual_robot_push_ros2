import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

from cv_bridge import CvBridge
import cv2

import sys
sys.path.append("/root/yolo_env/lib/python3.12/site-packages")

from ultralytics import YOLO

import numpy as np


class MultiRobotController(Node):

    def __init__(self):
        super().__init__('multi_robot_controller')
        
        self.bridge = CvBridge()
        
        self.yolo = YOLO("yolov8n.pt")

        self.robot_expected_pred = "traffic light"
        self.target_expected_pred = "stop sign"

        self.yolo_min_confidence = 0.25
        self.yolo_max_age_sec = 1.0

        self.lidar_front_angle_deg = 60.0
        self.lidar_front_half_angle_rad = np.deg2rad(self.lidar_front_angle_deg / 2.0)

        self.robot_ids = ["J0", "J1"]

        self.robots = {
            rid: {
                "odom": None,
                "lidar": None,
                "camera": None,
                "yolo_detections": {},
                "last_yolo_stamp": None,
                "state": "INIT",
                "ready": False,
                "last_angle": None,
                "last_dist": None,
            }
            for rid in self.robot_ids
        }

        self.cmd_vel_pub = {
            "J0": self.create_publisher(Twist, "/J0/cmd_vel", 10),
            "J1": self.create_publisher(Twist, "/J1/cmd_vel", 10),
        }

        for rid in self.robot_ids:

            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self.odom_cb(msg, r),
                10
            )

            self.create_subscription(
                LaserScan,
                f'/{rid}/lidar',
                lambda msg, r=rid: self.lidar_cb(msg, r), 
                10
            )
            
            self.create_subscription(
                Image,
                f'/{rid}/camera/image_raw',
                lambda msg, r=rid: self.camera_cb(msg, r),
                10
            )

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg, rid):
        self.robots[rid]["odom"] = msg

    def lidar_cb(self, msg, rid):
        self.robots[rid]["lidar"] = msg

        valid_front = []
        for i, r in enumerate(msg.ranges):
            if np.isinf(r) or np.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if self.is_front_lidar_angle(angle):
                valid_front.append(r)

        if valid_front:
            print(f"[{rid}] nearest front object: {min(valid_front):.2f} m")
    
    def camera_cb(self, msg, rid):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.robots[rid]["camera"] = frame

            h, w, _ = frame.shape

            print(f"[{rid}] camera frame: {w}x{h}")
            
            results = self.yolo(frame, verbose=False)[0]
            detections = {}
            
            for box in results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.yolo.names[cls]

                print(name, conf)

                if conf >= self.yolo_min_confidence:
                    detections[name] = max(detections.get(name, 0.0), conf)

            self.robots[rid]["yolo_detections"] = detections
            self.robots[rid]["last_yolo_stamp"] = self.get_clock().now().nanoseconds

            annotated = results.plot()
            
            # PODGLĄD
            cv2.imshow(rid, annotated)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f"[{rid}] camera conversion failed: {e}"
            )
        

    def control_loop(self):
        for rid in self.robot_ids:
            self.step_robot(rid)


    def step_robot(self, rid):

        robot = self.robots[rid]
        state = robot["state"]

        if state == "INIT":
            print(f"[{rid}] STATE: INIT - looking for other robot with YOLO")
            # Szuaknie drugiego robota za pomocą YOLO
            if self.robot_pred(rid):
                print(f"[{rid}] YOLO confirmed other robot as '{self.robot_expected_pred}'")
                # Jeżeli znajdzie - przejście do PREPARE
                print(f"[{rid}] INIT -> PREPARE")
                self.stop(rid)
                robot["state"] = "PREPARE"
                return
            # Jeżeli nie znajdzie, obraca się powoli
            self.rotate_slow(rid)
            return

        elif state == "PREPARE":

            print(f"[{rid}] STATE: PREPARE")

            if robot["lidar"] is None:
                return
            # Wykrycie odległości i kąta do drugiego robota za pomocą LiDAR
            angle, dist = self.detect_robot(rid)
            if angle is None or dist is None:
                self.rotate_slow(rid)
                return
            print(f"[{rid}] PREPARE robot: "
                f"angle={np.degrees(angle):.1f} deg "
                f"dist={dist:.2f} m")
            # Obrót do drugiego robota
            if abs(angle) > 0.08:
                self.rotate_to_angle(rid, angle)
                return
            # Podjazd do drugiego robota
            if dist > 0.40:
                self.drive_prepare(rid, angle, dist)
                return

            self.stop(rid)
            # Gotowe, czeka na drugiego robota
            robot["ready"] = True
            # Jeżeli oba gotowe, przejście do SEARCH
            if self.check_sync():
                print(">>> PREPARE COMPLETE -> SEARCH")

                for r in self.robot_ids:
                    self.robots[r]["state"] = "SEARCH"
                    self.robots[r]["ready"] = False
                    self.robots[r]["last_angle"] = None
                    self.robots[r]["last_dist"] = None

        elif state == "SEARCH":
            print(f"[{rid}] STATE: SEARCH - looking for target with YOLO")
            # Szukanie celu za pomocą YOLO
            if self.target_pred(rid):
                print(f"[{rid}] YOLO confirmed target as '{self.target_expected_pred}'")
                print(f"[{rid}] SEARCH -> ALIGN")
                self.stop(rid)
                robot["last_angle"] = None
                robot["last_dist"] = None
                # Jeżeli znajdzie - przejście do ALIGN
                robot["state"] = "ALIGN"
                return
            # Jeżeli nie znajdzie, obraca się powoli
            self.rotate_slow(rid)
            return

        elif state == "ALIGN":
            print(f"[{rid}] STATE: ALIGN")

            if robot["lidar"] is None:
                return
            # Wykrycie odległości i kąta do celu za pomocą LiDAR
            angle, dist = self.detect_target(rid)

            if angle is None:
                print(f"[{rid}] ALIGN -> SEARCH (lost by LiDAR)")
                # Jeżeli kąt jest zły, wraca do SEARCH
                robot["state"] = "SEARCH"
                return

            print(f"[{rid}] ALIGN -> angle: {np.degrees(angle):.2f} deg, dist: {dist:.2f} m")
            # Przejście do APPROACH, jeżeli obiekt na wprost
            if abs(angle) < 0.05:
                print(f"[{rid}] ALIGN -> APPROACH")
                self.stop(rid)
                robot["state"] = "APPROACH"
                return
            # Obrót do celu, jeżeli obiekt nie jest na wprost
            self.rotate_to_angle(rid, angle)

        elif state == "APPROACH":
            print(f"[{rid}] STATE: APPROACH")

            if robot["lidar"] is None:
                return
            # Wykrycie odległości i kąta do celu za pomocą LiDAR
            angle, dist = self.detect_target(rid)

            if angle is None:
                print(f"[{rid}] APPROACH -> SEARCH (lost by LiDAR)")
                # Jeżeli kąt jest zły, wraca do SEARCH
                robot["state"] = "SEARCH"
                return
            # Zmień na WAIT, jeżeli jest blisko celu
            if dist is not None and dist < 0.8:
                print(f"[{rid}] APPROACH -> WAIT (ready)")
                self.stop(rid)
                robot["ready"] = True
                robot["state"] = "WAIT"
                return
            # Podjazd do celu, korygując kąt
            if rid == "J0":
                self.drive_to_target(rid, angle - 20 * dist / 180)
            else:
                self.drive_to_target(rid, angle + 20 * dist / 180)

        elif state == "WAIT":

            print(f"[{rid}] STATE: WAIT")

            self.stop(rid)
            # Czekaj, aż oba roboty nie będą zsynchronizowane
            if self.check_sync():
                print(">>> BOTH ROBOTS READY -> PUSH")
                for r in self.robot_ids:
                    self.robots[r]["state"] = "PUSH"

        elif state == "PUSH":

            print(f"[{rid}] STATE: PUSH")

            msg = Twist()
            # Jedź prosto, pchając cel
            msg.linear.x = 0.25
            msg.angular.z = 0.0

            self.cmd_vel_pub[rid].publish(msg)

    def check_sync(self):
        return all(self.robots[r]["ready"] for r in self.robot_ids)

    # DETECTION

    def robot_pred(self, rid):
        return self.has_yolo_pred(rid, self.robot_expected_pred)

    def target_pred(self, rid):
        return self.has_yolo_pred(rid, self.target_expected_pred)

    def has_yolo_pred(self, rid, expected_pred):
        robot = self.robots[rid]
        detections = robot.get("yolo_detections", {})
        stamp = robot.get("last_yolo_stamp")

        if stamp is None:
            return False

        age_sec = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if age_sec > self.yolo_max_age_sec:
            return False

        conf = detections.get(expected_pred)
        return conf is not None and conf >= self.yolo_min_confidence

    def normalize_lidar_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def is_front_lidar_angle(self, angle):
        angle = self.normalize_lidar_angle(angle)
        return abs(angle) <= self.lidar_front_half_angle_rad

    def detect_target(self, rid):

        scan = self.robots[rid]["lidar"]

        points = []

        for i, r in enumerate(scan.ranges):

            if np.isinf(r) or np.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if not self.is_front_lidar_angle(angle):
                continue

            if 0.6 < r < 2:
                points.append((angle, r))

        if len(points) < 5:
            return self.robots[rid]["last_angle"], self.robots[rid]["last_dist"]

        angles = [self.normalize_lidar_angle(p[0]) for p in points]
        dists  = [p[1] for p in points]

        angle = np.median(angles)
        dist  = np.median(dists)
        self.robots[rid]["last_angle"] = angle
        self.robots[rid]["last_dist"] = dist
        return angle, dist

    def detect_robot(self, rid):

        scan = self.robots[rid]["lidar"]

        points = []

        for i, r in enumerate(scan.ranges):

            if np.isinf(r) or np.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if not self.is_front_lidar_angle(angle):
                continue

            if 0.05 < r < 0.6:
                points.append((angle, r))

        if len(points) < 3:
            return None, None

        angles = [self.normalize_lidar_angle(p[0]) for p in points]
        dists  = [p[1] for p in points]

        return np.median(angles), np.median(dists)

    # CONTROL
    def rotate_slow(self, rid):

        msg = Twist()
        if rid == "J0":
            msg.angular.z = -0.5
        else:
            msg.angular.z = 0.5
        self.cmd_vel_pub[rid].publish(msg)

    def stop(self, rid):

        msg = Twist()
        self.cmd_vel_pub[rid].publish(msg)

    def rotate_to_angle(self, rid, angle):

        msg = Twist()
        msg.angular.z = 1.5 * angle
        self.cmd_vel_pub[rid].publish(msg)

    def drive_to_target(self, rid, angle):
        msg = Twist()
        msg.linear.x = 0.25
        msg.angular.z = 1 * angle

        self.cmd_vel_pub[rid].publish(msg)

    def drive_prepare(self, rid, angle, dist):

        msg = Twist()

        msg.linear.x = min(0.08, 0.5 * (dist - 0.10))

        msg.angular.z = 0.8 * angle

        self.cmd_vel_pub[rid].publish(msg)


def main():
    rclpy.init()
    node = MultiRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
