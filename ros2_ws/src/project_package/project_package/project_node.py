import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np


class MultiRobotController(Node):

    def __init__(self):
        super().__init__('multi_robot_controller')

        self.robot_ids = ["J0", "J1"]

        self.robots = {
            rid: {
                "odom": None,
                "lidar": None,
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

        self.timer = self.create_timer(0.1, self.control_loop)

    # =========================================================
    # CALLBACKS
    # =========================================================

    def odom_cb(self, msg, rid):
        self.robots[rid]["odom"] = msg

    # def lidar_cb(self, msg, rid):
    #     self.robots[rid]["lidar"] = msg
    
    def lidar_cb(self, msg, rid):
        self.robots[rid]["lidar"] = msg

        valid = [r for r in msg.ranges if not np.isinf(r) and not np.isnan(r)]

        if valid:
            print(f"[{rid}] nearest object: {min(valid):.2f} m")

    # =========================================================
    # LOOP
    # =========================================================

    def control_loop(self):
        for rid in self.robot_ids:
            self.step_robot(rid)

    # =========================================================
    # FSM
    # =========================================================

    def step_robot(self, rid):

        robot = self.robots[rid]

        if robot["lidar"] is None:
            return

        state = robot["state"]

        if state == "INIT":
            print(f"[{rid}] INIT -> PREPARE")
            robot["state"] = "PREPARE"
        elif state == "PREPARE":

            print(f"[{rid}] STATE: PREPARE")

            angle, dist = self.detect_robot(rid)

            # robot not seen
            if angle is None or dist is None:
                self.rotate_slow(rid)
                return

            print(f"[{rid}] PREPARE target: "
                f"angle={np.degrees(angle):.1f} deg "
                f"dist={dist:.2f} m")

            # first align
            if abs(angle) > 0.08:
                self.rotate_to_angle(rid, angle)
                return

            # then move closer
            if dist > 0.40:
                self.drive_prepare(rid, angle, dist)
                return

            self.stop(rid)

            robot["ready"] = True

            if self.check_sync():
                print(">>> PREPARE COMPLETE -> SEARCH")

                for r in self.robot_ids:
                    self.robots[r]["state"] = "SEARCH"
                    self.robots[r]["ready"] = False



        elif state == "SEARCH":
            print(f"[{rid}] STATE: SEARCH")

            angle, dist = self.detect_target(rid)

            if angle is not None:
                print(f"[{rid}] SEARCH -> ALIGN")
                robot["state"] = "ALIGN"
                return

            if dist is not None and dist < 0.2:
                print(f"[{rid}] SEARCH -> APPROACH (close memory)")
                robot["state"] = "APPROACH"
                return

            self.rotate_slow(rid)

        elif state == "ALIGN":
            # print(f"[{rid}] STATE: ALIGN")

            angle, dist = self.detect_target(rid)

            if angle is None:
                print(f"[{rid}] ALIGN -> SEARCH (lost)")
                robot["state"] = "SEARCH"
                return
            print(f"[{rid}] ALIGN -> angle: {np.degrees(angle):.2f} deg, dist: {dist:.2f} m")
            if abs(angle) < 0.05:
                print(f"[{rid}] ALIGN -> APPROACH")
                self.stop(rid)
                robot["state"] = "APPROACH"
                return

            self.rotate_to_angle(rid, angle)

        elif state == "APPROACH":
            print(f"[{rid}] STATE: APPROACH")

            angle, dist = self.detect_target(rid)

            if angle is None:
                print(f"[{rid}] APPROACH -> SEARCH (lost)")
                robot["state"] = "SEARCH"
                return

            if dist is not None and dist < 0.8:                
                print(f"[{rid}] APPROACH -> WAIT (ready)")
                self.stop(rid)
                robot["ready"] = True
                robot["state"] = "WAIT"
                return
            if rid == "J0":
                self.drive_to_target(rid, angle-20*dist/180)
            else:
                self.drive_to_target(rid, angle+20*dist/180)
        elif state == "WAIT":

            print(f"[{rid}] STATE: WAIT")

            self.stop(rid)

            if self.check_sync():
                print(">>> BOTH ROBOTS READY -> PUSH")
                for r in self.robot_ids:
                    self.robots[r]["state"] = "PUSH"

        elif state == "PUSH":

            print(f"[{rid}] STATE: PUSH")

            msg = Twist()

            msg.linear.x = 0.25
            msg.angular.z = 0.0

            self.cmd_vel_pub[rid].publish(msg)

    # =========================================================
    # SYNC
    # =========================================================

    def check_sync(self):
        return all(self.robots[r]["ready"] for r in self.robot_ids)

    # =========================================================
    # SIMPLE LIDAR DETECTION
    # =========================================================

    def detect_target(self, rid):

        scan = self.robots[rid]["lidar"]

        points = []

        for i, r in enumerate(scan.ranges):

            if np.isinf(r) or np.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle_deg = np.degrees(angle)


            if 0.6 < r < 2:
                points.append((angle, r))

        if len(points) < 5:
            return self.robots[rid]["last_angle"], self.robots[rid]["last_dist"]

        angles = [p[0] for p in points]
        dists  = [p[1] for p in points]

        angle = np.median(angles)
        dist  = np.median(dists)
        # print("-> angle (deg):", np.median(angles), "dist:", np.median(dists))
        self.robots[rid]["last_angle"] = np.median(angles)
        self.robots[rid]["last_dist"] = np.median(dists)
        return np.median(angles), np.median(dists)

    def detect_robot(self, rid):

        scan = self.robots[rid]["lidar"]

        points = []

        for i, r in enumerate(scan.ranges):

            if np.isinf(r) or np.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle_deg = np.degrees(angle)


            if 0.05 < r < 0.6:
                points.append((angle, r))

        if len(points) < 3:
            return None, None

        angles = [p[0] for p in points]
        dists  = [p[1] for p in points]

        return np.median(angles), np.median(dists)
    # # =========================================================
    # # SECTORS
    # # =========================================================

    # def is_in_sector(self, rid, angle_deg):

    #     if rid == "J1":
    #         return -70 < angle_deg < 20
    #     else:
    #         return -20 < angle_deg < 70

    # =========================================================
    # CONTROL
    # =========================================================

    def rotate_slow(self, rid):

        msg = Twist()
        if rid == "J0":
            msg.angular.z = 0.5
        else:
            msg.angular.z = -0.5
        self.cmd_vel_pub[rid].publish(msg)

    def stop(self, rid):

        msg = Twist()
        self.cmd_vel_pub[rid].publish(msg)

    def rotate_to_angle(self, rid, angle):

        msg = Twist()
        msg.angular.z = 1.5 * (angle)
        self.cmd_vel_pub[rid].publish(msg)

    def drive_to_target(self, rid, angle):
        msg = Twist()
        msg.linear.x = 0.25
        msg.angular.z = 1 * (angle)

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