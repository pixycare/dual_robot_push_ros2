import rclpy
from rclpy.node import Node
#import cv2
#from std_msgs.msg import String
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist #, PointStamped
#from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np


class DanceNode(Node):

    def __init__(self):
        super().__init__('Dance')

        self.J1_odom = Odometry()
        self.J2_odom = Odometry()
        self.J1_lidar = LaserScan()
        self.J2_lidar = LaserScan()

        self.subscription_J1_odom = self.create_subscription(
            Odometry,
            '/J1/odom',
            self.J1_save_odom,
            10
        )

        self.subscription_J2_odom = self.create_subscription(
            Odometry,
            '/J2/odom',
            self.J2_save_odom,
            10
        )

        self.subscription_J1_lidar = self.create_subscription(
            LaserScan,
            '/J1/lidar',
            self.J1_save_lidar,
            10
        )

        self.subscription_J2_lidar = self.create_subscription(
            LaserScan,
            '/J2/lidar',
            self.J2_save_lidar,
            10
        )

        self.publisher_J1_cmd_vel = self.create_publisher(
            Twist,
            '/J1/cmd_vel',
            10
        )

        self.publisher_J2_cmd_vel = self.create_publisher(
            Twist,
            '/J2/cmd_vel',
            10
        )
        timer_T = 0.1
        self.timer_send_vel = self.create_timer(timer_T, self.send_vel)
        self.timer_print_odom = self.create_timer(20* timer_T, self.print_lidar)


    def J1_save_odom(self, msg):
        self.J1_odom = msg

    def J2_save_odom(self, msg):
        self.J2_odom = msg

    def J1_save_lidar(self, msg):
        self.J1_lidar = msg

    def J2_save_lidar(self, msg):
        self.J2_lidar = msg


    def send_vel(self):
        msg = Twist()
        msg.linear.x = 0.5  # Jedź do przodu
        msg.angular.z = 0.1 # Lekko skręcaj
        self.publisher_J1_cmd_vel.publish(msg)

        msg = Twist()
        msg.linear.x = 0.5  # Jedź do przodu
        msg.angular.z = -0.1 # Lekko skręcaj
        self.publisher_J2_cmd_vel.publish(msg)

    def print_odom(self):
        print("J1 pozycja:")
        print(self.J1_odom.pose.pose.position)
        print("J1 orientacja:")
        print(self.J1_odom.pose.pose.orientation)
        print("J2 pozycja:")
        print(self.J2_odom.pose.pose.position)
        print("J2 orientacja:")
        print(self.J2_odom.pose.pose.orientation)
        print("--------")

    def print_lidar(self):
        print("J1 lidar:")
        print(self.J1_lidar)

        print("J2 lidar:")
        print(self.J2_lidar)
        print("--------")



def main():
    print('Hi from project_package.')
    rclpy.init(args=None)
    node = DanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

