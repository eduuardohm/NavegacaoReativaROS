import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Bug1(Node):
    def __init__(self):
        super().__init__('bug1')

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Estado do robô
        self.state = "GOAL_SEEK"  # GOAL_SEEK ou WALL_FOLLOW
        self.pose = None
        self.ranges = []
        self.goal = (2.0, 2.0)  # Alvo (x, y) no mapa
        self.hit_point = None

        self.timer = self.create_timer(0.1, self.control_loop)

    # === CALLBACKS ===
    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.ranges = msg.ranges

    # === LÓGICA DE CONTROLE ===
    def control_loop(self):
        self.get_logger().info("Control loop started.")

        if self.pose is None:
            self.get_logger().info("Pose is None, skipping control.")
            return

        twist = Twist()

        # Distância ao alvo
        dx = self.goal[0] - self.pose.position.x
        dy = self.goal[1] - self.pose.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < 0.2:
            self.get_logger().info("Goal reached!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        if self.state == "GOAL_SEEK":
            if self.ranges and min(self.ranges) < 0.5:
                self.state = "WALL_FOLLOW"
                self.hit_point = (self.pose.position.x, self.pose.position.y)
                self.get_logger().info("Hit obstacle -> switching to WALL_FOLLOW")
            else:
                angle_to_goal = math.atan2(dy, dx)
                yaw = self.get_yaw()
                angle_error = angle_to_goal - yaw

                twist.linear.x = 0.15
                twist.angular.z = 1.0 * angle_error
                self.get_logger().info(f"Moving towards goal: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

                self.get_logger().info(f"Publishing to /cmd_vel: {twist}")  # Log de publicação do comando


                self.cmd_vel_pub.publish(twist)

        elif self.state == "WALL_FOLLOW":
            self.get_logger().info("WALL_FOLLOW state active.")
            if self.ranges and min(self.ranges) > 0.5:
                self.state = "GOAL_SEEK"
                self.get_logger().info("Path clear -> back to GOAL_SEEK")
            else:
                twist.linear.x = 0.1
                twist.angular.z = 0.3
                self.get_logger().info(f"Following wall: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
                self.cmd_vel_pub.publish(twist)

    def get_yaw(self):
        q = self.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = Bug1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()