import rclpy
from rclpy.node import Node

import math
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        # ---------------- parameters ----------------
        self.lookahead_distance = 0.3    # meters
        self.linear_speed = 0.2           # m/s
        self.angular_gain = 0.1           # tuning parameter
        self.goal_tolerance = 0.3         # meters

        # ---------------- state ----------------
        self.current_pose = None
        self.path = []

        # ---------------- subscribers ----------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ugv/pose',
            self.pose_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/ugv/path',
            self.path_callback,
            10
        )

        # ---------------- publisher ----------------
        self.cmd_pub = self.create_publisher(
            Twist,
            'ugv/cmd_vel',
            10
        )

        # control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("UGV path controller started")


    # ---------------- callbacks ----------------

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.get_logger().info(f"New path received ({len(self.path)} points)")


    # ---------------- control loop ----------------

    def control_loop(self):
        if self.current_pose is None or not self.path:
            return

        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(
            self.current_pose.pose.orientation
        )

        # find lookahead point
        target = self.find_lookahead_point(rx, ry)

        if target is None:
            self.stop_robot()
            return

        tx, ty = target

        # heading error
        angle_to_target = math.atan2(ty - ry, tx - rx)
        heading_error = self.normalize_angle(angle_to_target - yaw)

        # goal check
        gx = self.path[-1].pose.position.x
        gy = self.path[-1].pose.position.y
        if math.hypot(gx - rx, gy - ry) < self.goal_tolerance:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            self.path = []
            return

        # compute velocities
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_gain * heading_error

        self.cmd_pub.publish(cmd)


    # ---------------- helpers ----------------

    def find_lookahead_point(self, rx, ry):
        for pose in self.path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            if math.hypot(px - rx, py - ry) >= self.lookahead_distance:
                return (px, py)
        return None


    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


    def get_yaw_from_quaternion(self, q):
        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
