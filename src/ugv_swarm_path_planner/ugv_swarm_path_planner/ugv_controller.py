import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry


class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        # ---------------- Parameters ----------------
        self.lookahead_distance = 0.3
        self.max_linear_speed = 0.15
        self.max_angular_speed = 1.0
        self.angular_gain = 3.5
        self.goal_tolerance = 0.2
        self.waypoint_tolerance = 0.15  # NEW

        self.current_pose = None
        self.path = []

        # ---------------- Subscribers ----------------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/ugv/path',
            self.path_callback,
            10
        )

        # ---------------- Publisher ----------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("UGV Controller started")

    # ---------------- Callbacks ----------------

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.path = list(msg.poses)
        self.get_logger().info(f"New path received ({len(self.path)} points)")

    # ---------------- Control Loop ----------------

    def control_loop(self):

        if self.current_pose is None or not self.path:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        # 🔥 Prune path based on current position
        self.prune_path(rx, ry)

        if not self.path:
            self.stop_robot()
            return

        # Goal check
        gx = self.path[-1].pose.position.x
        gy = self.path[-1].pose.position.y
        distance_to_goal = math.hypot(gx - rx, gy - ry)

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            self.path = []
            return

        target = self.find_lookahead_point(rx, ry)

        if target is None:
            self.stop_robot()
            return

        tx, ty = target

        angle_to_target = math.atan2(ty - ry, tx - rx)
        heading_error = self.normalize_angle(angle_to_target - yaw)

        linear_speed = min(self.max_linear_speed,
                           0.5 * distance_to_goal)

        angular_speed = self.angular_gain * heading_error
        angular_speed = max(min(angular_speed,
                                self.max_angular_speed),
                            -self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self.cmd_pub.publish(cmd)

    # ---------------- Path Pruning ----------------

    def prune_path(self, rx, ry):
        """
        Remove waypoints that are already behind or within tolerance.
        """
        while self.path:
            px = self.path[0].pose.position.x
            py = self.path[0].pose.position.y

            distance = math.hypot(px - rx, py - ry)

            if distance < self.waypoint_tolerance:
                self.path.pop(0)
            else:
                break

    # ---------------- Helpers ----------------

    def find_lookahead_point(self, rx, ry):
        for pose in self.path:
            px = pose.pose.position.x
            py = pose.pose.position.y

            if math.hypot(px - rx, py - ry) >= self.lookahead_distance:
                return (px, py)

        return None

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def get_yaw(self, q):
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
