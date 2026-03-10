import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry


class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        # Parameters
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("heading_gain", 2.0)
        self.declare_parameter("cross_track_gain", 2.0)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("waypoint_tolerance", 0.5)

        self.max_linear_speed = self.get_parameter(
            "max_linear_speed").value
        self.max_angular_speed = self.get_parameter(
            "max_angular_speed").value
        self.heading_gain = self.get_parameter(
            "heading_gain").value
        self.cross_track_gain = self.get_parameter(
            "cross_track_gain").value
        self.goal_tolerance = self.get_parameter(
            "goal_tolerance").value
        self.waypoint_tolerance = self.get_parameter(
            "waypoint_tolerance").value

        self.current_pose = None
        self.path = []

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            'path',
            self.path_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("UGV Controller started")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.path = list(msg.poses)
        self.get_logger().info(f"New path received ({len(self.path)} points)")

    def control_loop(self):

        if self.current_pose is None or not self.path:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        self.prune_path(rx, ry)

        if not self.path:
            self.stop_robot()
            return

        gx = self.path[-1].pose.position.x
        gy = self.path[-1].pose.position.y
        distance_to_goal = math.hypot(gx - rx, gy - ry)

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            self.path = []
            return

        # Stanley-style: align with path direction + correct cross-track error
        cross_track, path_heading = self.find_nearest_segment(rx, ry)
        heading_error = self.normalize_angle(path_heading - yaw)

        # Linear speed: slow near goal, stop when facing wrong way
        linear_speed = min(self.max_linear_speed,
                           0.5 * distance_to_goal)
        linear_speed *= max(0.0, math.cos(heading_error))

        # Angular: heading alignment + cross-track correction
        angular_speed = self.heading_gain * heading_error
        if linear_speed > 0.01:
            angular_speed += math.atan2(
                self.cross_track_gain * cross_track,
                linear_speed
            )
        angular_speed = max(min(angular_speed,
                                self.max_angular_speed),
                            -self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self.cmd_pub.publish(cmd)

    def prune_path(self, rx, ry):
        while self.path:
            px = self.path[0].pose.position.x
            py = self.path[0].pose.position.y
            if math.hypot(px - rx, ry - py) < self.waypoint_tolerance:
                self.path.pop(0)
            else:
                break

    def find_nearest_segment(self, rx, ry):
        """Return (signed_cross_track, path_heading) for the nearest segment."""
        if len(self.path) < 2:
            px = self.path[0].pose.position.x
            py = self.path[0].pose.position.y
            return 0.0, math.atan2(py - ry, px - rx)

        min_dist_sq = float('inf')
        best_idx = 0

        for i in range(len(self.path) - 1):
            ax = self.path[i].pose.position.x
            ay = self.path[i].pose.position.y
            bx = self.path[i + 1].pose.position.x
            by = self.path[i + 1].pose.position.y

            abx, aby = bx - ax, by - ay
            apx, apy = rx - ax, ry - ay
            ab_sq = abx * abx + aby * aby

            if ab_sq < 1e-6:
                dist_sq = apx * apx + apy * apy
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                px = ax + t * abx
                py = ay + t * aby
                dist_sq = (rx - px) ** 2 + (ry - py) ** 2

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_idx = i

        ax = self.path[best_idx].pose.position.x
        ay = self.path[best_idx].pose.position.y
        bx = self.path[best_idx + 1].pose.position.x
        by = self.path[best_idx + 1].pose.position.y

        dx, dy = bx - ax, by - ay
        path_heading = math.atan2(dy, dx)

        # Signed cross-track: positive = robot is right of path (needs left turn)
        path_len = math.hypot(dx, dy)
        if path_len > 1e-6:
            cross_track = (dy * (rx - ax) - dx * (ry - ay)) / path_len
        else:
            cross_track = 0.0

        return cross_track, path_heading

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

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
