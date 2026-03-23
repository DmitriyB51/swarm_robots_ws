import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry


class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        self.declare_parameter("max_linear_speed", 0.7)
        self.declare_parameter("max_angular_speed", 6.0)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("lookahead", 0.4)

        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.lookahead = self.get_parameter("lookahead").value

        self.current_pose = None
        self.path = []

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, 'path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("UGV Controller started")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.path = list(msg.poses)
        self.get_logger().info(f"New path received ({len(self.path)} points)")

    def control_loop(self):
        if self.current_pose is None:
            return

        if not self.path:
            self.cmd_pub.publish(Twist())
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        # Find lookahead target on path
        tx, ty = self.get_lookahead_point(rx, ry)

        # Goal check
        gx = self.path[-1].pose.position.x
        gy = self.path[-1].pose.position.y
        if math.hypot(gx - rx, gy - ry) < self.goal_tolerance:
            self.get_logger().info("Goal reached")
            self.cmd_pub.publish(Twist())
            self.path = []
            return

        # Heading to target
        heading_error = self.normalize_angle(
            math.atan2(ty - ry, tx - rx) - yaw)

        # Angular
        angular = self.max_angular_speed * (2.0 / math.pi) * heading_error
        angular = max(-self.max_angular_speed,
                      min(self.max_angular_speed, angular))

        # Linear — full speed when aligned, zero when sideways
        linear = self.max_linear_speed * max(0.0, math.cos(heading_error))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def get_lookahead_point(self, rx, ry):
        """Walk along the path and return the first point >= lookahead away."""
        # Drop points that are behind the robot
        while len(self.path) > 1:
            px = self.path[0].pose.position.x
            py = self.path[0].pose.position.y
            # Only drop if the NEXT point is closer (we've passed this one)
            nx = self.path[1].pose.position.x
            ny = self.path[1].pose.position.y
            if math.hypot(px - rx, py - ry) < math.hypot(nx - rx, ny - ry):
                break
            self.path.pop(0)

        # Walk forward to find lookahead point
        for pose in self.path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            if math.hypot(px - rx, py - ry) >= self.lookahead:
                return px, py

        # Default: last point
        return (self.path[-1].pose.position.x,
                self.path[-1].pose.position.y)

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
