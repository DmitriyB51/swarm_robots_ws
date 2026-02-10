import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.time import Time


class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        if self.limit is not None:
            output = max(min(output, self.limit), -self.limit)

        return output


class DroneSlaveController(Node):

    def __init__(self):
        super().__init__('drone_slave_controller')

        # subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )

        # publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID
        self.pid_x = PID(kp=1.0, ki=0.0, kd=0.3, limit=2.0)
        self.pid_y = PID(kp=1.0, ki=0.0, kd=0.3, limit=2.0)
        self.pid_z = PID(kp=1.2, ki=0.0, kd=0.4, limit=1.5)

        self.goal_pose = None
        self.current_pose = None
        self.last_time = self.get_clock().now()

        # Control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Drone slave PID controller started")

    def goal_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.goal_pose is None or self.current_pose is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Position errors
        ex = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        ey = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        ez = self.goal_pose.pose.position.z - self.current_pose.pose.position.z

        # PID output => velocity
        vx = self.pid_x.compute(ex, dt)
        vy = self.pid_y.compute(ey, dt)
        vz = self.pid_z.compute(ez, dt)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz

        # TODO: Add yaw control
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSlaveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
