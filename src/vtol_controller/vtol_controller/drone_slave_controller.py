import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.time import Time


class PID:
    def __init__(self, kp, ki, kd, limit=None, max_rate=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.max_rate = max_rate  # max change per second (acceleration limit)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_output = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_output = 0.0

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

        # Acceleration limit
        if self.max_rate is not None:
            max_change = self.max_rate * dt
            output = max(min(output, self.prev_output + max_change),
                         self.prev_output - max_change)

        self.prev_output = output
        return output



def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_roll_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.atan2(sinr_cosp, cosr_cosp)


def get_pitch_from_quaternion(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DroneSlaveController(Node):

    def __init__(self):
        super().__init__('drone_slave_controller')

        # subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'pose',
            self.pose_callback,
            10
        )

        # publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)


        # Position PIDs (world-frame, same as roi_drones)
        self.pid_x = PID(kp=0.8, ki=0, kd=0.1, limit=2.0, max_rate=1.0)
        self.pid_y = PID(kp=0.8, ki=0, kd=0.1, limit=2.0, max_rate=1.0)
        self.pid_z = PID(kp=1.0, ki=0, kd=0.15, limit=2.0, max_rate=1.0)

        # Angular PIDs
        self.pid_yaw = PID(kp=1.5, ki=0, kd=0.2, limit=0.5, max_rate=0.5)
        self.pid_roll = PID(kp=3.0, ki=0, kd=0.5, limit=2.0)
        self.pid_pitch = PID(kp=3.0, ki=0, kd=0.5, limit=2.0)


        # How much the drone tilts based on velocity
        self.tilt_factor = 0.08

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

        # Position control (world-frame, separate X/Y/Z)
        vx = self.pid_x.compute(ex, dt)
        vy = self.pid_y.compute(ey, dt)
        vz = self.pid_z.compute(ez, dt)

        # Prevent going underground
        min_z = 0.15
        current_z = self.current_pose.pose.position.z
        if current_z < min_z and vz < 0.0:
            vz = 0.0
            self.pid_z.reset()

        # Yaw control from goal orientation
        goal_yaw = get_yaw_from_quaternion(self.goal_pose.pose.orientation)
        current_yaw = get_yaw_from_quaternion(self.current_pose.pose.orientation)
        yaw_error = wrap_angle(goal_yaw - current_yaw)
        yaw_rate = self.pid_yaw.compute(yaw_error, dt)

        # Convert world velocity to body-frame for tilt compensation
        front_vel = vx * math.cos(goal_yaw) + vy * math.sin(goal_yaw)
        side_vel = -vx * math.sin(goal_yaw) + vy * math.cos(goal_yaw)

        desired_pitch = -self.tilt_factor * front_vel
        desired_roll = self.tilt_factor * side_vel

        current_roll = get_roll_from_quaternion(self.current_pose.pose.orientation)
        current_pitch = get_pitch_from_quaternion(self.current_pose.pose.orientation)

        roll_rate = self.pid_roll.compute(desired_roll - current_roll, dt)
        pitch_rate = self.pid_pitch.compute(desired_pitch - current_pitch, dt)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.x = roll_rate
        cmd.angular.y = pitch_rate
        cmd.angular.z = yaw_rate

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSlaveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
