import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LeaderFollower(Node):

    def __init__(self):
        super().__init__('leader_follower')

        # Subscribe to leader pose
        self.leader_sub = self.create_subscription(
            PoseStamped,
            '/vtol_1/pose',
            self.leader_callback,
            10
        )

        # Publishers for follower goal poses
        self.goal_pub_2 = self.create_publisher(PoseStamped, '/vtol_2/goal_pose', 10)
        self.goal_pub_3 = self.create_publisher(PoseStamped, '/vtol_3/goal_pose', 10)

        
        self.offset_2 = (-3.0, -2.0, 0.0)  # behind-right
        self.offset_3 = (-3.0,  2.0, 0.0)  # behind-left

        self.get_logger().info('Leader-follower node started')

    def leader_callback(self, msg):
        yaw = get_yaw_from_quaternion(msg.pose.orientation)

        # Rotate body-frame offsets to world frame
        for offset, pub in [(self.offset_2, self.goal_pub_2),
                            (self.offset_3, self.goal_pub_3)]:
            dx_world = offset[0] * math.cos(yaw) - offset[1] * math.sin(yaw)
            dy_world = offset[0] * math.sin(yaw) + offset[1] * math.cos(yaw)

            goal = PoseStamped()
            goal.header = msg.header
            goal.pose.position.x = msg.pose.position.x + dx_world
            goal.pose.position.y = msg.pose.position.y + dy_world
            goal.pose.position.z = msg.pose.position.z + offset[2]
            goal.pose.orientation = msg.pose.orientation
            pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
