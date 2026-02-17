import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


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

        # Offsets from leader (x, y, z)
        self.offset_2 = (-3.0, -2.0, 0.0)  # behind-left
        self.offset_3 = (-3.0,  2.0, 0.0)  # behind-right

        self.get_logger().info('Leader-follower node started')

    def leader_callback(self, msg):
        # vtol_2 goal
        goal_2 = PoseStamped()
        goal_2.header = msg.header
        goal_2.pose.position.x = msg.pose.position.x + self.offset_2[0]
        goal_2.pose.position.y = msg.pose.position.y + self.offset_2[1]
        goal_2.pose.position.z = msg.pose.position.z + self.offset_2[2]
        goal_2.pose.orientation = msg.pose.orientation
        self.goal_pub_2.publish(goal_2)

        # vtol_3 goal
        goal_3 = PoseStamped()
        goal_3.header = msg.header
        goal_3.pose.position.x = msg.pose.position.x + self.offset_3[0]
        goal_3.pose.position.y = msg.pose.position.y + self.offset_3[1]
        goal_3.pose.position.z = msg.pose.position.z + self.offset_3[2]
        goal_3.pose.orientation = msg.pose.orientation
        self.goal_pub_3.publish(goal_3)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
