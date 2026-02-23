import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from vtol_interfaces.action import SwarmNavigate


class SwarmNavigateClient(Node):

    def __init__(self):
        super().__init__('swarm_navigate_client')
        self._client = ActionClient(self, SwarmNavigate, 'swarm_navigate')
        self._done = False

    def send_goal(self, waypoints, tolerance=0.5, timeout=60.0):
        goal_msg = SwarmNavigate.Goal()
        goal_msg.goal_poses = waypoints
        goal_msg.position_tolerance = tolerance
        goal_msg.waypoint_timeout = timeout

        
        self._client.wait_for_server()

        self.get_logger().info(
            f'Sending goal with {len(waypoints)} waypoint(s)'
        )
        future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Waypoint {fb.current_waypoint_index + 1}/{fb.total_waypoints}, '
            f'distance: {fb.distance_to_waypoint:.2f}m'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._done = True
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = 'SUCCESS' if result.success else 'FAILED'
        self.get_logger().info(
            f'[{status}] {result.message} '
            f'(waypoints reached: {result.waypoints_reached})'
        )
        self._done = True


def make_pose(x, y, z, yaw=0.0):
    """Helper to create a PoseStamped with position and yaw."""
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(args=None):
    rclpy.init(args=args)
    client = SwarmNavigateClient()

    # 4 waypoints
    waypoints = [
        make_pose( 5.0,  5.0, 4.0, yaw= 1.5708),
        make_pose( 0.0, 10.0, 3.0, yaw= 3.1416),
        make_pose(-5.0,  5.0, 2.0, yaw=-1.5708),
        make_pose( 0.0,  0.0, 0.0, yaw= 0.0),
    ]

    client.send_goal(waypoints, tolerance=0.5, timeout=60.0)

    while rclpy.ok() and not client._done:
        rclpy.spin_once(client, timeout_sec=0.1)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
