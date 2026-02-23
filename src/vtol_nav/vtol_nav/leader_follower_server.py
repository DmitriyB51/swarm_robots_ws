"""
Leader-Follower Action Server

Takes a goal_pose for a specified leader drone, navigates it there
while specified followers maintain formation behind the leader.
Returns when the leader reaches the goal.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from vtol_interfaces.action import LeaderFollowerNavigate


def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LeaderFollowerServer(Node):

    def __init__(self):
        super().__init__('leader_follower_server')

        # Formation offsets for followers (body frame: x=forward, y=left)
        # First follower: behind-right, Second follower: behind-left
        self.follower_offsets = [
            (-3.0, -2.0, 0.0),  # behind-right
            (-3.0,  2.0, 0.0),  # behind-left
        ]

        self.dwell_time = 1.5  # seconds to stay within tolerance
        self.all_drone_names = ['vtol_1', 'vtol_2', 'vtol_3']

        self.cb_group = ReentrantCallbackGroup()

        # Action server
        self._action_server = ActionServer(
            self,
            LeaderFollowerNavigate,
            'leader_follower_navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # Publishers for drone goal poses (dynamically used)
        self.goal_publishers = {}
        for name in self.all_drone_names:
            self.goal_publishers[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10
            )

        # Subscribers for drone poses
        self.drone_poses = {}
        for name in self.all_drone_names:
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                10,
                callback_group=self.cb_group,
            )

        # Current formation state (set during goal execution)
        self._formation_active = False
        self._current_leader = None
        self._current_followers = []

        # Timer for follower formation updates (20 Hz)
        self.create_timer(0.05, self._update_followers, callback_group=self.cb_group)

        self.get_logger().info('Leader-Follower action server started')

    def _pose_callback(self, msg, drone_name):
        self.drone_poses[drone_name] = msg

    def _update_followers(self):
        """Update follower goal poses based on leader's current pose."""
        if not self._formation_active or self._current_leader is None:
            return

        leader_pose = self.drone_poses.get(self._current_leader)
        if leader_pose is None:
            return

        yaw = get_yaw_from_quaternion(leader_pose.pose.orientation)

        # Compute follower goals in formation
        for i, follower_name in enumerate(self._current_followers):
            if i >= len(self.follower_offsets):
                break

            offset = self.follower_offsets[i]
            pub = self.goal_publishers.get(follower_name)
            if pub is None:
                continue

            dx_world = offset[0] * math.cos(yaw) - offset[1] * math.sin(yaw)
            dy_world = offset[0] * math.sin(yaw) + offset[1] * math.cos(yaw)

            goal = PoseStamped()
            goal.header = leader_pose.header
            goal.pose.position.x = leader_pose.pose.position.x + dx_world
            goal.pose.position.y = leader_pose.pose.position.y + dy_world
            goal.pose.position.z = leader_pose.pose.position.z + offset[2]
            goal.pose.orientation = leader_pose.pose.orientation
            pub.publish(goal)

    def goal_callback(self, goal_request):
        leader = goal_request.leader_drone_name
        followers = list(goal_request.follower_drone_names)

        # Default to vtol_1 as leader if not specified
        if not leader:
            leader = 'vtol_1'
        # Default followers if not specified
        if not followers:
            followers = [n for n in self.all_drone_names if n != leader]

        self.get_logger().info(
            f'Received goal: leader={leader}, followers={followers}, '
            f'position=({goal_request.goal_pose.pose.position.x:.1f}, '
            f'{goal_request.goal_pose.pose.position.y:.1f}, '
            f'{goal_request.goal_pose.pose.position.z:.1f})'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        tolerance = goal.position_tolerance if goal.position_tolerance > 0 else 0.5
        timeout = goal.waypoint_timeout if goal.waypoint_timeout > 0 else 60.0
        goal_pose = goal.goal_pose

        # Get leader and followers from goal
        leader = goal.leader_drone_name if goal.leader_drone_name else 'vtol_1'
        followers = list(goal.follower_drone_names) if goal.follower_drone_names else [
            n for n in self.all_drone_names if n != leader
        ]

        # Set current formation
        self._current_leader = leader
        self._current_followers = followers
        self._formation_active = True

        # Publish goal to leader
        leader_pub = self.goal_publishers.get(leader)
        if leader_pub:
            leader_pub.publish(goal_pose)
        self.get_logger().info(f'Navigating {leader} (leader) to goal...')

        # Wait for arrival
        start = time.monotonic()
        in_tolerance_since = None

        while True:
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                self._formation_active = False
                goal_handle.abort()
                result = LeaderFollowerNavigate.Result()
                result.success = False
                result.message = f'Timeout waiting for {leader} to reach goal'
                self.get_logger().warn(result.message)
                return result

            if goal_handle.is_cancel_requested:
                self._formation_active = False
                goal_handle.canceled()
                result = LeaderFollowerNavigate.Result()
                result.success = False
                result.message = 'Canceled by user'
                return result

            leader_pose = self.drone_poses.get(leader)
            if leader_pose is None:
                time.sleep(0.1)
                continue

            # Compute distance to goal
            dx = leader_pose.pose.position.x - goal_pose.pose.position.x
            dy = leader_pose.pose.position.y - goal_pose.pose.position.y
            dz = leader_pose.pose.position.z - goal_pose.pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Check dwell time within tolerance
            if dist < tolerance:
                if in_tolerance_since is None:
                    in_tolerance_since = time.monotonic()
                dwell = time.monotonic() - in_tolerance_since
                if dwell >= self.dwell_time:
                    break  # Goal reached
            else:
                in_tolerance_since = None

            # Publish feedback
            feedback = LeaderFollowerNavigate.Feedback()
            feedback.leader_pose = leader_pose
            feedback.follower_poses = [
                self.drone_poses.get(f, PoseStamped()) for f in followers
            ]
            feedback.distance_to_goal = dist
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        self._formation_active = False
        goal_handle.succeed()
        result = LeaderFollowerNavigate.Result()
        result.success = True
        result.message = f'{leader} reached goal with followers in formation'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
