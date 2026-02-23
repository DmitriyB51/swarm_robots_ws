import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from vtol_interfaces.action import SwarmNavigate


class SwarmNavigateServer(Node):

    def __init__(self):
        super().__init__('swarm_navigate_server')

        self.num_drones = 3
        self.dwell_time = 1.5  # wait before goinf to next point

        # Callback group for concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Action server
        self._action_server = ActionServer(
            self,
            SwarmNavigate, #in vtol_interface
            'swarm_navigate',
            execute_callback=self.execute_callback, #start
            goal_callback=self.goal_callback, #goal input
            cancel_callback=self.cancel_callback,#user cancelation
            callback_group=cb_group,
        )

        # Publisher: send goal to leader drone
        self.leader_goal_pub = self.create_publisher(
            PoseStamped, '/vtol_1/goal_pose', 10
        )

        # Subscribe to all drone poses
        self.drone_poses = {}
        for i in range(1, self.num_drones + 1):
            name = f'vtol_{i}'
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                10,
                callback_group=cb_group,
            )
    #latest pose 
    def _pose_callback(self, msg, drone_name):
        self.drone_poses[drone_name] = msg

    def goal_callback(self, goal_request):
        if len(goal_request.goal_poses) == 0:
            self.get_logger().warn('no waypoints')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Accepted goal with {len(goal_request.goal_poses)} waypoints'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        tolerance = goal.position_tolerance if goal.position_tolerance > 0 else 0.5
        timeout = goal.waypoint_timeout if goal.waypoint_timeout > 0 else 60.0
        total = len(goal.goal_poses)

        for idx, waypoint in enumerate(goal.goal_poses):
            # Publish waypoint to leader
            self.leader_goal_pub.publish(waypoint)
            self.get_logger().info(
                f'Navigating to waypoint {idx + 1}/{total} '
                f'({waypoint.pose.position.x:.1f}, '
                f'{waypoint.pose.position.y:.1f}, '
                f'{waypoint.pose.position.z:.1f})'
            )

            # Wait for arrival
            arrived = self._wait_for_arrival(
                goal_handle, waypoint, tolerance, timeout, idx, total
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = SwarmNavigate.Result()
                result.success = False
                result.waypoints_reached = idx
                result.message = f'Canceled at waypoint {idx + 1}'
                self.get_logger().info(result.message)
                return result

            if not arrived:
                goal_handle.abort()
                result = SwarmNavigate.Result()
                result.success = False
                result.waypoints_reached = idx
                result.message = f'Timeout at waypoint {idx + 1}'
                self.get_logger().warn(result.message)
                return result

            self.get_logger().info(f'Reached waypoint {idx + 1}/{total}')

        goal_handle.succeed()
        result = SwarmNavigate.Result()
        result.success = True
        result.waypoints_reached = total
        result.message = f'All {total} waypoint(s) reached'
        self.get_logger().info(result.message)
        return result

    def _wait_for_arrival(self, goal_handle, waypoint, tolerance, timeout, idx, total):
        start = time.monotonic()
        in_tolerance_since = None

        while True:
            # Check timeout
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                return False

            # Check cancel
            if goal_handle.is_cancel_requested:
                return False

            # Get leader pose
            leader_pose = self.drone_poses.get('vtol_1')
            if leader_pose is None:
                time.sleep(0.1)
                continue

            # Compute distance
            dx = leader_pose.pose.position.x - waypoint.pose.position.x
            dy = leader_pose.pose.position.y - waypoint.pose.position.y
            dz = leader_pose.pose.position.z - waypoint.pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Check dwell within tolerance
            if dist < tolerance:
                if in_tolerance_since is None:
                    in_tolerance_since = time.monotonic()
                dwell = time.monotonic() - in_tolerance_since
                if dwell >= self.dwell_time:
                    return True
            else:
                in_tolerance_since = None

            # Publish feedback
            feedback = SwarmNavigate.Feedback()
            feedback.current_waypoint_index = idx
            feedback.total_waypoints = total
            feedback.drone_poses = [
                self.drone_poses.get(f'vtol_{i + 1}', PoseStamped())
                for i in range(self.num_drones)
            ]
            feedback.distance_to_waypoint = dist
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)  


def main(args=None):
    rclpy.init(args=args)
    node = SwarmNavigateServer()
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
