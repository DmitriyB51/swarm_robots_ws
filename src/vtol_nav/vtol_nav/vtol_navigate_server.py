"""
VTOL Navigate Action Server

A modular action server that navigates individual drones to goal poses.
Takes drone_name and goal_pose, sends commands to drone_slave_controller,
and responds when the drone reaches the goal.

Multiple instances can be launched, or it can be used per-drone.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from vtol_interfaces.action import VtolNavigate


class VtolNavigateServer(Node):

    def __init__(self):
        super().__init__('vtol_navigate_server')

        # Declare parameter for which drones to serve (empty = all)
        self.declare_parameter('drone_names', ['vtol_1', 'vtol_2', 'vtol_3'])
        self.drone_names = self.get_parameter('drone_names').get_parameter_value().string_array_value

        self.dwell_time = 1.5  # seconds within tolerance

        self.cb_group = ReentrantCallbackGroup()

        # Action server
        self._action_server = ActionServer(
            self,
            VtolNavigate,
            'vtol_navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # Publishers for drone goal poses (to drone_slave_controller)
        self.goal_publishers = {}
        for name in self.drone_names:
            self.goal_publishers[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10
            )

        # Subscribers for drone poses
        self.drone_poses = {}
        for name in self.drone_names:
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                10,
                callback_group=self.cb_group,
            )

        self.get_logger().info(
            f'VTOL Navigate server started for drones: {self.drone_names}'
        )

    def _pose_callback(self, msg, drone_name):
        self.drone_poses[drone_name] = msg

    def goal_callback(self, goal_request):
        drone_name = goal_request.drone_name
        if drone_name not in self.drone_names:
            self.get_logger().warn(
                f'Drone "{drone_name}" not served. Available: {self.drone_names}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Received goal for {drone_name} at '
            f'({goal_request.goal_pose.pose.position.x:.1f}, '
            f'{goal_request.goal_pose.pose.position.y:.1f}, '
            f'{goal_request.goal_pose.pose.position.z:.1f})'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        drone_name = goal.drone_name
        goal_pose = goal.goal_pose
        tolerance = goal.position_tolerance if goal.position_tolerance > 0 else 0.5
        timeout = goal.waypoint_timeout if goal.waypoint_timeout > 0 else 60.0

        # Get the publisher for this drone
        goal_pub = self.goal_publishers.get(drone_name)
        if goal_pub is None:
            goal_handle.abort()
            result = VtolNavigate.Result()
            result.success = False
            result.message = f'No publisher for drone {drone_name}'
            return result

        # Publish goal to drone_slave_controller
        goal_pub.publish(goal_pose)
        self.get_logger().info(f'Navigating {drone_name} to goal...')

        # Wait for arrival
        start = time.monotonic()
        in_tolerance_since = None

        while True:
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                goal_handle.abort()
                result = VtolNavigate.Result()
                result.success = False
                result.message = f'Timeout: {drone_name} did not reach goal'
                self.get_logger().warn(result.message)
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = VtolNavigate.Result()
                result.success = False
                result.message = f'Canceled navigation for {drone_name}'
                return result

            current_pose = self.drone_poses.get(drone_name)
            if current_pose is None:
                time.sleep(0.1)
                continue

            # Compute distance to goal
            dx = current_pose.pose.position.x - goal_pose.pose.position.x
            dy = current_pose.pose.position.y - goal_pose.pose.position.y
            dz = current_pose.pose.position.z - goal_pose.pose.position.z
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
            feedback = VtolNavigate.Feedback()
            feedback.current_pose = current_pose
            feedback.distance_to_goal = dist
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        goal_handle.succeed()
        result = VtolNavigate.Result()
        result.success = True
        result.message = f'{drone_name} reached goal'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VtolNavigateServer()
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
