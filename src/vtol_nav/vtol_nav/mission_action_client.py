"""
Mission Action Client

A multi-phase mission client that:
- Point 1: Uses leader_follower_server to navigate vtol_1 while vtol_2, vtol_3 follow
- Points 2-4: Each drone navigates independently using vtol_navigate_server
- Point 5: Uses leader_follower_server with vtol_2 as leader, vtol_1/vtol_3 follow
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from vtol_interfaces.action import LeaderFollowerNavigate, VtolNavigate


def make_pose(x, y, z, yaw=0.0):
    """Create a PoseStamped with position and yaw."""
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class MissionActionClient(Node):

    def __init__(self):
        super().__init__('mission_action_client')

        self.cb_group = ReentrantCallbackGroup()

        # Action clients
        self._leader_follower_client = ActionClient(
            self, LeaderFollowerNavigate, 'leader_follower_navigate',
            callback_group=self.cb_group
        )
        self._vtol_navigate_client = ActionClient(
            self, VtolNavigate, 'vtol_navigate',
            callback_group=self.cb_group
        )

        # Mission waypoints
        # Point 1: Formation flight with vtol_1 as leader
        self.formation_waypoint_1 = make_pose(10.0, 0.0, 5.0, yaw=0.0)

        # Points 2-4: Independent flight per drone
        self.independent_waypoints = {
            'vtol_1': [
                make_pose(15.0, 5.0, 6.0, yaw=0.5),
                make_pose(20.0, 0.0, 5.0, yaw=0.0),
                make_pose(15.0, -5.0, 4.0, yaw=-0.5),
            ],
            'vtol_2': [
                make_pose(5.0, 10.0, 5.0, yaw=1.57),
                make_pose(0.0, 15.0, 6.0, yaw=3.14),
                make_pose(-5.0, 10.0, 5.0, yaw=-1.57),
            ],
            'vtol_3': [
                make_pose(5.0, -10.0, 5.0, yaw=-1.57),
                make_pose(0.0, -15.0, 6.0, yaw=3.14),
                make_pose(-5.0, -10.0, 5.0, yaw=1.57),
            ],
        }

        # Point 5: Formation flight with vtol_2 as leader
        self.formation_waypoint_2 = make_pose(-10.0, 5.0, 5.0, yaw=2.0)

        self._mission_complete = False

    def run_mission(self):
        """Execute the full mission."""
        self.get_logger().info('=== Starting Mission ===')

        # Wait for servers
        self.get_logger().info('Waiting for action servers...')
        if not self._leader_follower_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Leader-follower server not available')
            return
        if not self._vtol_navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('VTOL navigate server not available')
            return

        self.get_logger().info('Action servers ready!')

        # === Phase 1: Leader-Follower Formation Flight (vtol_1 leads) ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 1: Formation Flight (vtol_1 leads, vtol_2/vtol_3 follow) ===')
        success = self._execute_leader_follower(
            goal_pose=self.formation_waypoint_1,
            leader='vtol_1',
            followers=['vtol_2', 'vtol_3']
        )
        if not success:
            self.get_logger().error('Phase 1 failed, aborting mission')
            self._mission_complete = True
            return

        # === Phase 2-4: Independent Flight for Each Drone ===
        for phase_idx in range(3):
            self.get_logger().info('')
            self.get_logger().info(f'=== Phase {phase_idx + 2}: Independent Flight ===')
            success = self._execute_independent_phase(phase_idx)
            if not success:
                self.get_logger().error(f'Phase {phase_idx + 2} failed, aborting mission')
                self._mission_complete = True
                return

        # === Phase 5: Leader-Follower Formation Flight (vtol_2 leads) ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 5: Formation Flight (vtol_2 leads, vtol_1/vtol_3 follow) ===')
        success = self._execute_leader_follower(
            goal_pose=self.formation_waypoint_2,
            leader='vtol_2',
            followers=['vtol_1', 'vtol_3']
        )
        if not success:
            self.get_logger().error('Phase 5 failed, aborting mission')
            self._mission_complete = True
            return

        self.get_logger().info('')
        self.get_logger().info('=== Mission Complete! ===')
        self._mission_complete = True

    def _execute_leader_follower(self, goal_pose, leader='vtol_1', followers=None):
        """Execute leader-follower navigation to a goal pose."""
        if followers is None:
            followers = ['vtol_2', 'vtol_3']

        goal_msg = LeaderFollowerNavigate.Goal()
        goal_msg.leader_drone_name = leader
        goal_msg.follower_drone_names = followers
        goal_msg.goal_pose = goal_pose
        goal_msg.position_tolerance = 0.5
        goal_msg.waypoint_timeout = 60.0

        self.get_logger().info(
            f'Sending formation goal: leader={leader}, followers={followers}, '
            f'position=({goal_pose.pose.position.x:.1f}, '
            f'{goal_pose.pose.position.y:.1f}, {goal_pose.pose.position.z:.1f})'
        )

        future = self._leader_follower_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._leader_follower_feedback(fb, leader)
        )

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Leader-follower goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'Formation goal reached: {result.message}')
        else:
            self.get_logger().error(f'Formation goal failed: {result.message}')

        return result.success

    def _leader_follower_feedback(self, feedback_msg, leader_name):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[Formation {leader_name}] Distance to goal: {fb.distance_to_goal:.2f}m'
        )

    def _execute_independent_phase(self, phase_idx):
        """Execute independent navigation for all drones in parallel."""
        futures = {}

        for drone_name, waypoints in self.independent_waypoints.items():
            if phase_idx >= len(waypoints):
                continue

            goal_pose = waypoints[phase_idx]
            goal_msg = VtolNavigate.Goal()
            goal_msg.drone_name = drone_name
            goal_msg.goal_pose = goal_pose
            goal_msg.position_tolerance = 0.5
            goal_msg.waypoint_timeout = 60.0

            self.get_logger().info(
                f'Sending {drone_name} to ({goal_pose.pose.position.x:.1f}, '
                f'{goal_pose.pose.position.y:.1f}, {goal_pose.pose.position.z:.1f})'
            )

            future = self._vtol_navigate_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda fb, name=drone_name: self._vtol_feedback(fb, name)
            )
            futures[drone_name] = future

        # Wait for all goals to be accepted
        goal_handles = {}
        for drone_name, future in futures.items():
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'{drone_name} goal rejected')
                return False
            goal_handles[drone_name] = goal_handle

        # Wait for all results
        all_success = True
        for drone_name, goal_handle in goal_handles.items():
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if result.success:
                self.get_logger().info(f'{drone_name}: {result.message}')
            else:
                self.get_logger().error(f'{drone_name}: {result.message}')
                all_success = False

        return all_success

    def _vtol_feedback(self, feedback_msg, drone_name):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{drone_name}] Distance to goal: {fb.distance_to_goal:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    client = MissionActionClient()

    # Run mission in a separate thread to not block
    import threading
    mission_thread = threading.Thread(target=client.run_mission)
    mission_thread.start()

    # Spin while mission is running
    while rclpy.ok() and not client._mission_complete:
        rclpy.spin_once(client, timeout_sec=0.1)

    mission_thread.join()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
