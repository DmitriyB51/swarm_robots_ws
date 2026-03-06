"""
Mission Action Client

A multi-phase mission client that:
- Phase 1a: Formation flight to initial gathering point (master-slave mode)
- Phase 1b: Formation flight to main waypoint (master-slave mode)
- Phase 2: Each drone navigates independently to separate waypoints
- Phase 3: All drones descend to z=0.5
- Phase 4: All drones ascend to z=5
"""

import math
import time
import os
import yaml
from PIL import Image
import numpy as np
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


def make_pose_quat(x, y, z, qx, qy, qz, qw):
    """Create a PoseStamped with position and quaternion orientation."""
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
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

        # Spiral search parameters
        self.detection_radius = 3.0   # meters - how close drone must be to "detect" target
        self.flight_altitude = 5.0    # meters - altitude for search
        
        # Spiral parameters (around origin)
        self.spiral_start_radius = 25.0  # meters - starting radius from origin
        self.spiral_end_radius = 2.0     # meters - minimum radius
        self.radius_decrement = 3.0      # meters - decrease per full rotation
        self.points_per_rotation = 12    # waypoints per full circle
        
        # Origin for spiral search
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Map file path
        self.map_yaml_path = os.path.expanduser(
            '~/swarm_robots_ws/src/map_provider/maps/updated_maze_map.yaml'
        )

        # Fixed target point
        self.fixed_target_x = 4.0
        self.fixed_target_y = 3.0

        # Spiral search state
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.target_point = None
        self.target_found = False
        self.finder_drone = None

        # Load map from file
        self._load_map_from_file()

        # Subscribe to drone poses for spiral search
        self.all_drone_names = ['vtol_1', 'vtol_2', 'vtol_3']
        self.drone_poses = {}
        for name in self.all_drone_names:
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda msg, n=name: self._spiral_pose_callback(msg, n),
                10, callback_group=self.cb_group
            )

        # Publishers for drone goal poses (for spiral search)
        self.goal_publishers = {}
        for name in self.all_drone_names:
            self.goal_publishers[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10
            )

        # Final goal_pose publisher
        self.final_goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )

        # Phase 1a: Initial formation point (several meters away for proper formation)
        self.formation_waypoint_1 = make_pose_quat(
            -25.0, 30.0, 9.0,
            0.0, 0.0, 0.0, 1.0
        )

        # Phase 1b: Main formation waypoint (master-slave mode)
        self.formation_waypoint_2 = make_pose_quat(
            0.23469066619873047, 17.478796005249023, 5.0,
            0.0, 0.0, -0.7051481933635764, 0.7090599589569871
        )

        # Phase 2: Independent flight - each drone goes to a separate point
        self.independent_waypoints = {
            'vtol_1': make_pose_quat(
                -1.0795470476150513, -18.356243133544922, 5.0,
                0.0, 0.0, 0.7239233431599225, 0.6898804195135279
            ),
            'vtol_2': make_pose_quat(
                -17.56472396850586, -0.7498464584350586, 5.0,
                0.0, 0.0, 0.021339575169731635, 0.9997722853388042
            ),
            'vtol_3': make_pose_quat(
                15.534133911132812, -0.7533082962036133, 5.0,
                0.0, 0.0, -0.9999898369146042, 0.004508444022421742
            ),
            
        }

        # Phase 3: Descent waypoints (z=0.5, same x/y as phase 2)
        self.descent_waypoints = {
            'vtol_1': make_pose_quat(
                -1.0795470476150513, -18.356243133544922, 1.5,
                0.0, 0.0, 0.7239233431599225, 0.6898804195135279
            ),
            'vtol_2': make_pose_quat(
                -17.56472396850586, -0.7498464584350586, 1.5,
                0.0, 0.0, 0.021339575169731635, 0.9997722853388042
            ),
            'vtol_3': make_pose_quat(
                15.534133911132812, -0.7533082962036133, 1.5,
                0.0, 0.0, -0.9999898369146042, 0.004508444022421742
            ),
            
        }

        # Phase 4: Ascent waypoints (z=5, same x/y as phase 2)
        self.ascent_waypoints = {
            'vtol_1': make_pose_quat(
                -1.0795470476150513, -18.356243133544922, 5.0,
                0.0, 0.0, 0.7239233431599225, 0.6898804195135279
            ),
            'vtol_2': make_pose_quat(
                -17.56472396850586, -0.7498464584350586, 5.0,
                0.0, 0.0, 0.021339575169731635, 0.9997722853388042
            ),
            'vtol_3': make_pose_quat(
                15.534133911132812, -0.7533082962036133, 5.0,
                0.0, 0.0, -0.9999898369146042, 0.004508444022421742
            ),
            
        }

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

        # === Phase 1a: Initial Formation Point (master-slave mode) ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 1a: Initial Formation Point (vtol_1 leads, vtol_2/vtol_3 follow) ===')
        success = self._execute_leader_follower(
            goal_pose=self.formation_waypoint_1,
            leader='vtol_1',
            followers=['vtol_2', 'vtol_3']
        )
        if not success:
            self.get_logger().error('Phase 1a failed, aborting mission')
            self._mission_complete = True
            return

        # Delay before next phase
        self.get_logger().info('Waiting 3 seconds before proceeding to Phase 1b...')
        time.sleep(3.0)

        # === Phase 1b: Main Formation Waypoint (master-slave mode) ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 1b: Main Formation Waypoint (vtol_1 leads, vtol_2/vtol_3 follow) ===')
        success = self._execute_leader_follower(
            goal_pose=self.formation_waypoint_2,
            leader='vtol_1',
            followers=['vtol_2', 'vtol_3']
        )
        if not success:
            self.get_logger().error('Phase 1b failed, aborting mission')
            self._mission_complete = True
            return

        # Delay before independent flight
        self.get_logger().info('Waiting 3 seconds before proceeding to Phase 2...')
        time.sleep(3.0)

        # === Phase 2: Independent Flight for Each Drone ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 2: Independent Flight to Separate Points ===')
        success = self._execute_independent_flight(self.independent_waypoints)
        if not success:
            self.get_logger().error('Phase 2 failed, aborting mission')
            self._mission_complete = True
            return

        # Delay before descent
        self.get_logger().info('Waiting 3 seconds before proceeding to Phase 3...')
        time.sleep(3.0)

        # === Phase 3: Descent to z=0.5 ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 3: Descent to z=0.5 ===')
        success = self._execute_independent_flight(self.descent_waypoints)
        if not success:
            self.get_logger().error('Phase 3 failed, aborting mission')
            self._mission_complete = True
            return

        # Delay before ascent
        self.get_logger().info('Waiting 3 seconds before proceeding to Phase 4...')
        time.sleep(3.0)

        # === Phase 4: Ascent to z=5 ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 4: Ascent to z=5 ===')
        success = self._execute_independent_flight(self.ascent_waypoints)
        if not success:
            self.get_logger().error('Phase 4 failed, aborting mission')
            self._mission_complete = True
            return

        # Delay before spiral search
        self.get_logger().info('Waiting 3 seconds before proceeding to Phase 5...')
        time.sleep(3.0)

        # === Phase 5: Spiral Search ===
        self.get_logger().info('')
        self.get_logger().info('=== Phase 5: Spiral Search for Random Target ===')
        success = self._execute_spiral_search()
        if not success:
            self.get_logger().error('Phase 5 (Spiral Search) failed')
            # Not aborting mission, just noting the failure

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

    def _execute_independent_flight(self, waypoints_dict):
        """Execute independent navigation for all drones in parallel."""
        futures = {}

        for drone_name, goal_pose in waypoints_dict.items():
            goal_msg = VtolNavigate.Goal()
            goal_msg.drone_name = drone_name
            goal_msg.goal_pose = goal_pose
            goal_msg.position_tolerance = 0.5
            goal_msg.waypoint_timeout = 60.0

            self.get_logger().info(
                f'Sending {drone_name} to ({goal_pose.pose.position.x:.2f}, '
                f'{goal_pose.pose.position.y:.2f}, {goal_pose.pose.position.z:.2f})'
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

    # === Spiral Search Methods ===

    def _load_map_from_file(self):
        """Load map from YAML and image file."""
        try:
            # Read YAML config
            with open(self.map_yaml_path, 'r') as f:
                map_config = yaml.safe_load(f)

            self.map_resolution = map_config['resolution']
            origin = map_config['origin']
            self.map_origin_x = origin[0]
            self.map_origin_y = origin[1]
            occupied_thresh = map_config.get('occupied_thresh', 0.65)
            free_thresh = map_config.get('free_thresh', 0.196)
            negate = map_config.get('negate', 0)

            # Get image path (relative to YAML file)
            yaml_dir = os.path.dirname(self.map_yaml_path)
            image_path = os.path.join(yaml_dir, map_config['image'])

            # Load image
            img = Image.open(image_path).convert('L')  # Convert to grayscale
            img_array = np.array(img)

            # Image dimensions
            self.map_height, self.map_width = img_array.shape

            # Convert to occupancy values (0-255 to 0-100 scale)
            # In ROS maps: white (255) = free, black (0) = occupied
            if negate:
                img_array = 255 - img_array

            # Normalize to 0-1
            normalized = img_array / 255.0

            # Convert to occupancy grid values
            # free (below free_thresh) -> 0
            # occupied (above occupied_thresh) -> 100
            # unknown (in between) -> -1
            self.map_data = np.zeros(img_array.shape, dtype=np.int8)
            self.map_data[normalized < free_thresh] = 0  # free
            self.map_data[normalized > occupied_thresh] = 100  # occupied
            self.map_data[(normalized >= free_thresh) & (normalized <= occupied_thresh)] = -1  # unknown

            # Flip vertically because image Y is inverted vs world Y
            self.map_data = np.flipud(self.map_data)

            self.get_logger().info(
                f'Map loaded from file: {self.map_width}x{self.map_height}, '
                f'resolution: {self.map_resolution}m/cell, '
                f'origin: ({self.map_origin_x}, {self.map_origin_y})'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            self.map_data = None

    def _spiral_pose_callback(self, msg: PoseStamped, drone_name: str):
        """Store drone pose for spiral search."""
        self.drone_poses[drone_name] = msg

    def _world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coordinates to grid cell indices."""
        if self.map_data is None:
            return None, None

        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)

        return grid_x, grid_y

    def _is_obstacle(self, x: float, y: float) -> bool:
        """Check if world position is an obstacle."""
        if self.map_data is None:
            return True  # Assume obstacle if no map

        grid_x, grid_y = self._world_to_grid(x, y)

        if grid_x < 0 or grid_x >= self.map_width:
            return True
        if grid_y < 0 or grid_y >= self.map_height:
            return True

        cell_value = self.map_data[grid_y, grid_x]

        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        return cell_value > 50 or cell_value < 0

    def _distance_2d(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate 2D distance."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def _generate_circular_spiral_waypoints(self) -> list:
        """
        Generate circular spiral waypoints around origin.
        Starts from outer radius and makes circular rotations,
        each rotation bringing drones closer to the origin.
        """
        waypoints = []
        current_radius = self.spiral_start_radius
        angular_step = (2 * math.pi) / self.points_per_rotation
        
        while current_radius >= self.spiral_end_radius:
            # Generate one complete rotation at this radius
            for i in range(self.points_per_rotation):
                angle = i * angular_step
                wp_x = self.origin_x + current_radius * math.cos(angle)
                wp_y = self.origin_y + current_radius * math.sin(angle)
                
                # Only add if not obstacle
                if not self._is_obstacle(wp_x, wp_y):
                    waypoints.append((wp_x, wp_y))
            
            # Decrease radius for next rotation (move closer to origin)
            current_radius -= self.radius_decrement
        
        self.get_logger().info(
            f'Generated {len(waypoints)} circular spiral waypoints around origin'
        )
        return waypoints

    def _check_target_detected(self, drone_name: str) -> bool:
        """Check if drone is close enough to detect the target."""
        if self.target_point is None:
            return False

        if drone_name not in self.drone_poses:
            return False

        pose = self.drone_poses[drone_name]
        drone_x = pose.pose.position.x
        drone_y = pose.pose.position.y

        target_x, target_y = self.target_point
        distance = self._distance_2d(drone_x, drone_y, target_x, target_y)

        return distance <= self.detection_radius

    def _navigate_drone_to(self, drone_name: str, x: float, y: float, z: float):
        """Send drone to a position."""
        if drone_name not in self.goal_publishers:
            return

        pose = make_pose(x, y, z)
        self.goal_publishers[drone_name].publish(pose)

    def _execute_spiral_search(self) -> bool:
        """
        Execute circular spiral search around the origin.
        All drones follow the same circular pattern, getting closer each rotation.
        """
        # Check if map was loaded successfully
        if self.map_data is None:
            self.get_logger().error('No map loaded - cannot execute spiral search')
            return False

        # Wait for drone poses
        self.get_logger().info('Waiting for drone poses...')
        timeout = 30.0
        start = time.monotonic()
        while len(self.drone_poses) < len(self.all_drone_names) and time.monotonic() - start < timeout:
            time.sleep(0.5)

        if len(self.drone_poses) < len(self.all_drone_names):
            self.get_logger().warn(
                f'Only {len(self.drone_poses)}/{len(self.all_drone_names)} drones detected'
            )

        # Set target point
        target_x = self.fixed_target_x
        target_y = self.fixed_target_y
        self.target_point = (target_x, target_y)
        self.get_logger().info(f'Target set at ({target_x:.2f}, {target_y:.2f})')

        # Generate circular spiral waypoints around origin
        waypoints = self._generate_circular_spiral_waypoints()
        
        if len(waypoints) == 0:
            self.get_logger().error('No waypoints generated')
            return False

        # Assign initial angular offsets for each drone (spread them around the circle)
        drone_angle_offsets = {}
        angle_spread = (2 * math.pi) / len(self.all_drone_names)
        for i, drone_name in enumerate(self.all_drone_names):
            drone_angle_offsets[drone_name] = i * angle_spread

        self.get_logger().info('=== Starting Circular Spiral Search Around Origin ===')
        self.get_logger().info(f'Searching with {len(waypoints)} waypoints')

        waypoint_idx = 0
        
        while not self.target_found and waypoint_idx < len(waypoints):
            wp_x, wp_y = waypoints[waypoint_idx]
            current_radius = self._distance_2d(self.origin_x, self.origin_y, wp_x, wp_y)
            current_angle = math.atan2(wp_y - self.origin_y, wp_x - self.origin_x)
            
            self.get_logger().info(
                f'Rotation waypoint {waypoint_idx + 1}/{len(waypoints)} - '
                f'radius: {current_radius:.1f}m'
            )

            # Move each drone to its position on the circle (with angular offset)
            for drone_name in self.all_drone_names:
                if drone_name not in self.drone_poses:
                    continue
                    
                # Calculate this drone's position with offset
                drone_angle = current_angle + drone_angle_offsets[drone_name]
                drone_wp_x = self.origin_x + current_radius * math.cos(drone_angle)
                drone_wp_y = self.origin_y + current_radius * math.sin(drone_angle)
                
                # Navigate drone
                self._navigate_drone_to(drone_name, drone_wp_x, drone_wp_y, self.flight_altitude)

            # Wait for drones to move towards waypoint
            time.sleep(1.0)

            # Check if any drone detected the target
            for drone_name in self.all_drone_names:
                if self._check_target_detected(drone_name):
                    self.target_found = True
                    self.finder_drone = drone_name
                    self.get_logger().info(f'*** TARGET FOUND by {drone_name}! ***')
                    break

            if not self.target_found:
                waypoint_idx += 1

        # If spiral completed without finding, go directly to target
        if not self.target_found:
            self.get_logger().warn('Spiral search completed without finding target - going directly')
            self.target_found = True
            self.finder_drone = self.all_drone_names[0]

        # === Target Found - Finder stops at target, others follow as slaves ===
        target_x, target_y = self.target_point
        
        self.get_logger().info(
            f'Finder {self.finder_drone} navigating to target at ({target_x:.2f}, {target_y:.2f})'
        )
        self._navigate_drone_to(self.finder_drone, target_x, target_y, self.flight_altitude)

        # Wait for finder to approach target
        time.sleep(3.0)

        # Other drones follow the finder as slaves (formation behind)
        self.get_logger().info('Other drones following finder as slaves...')
        
        # Formation offsets (behind and to the sides)
        follower_offsets = [(-3.0, -2.0), (-3.0, 2.0)]
        offset_idx = 0

        for drone_name in self.all_drone_names:
            if drone_name == self.finder_drone:
                continue

            # Get finder's current position
            if self.finder_drone in self.drone_poses:
                finder_pose = self.drone_poses[self.finder_drone]
                finder_x = finder_pose.pose.position.x
                finder_y = finder_pose.pose.position.y
            else:
                finder_x, finder_y = target_x, target_y

            # Apply formation offset
            if offset_idx < len(follower_offsets):
                ox, oy = follower_offsets[offset_idx]
                offset_idx += 1
            else:
                ox, oy = (-4.0, 0.0)

            follow_x = finder_x + ox
            follow_y = finder_y + oy

            self.get_logger().info(
                f'{drone_name} following as slave to ({follow_x:.2f}, {follow_y:.2f})'
            )
            self._navigate_drone_to(drone_name, follow_x, follow_y, self.flight_altitude)

        # Wait for formation
        time.sleep(5.0)

        # Publish final goal_pose for navigation
        final_pose = make_pose(target_x, target_y, self.flight_altitude)
        self.final_goal_pub.publish(final_pose)
        self.get_logger().info(
            f'Published goal_pose: ({target_x:.2f}, {target_y:.2f}, {self.flight_altitude})'
        )

        self.get_logger().info('Spiral search completed successfully')
        return True


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
