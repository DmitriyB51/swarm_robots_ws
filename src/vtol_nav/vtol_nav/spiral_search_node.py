"""
Spiral Search Node

This node:
1. Drones execute circular spiral pattern around the origin (0,0)
2. Each rotation brings them closer to the origin
3. Uses a fixed target point (x=4, y=3)
4. When a drone "finds" the target (within detection radius), it stops
5. Other drones follow the finder drone as slaves
6. Finally publishes the target as goal_pose for navigation
"""

import math
import time
import threading
import os
import yaml
from PIL import Image
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped


class SpiralSearchNode(Node):

    def __init__(self):
        super().__init__('spiral_search_node')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.drone_names = ['vtol_1', 'vtol_2', 'vtol_3']
        self.detection_radius = 3.0  # meters - how close drone must be to "detect" target
        self.flight_altitude = 5.0   # meters - altitude for search
        
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

        # State
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.drone_poses = {}
        self.target_point = None
        self.target_found = False
        self.finder_drone = None
        self.search_complete = False

        # Load map from file
        self._load_map_from_file()

        # Subscribe to drone poses
        for name in self.drone_names:
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self.pose_callback(msg, n),
                10,
                callback_group=self.cb_group
            )

        # Publishers for drone goal poses
        self.goal_publishers = {}
        for name in self.drone_names:
            self.goal_publishers[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10
            )

        # Final goal_pose publisher (for overall mission)
        self.final_goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )

        self.get_logger().info('Spiral Search Node initialized')

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

    def pose_callback(self, msg: PoseStamped, drone_name: str):
        """Store drone pose."""
        self.drone_poses[drone_name] = msg

    def world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coordinates to grid cell indices."""
        if self.map_data is None:
            return None, None

        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)

        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> tuple:
        """Convert grid cell indices to world coordinates (cell center)."""
        if self.map_data is None:
            return None, None

        x = self.map_origin_x + (grid_x + 0.5) * self.map_resolution
        y = self.map_origin_y + (grid_y + 0.5) * self.map_resolution

        return x, y

    def is_obstacle(self, x: float, y: float) -> bool:
        """Check if world position is an obstacle."""
        if self.map_data is None:
            return True  # Assume obstacle if no map

        grid_x, grid_y = self.world_to_grid(x, y)

        if grid_x < 0 or grid_x >= self.map_width:
            return True
        if grid_y < 0 or grid_y >= self.map_height:
            return True

        cell_value = self.map_data[grid_y, grid_x]

        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        # Consider occupied (>50) or unknown (-1) as obstacles
        return cell_value > 50 or cell_value < 0

    def make_pose(self, x: float, y: float, z: float, yaw: float = 0.0) -> PoseStamped:
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def get_yaw_from_quaternion(self, q) -> float:
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance_2d(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate 2D distance."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def generate_circular_spiral_waypoints(self) -> list:
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
                if not self.is_obstacle(wp_x, wp_y):
                    waypoints.append((wp_x, wp_y))
            
            # Decrease radius for next rotation (move closer to origin)
            current_radius -= self.radius_decrement
        
        self.get_logger().info(
            f'Generated {len(waypoints)} circular spiral waypoints around origin'
        )
        return waypoints

    def check_target_detected(self, drone_name: str) -> bool:
        """Check if drone is close enough to detect the target."""
        if self.target_point is None:
            return False

        if drone_name not in self.drone_poses:
            return False

        pose = self.drone_poses[drone_name]
        drone_x = pose.pose.position.x
        drone_y = pose.pose.position.y

        target_x, target_y = self.target_point
        distance = self.distance_2d(drone_x, drone_y, target_x, target_y)

        return distance <= self.detection_radius

    def navigate_drone_to(self, drone_name: str, x: float, y: float, z: float):
        """Send drone to a position."""
        if drone_name not in self.goal_publishers:
            return

        pose = self.make_pose(x, y, z)
        self.goal_publishers[drone_name].publish(pose)

    def wait_for_drone_arrival(self, drone_name: str, x: float, y: float, 
                                tolerance: float = 1.0, timeout: float = 30.0) -> bool:
        """Wait for drone to reach position."""
        start = time.monotonic()

        while time.monotonic() - start < timeout:
            if drone_name not in self.drone_poses:
                time.sleep(0.1)
                continue

            pose = self.drone_poses[drone_name]
            drone_x = pose.pose.position.x
            drone_y = pose.pose.position.y

            dist = self.distance_2d(drone_x, drone_y, x, y)
            if dist <= tolerance:
                return True

            time.sleep(0.1)

        return False

    def execute_spiral_search(self) -> bool:
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
        while len(self.drone_poses) < len(self.drone_names) and time.monotonic() - start < timeout:
            time.sleep(0.5)

        if len(self.drone_poses) < len(self.drone_names):
            self.get_logger().warn(
                f'Only {len(self.drone_poses)}/{len(self.drone_names)} drones detected'
            )

        # Set target point
        target_x = self.fixed_target_x
        target_y = self.fixed_target_y
        self.target_point = (target_x, target_y)
        self.get_logger().info(f'Target set at ({target_x:.2f}, {target_y:.2f})')

        # Generate circular spiral waypoints around origin
        waypoints = self.generate_circular_spiral_waypoints()
        
        if len(waypoints) == 0:
            self.get_logger().error('No waypoints generated')
            return False

        # Assign initial angular offsets for each drone (spread them around the circle)
        drone_angle_offsets = {}
        angle_spread = (2 * math.pi) / len(self.drone_names)
        for i, drone_name in enumerate(self.drone_names):
            drone_angle_offsets[drone_name] = i * angle_spread

        self.get_logger().info('=== Starting Circular Spiral Search Around Origin ===')
        self.get_logger().info(f'Searching with {len(waypoints)} waypoints')

        waypoint_idx = 0
        
        while not self.target_found and waypoint_idx < len(waypoints):
            wp_x, wp_y = waypoints[waypoint_idx]
            current_radius = self.distance_2d(self.origin_x, self.origin_y, wp_x, wp_y)
            current_angle = math.atan2(wp_y - self.origin_y, wp_x - self.origin_x)
            
            self.get_logger().info(
                f'Rotation waypoint {waypoint_idx + 1}/{len(waypoints)} - '
                f'radius: {current_radius:.1f}m'
            )

            # Move each drone to its position on the circle (with angular offset)
            for drone_name in self.drone_names:
                if drone_name not in self.drone_poses:
                    continue
                    
                # Calculate this drone's position with offset
                drone_angle = current_angle + drone_angle_offsets[drone_name]
                drone_wp_x = self.origin_x + current_radius * math.cos(drone_angle)
                drone_wp_y = self.origin_y + current_radius * math.sin(drone_angle)
                
                # Navigate drone
                self.navigate_drone_to(drone_name, drone_wp_x, drone_wp_y, self.flight_altitude)

            # Wait for drones to move towards waypoint
            time.sleep(1.0)

            # Check if any drone detected the target
            for drone_name in self.drone_names:
                if self.check_target_detected(drone_name):
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
            self.finder_drone = self.drone_names[0]

        # === Target Found - Finder stops at target, others follow as slaves ===
        target_x, target_y = self.target_point
        
        self.get_logger().info(
            f'Finder {self.finder_drone} navigating to target at ({target_x:.2f}, {target_y:.2f})'
        )
        self.navigate_drone_to(self.finder_drone, target_x, target_y, self.flight_altitude)

        # Wait for finder to approach target
        time.sleep(3.0)

        # Other drones follow the finder as slaves (formation behind)
        self.get_logger().info('Other drones following finder as slaves...')
        
        # Formation offsets (behind and to the sides)
        follower_offsets = [(-3.0, -2.0), (-3.0, 2.0)]
        offset_idx = 0

        for drone_name in self.drone_names:
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
            self.navigate_drone_to(drone_name, follow_x, follow_y, self.flight_altitude)

        # Wait for formation
        time.sleep(5.0)

        # Publish final goal_pose for navigation
        final_pose = self.make_pose(target_x, target_y, self.flight_altitude)
        self.final_goal_pub.publish(final_pose)
        self.get_logger().info(
            f'Published goal_pose: ({target_x:.2f}, {target_y:.2f}, {self.flight_altitude})'
        )

        self.search_complete = True
        self.get_logger().info('=== Spiral Search Complete ===')

        return True

        return True

    def run_search(self):
        """Run the spiral search (blocking)."""
        return self.execute_spiral_search()


def main(args=None):
    rclpy.init(args=args)
    node = SpiralSearchNode()

    # Run search in separate thread
    search_thread = threading.Thread(target=node.run_search)
    search_thread.start()

    # Spin while search is running
    while rclpy.ok() and not node.search_complete:
        rclpy.spin_once(node, timeout_sec=0.1)

    search_thread.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
