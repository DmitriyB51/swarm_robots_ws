"""
Block Search Node

This node:
1. Divides the maze into 3 horizontal strips
2. Each drone sweeps its strip with a lawnmower/zigzag pattern
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
        self.row_spacing = 3.0       # meters between zigzag rows
        self.wp_spacing = 2.0        # meters between waypoints along a row

        # Map file path
        self.map_yaml_path = os.path.expanduser(
            '~/swarm_robots_ws/src/map_provider/maps/updated_maze_map.yaml'
        )

        # Fixed target point
        self.fixed_target_x = 13.0
        self.fixed_target_y = 14.0

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

        self.get_logger().info('Block Search Node initialized')

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

    def compute_search_bounds(self):
        """Compute search bounds from free cells bounding box, inset by 2m to stay inside walls."""
        if self.map_data is None:
            return None

        free_cells = np.argwhere(self.map_data == 0)
        if len(free_cells) == 0:
            return None

        # free_cells is (row, col) i.e. (grid_y, grid_x)
        grid_y_min = free_cells[:, 0].min()
        grid_y_max = free_cells[:, 0].max()
        grid_x_min = free_cells[:, 1].min()
        grid_x_max = free_cells[:, 1].max()

        # Convert to world coords
        world_x_min = self.map_origin_x + grid_x_min * self.map_resolution
        world_x_max = self.map_origin_x + grid_x_max * self.map_resolution
        world_y_min = self.map_origin_y + grid_y_min * self.map_resolution
        world_y_max = self.map_origin_y + grid_y_max * self.map_resolution

        # Small inset to stay just inside the outer perimeter
        margin = 0.5
        x_min = world_x_min + margin
        x_max = world_x_max - margin
        y_min = world_y_min + margin
        y_max = world_y_max - margin

        self.get_logger().info(
            f'Search bounds: X [{x_min:.1f}, {x_max:.1f}], Y [{y_min:.1f}, {y_max:.1f}]'
        )
        return (x_min, x_max, y_min, y_max)

    def generate_lawnmower_waypoints(self, x_min, x_max, y_min, y_max):
        """Generate lawnmower/zigzag waypoints for a rectangular strip.
        No obstacle filtering — drones fly at altitude above maze walls."""
        waypoints = []
        y = y_min
        row = 0

        while y <= y_max:
            if row % 2 == 0:
                x = x_min
                while x <= x_max:
                    waypoints.append((x, y))
                    x += self.wp_spacing
            else:
                x = x_max
                while x >= x_min:
                    waypoints.append((x, y))
                    x -= self.wp_spacing
            y += self.row_spacing
            row += 1

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

    def execute_block_search(self) -> bool:
        """
        Execute block search — divide map into 3 horizontal strips,
        each drone sweeps its strip with a lawnmower/zigzag pattern.
        """
        # Check if map was loaded successfully
        if self.map_data is None:
            self.get_logger().error('No map loaded - cannot execute block search')
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

        # Compute search bounds from map
        bounds = self.compute_search_bounds()
        if bounds is None:
            self.get_logger().error('Could not compute search bounds')
            return False

        x_min, x_max, y_min, y_max = bounds

        # Divide Y range into 3 equal horizontal strips
        y_range = y_max - y_min
        strip_height = y_range / 3.0

        strip_assignments = {}
        for i, drone_name in enumerate(self.drone_names):
            strip_y_min = y_min + i * strip_height
            strip_y_max = y_min + (i + 1) * strip_height
            strip_assignments[drone_name] = (strip_y_min, strip_y_max)
            self.get_logger().info(
                f'{drone_name} assigned strip: Y [{strip_y_min:.1f}, {strip_y_max:.1f}]'
            )

        # Generate lawnmower waypoints for each drone's strip
        drone_waypoints = {}
        for drone_name in self.drone_names:
            sy_min, sy_max = strip_assignments[drone_name]
            wps = self.generate_lawnmower_waypoints(x_min, x_max, sy_min, sy_max)
            drone_waypoints[drone_name] = wps
            self.get_logger().info(
                f'{drone_name}: {len(wps)} waypoints generated'
            )

        self.get_logger().info('=== Starting Block Search ===')

        # Track waypoint index per drone
        drone_wp_idx = {name: 0 for name in self.drone_names}

        while not self.target_found:
            for drone_name in self.drone_names:
                wps = drone_waypoints[drone_name]
                if len(wps) == 0:
                    continue
                idx = drone_wp_idx[drone_name]

                # Wrap around to keep searching
                if idx >= len(wps):
                    drone_wp_idx[drone_name] = 0
                    idx = 0

                wp_x, wp_y = wps[idx]
                self.navigate_drone_to(drone_name, wp_x, wp_y, self.flight_altitude)
                drone_wp_idx[drone_name] = idx + 1

            # Wait for drones to move towards waypoint
            time.sleep(1.0)

            # Check if any drone detected the target
            for drone_name in self.drone_names:
                if self.check_target_detected(drone_name):
                    self.target_found = True
                    self.finder_drone = drone_name
                    self.get_logger().info(f'*** TARGET FOUND by {drone_name}! ***')
                    break

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
        self.get_logger().info('=== Block Search Complete ===')

        return True

    def run_search(self):
        """Run the block search (blocking)."""
        return self.execute_block_search()


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
