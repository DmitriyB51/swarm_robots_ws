import rclpy
from rclpy.node import Node

import numpy as np
import heapq
import math

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('ugv_path_planner')

        self.start_world = None
        self.goal_world = None

        self.obstacle_inflation_radius = 8  # cells (0.4m at 0.05m/cell)
        self.occ_threshold = 50

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            'path',
            10
        )

        self.map = None
        self.map_info = None

        self.get_logger().info("UGV A* Path Planner started (map-based)")

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )

    def odom_callback(self, msg: Odometry):
        self.start_world = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg: PoseStamped):
        self.goal_world = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        self.plan_and_publish()

    # Planning
    def plan_and_publish(self):

        if self.map is None:
            self.get_logger().warn("Map not received yet")
            return

        if self.start_world is None:
            self.get_logger().warn("Odometry not received yet")
            return

        start = self.world_to_grid(self.start_world)
        goal = self.world_to_grid(self.goal_world)

        inflated_map = self.inflate_obstacles(self.map)
        path_cells = self.a_star(inflated_map, start, goal)

        if not path_cells:
            self.get_logger().warn("No path found")
            return

        path_cells = self.smooth_path(path_cells)
        path_msg = self.cells_to_path(path_cells)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Path published")

    # A*
    def a_star(self, grid, start, goal):

        h = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (h(start, goal), 0, start))

        came_from = {}
        g_score = {start: 0}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)

                if not self.is_valid(grid, neighbor):
                    continue

                new_cost = cost + math.hypot(dx, dy)

                if neighbor not in g_score or new_cost < g_score[neighbor]:
                    g_score[neighbor] = new_cost
                    priority = new_cost + h(neighbor, goal)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current

        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # Map Utilities
    def inflate_obstacles(self, grid):
        inflated = np.copy(grid)
        obstacle_cells = np.where(grid > self.occ_threshold)

        for y, x in zip(*obstacle_cells):
            for dy in range(-self.obstacle_inflation_radius,
                            self.obstacle_inflation_radius + 1):
                for dx in range(-self.obstacle_inflation_radius,
                                self.obstacle_inflation_radius + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                        inflated[ny, nx] = 100
        return inflated

    def is_valid(self, grid, cell):
        x, y = cell
        if x < 0 or y < 0:
            return False
        if x >= grid.shape[1] or y >= grid.shape[0]:
            return False
        return grid[y, x] < self.occ_threshold

    def smooth_path(self, path):
        if len(path) < 3:
            return path

        # Step 1: remove collinear points (keep turn points)
        key_points = [path[0]]
        for i in range(1, len(path) - 1):
            prev = key_points[-1]
            curr = path[i]
            nxt = path[i + 1]

            if (prev[0] - curr[0], prev[1] - curr[1]) == \
               (curr[0] - nxt[0], curr[1] - nxt[1]):
                continue
            key_points.append(curr)
        key_points.append(path[-1])

        # Step 2: insert intermediate waypoints on long segments
        # so the controller has dense guidance (max ~10 cells apart ≈ 0.5m)
        max_gap = 10
        dense = [key_points[0]]
        for i in range(1, len(key_points)):
            ax, ay = dense[-1]
            bx, by = key_points[i]
            dx, dy = bx - ax, by - ay
            seg_len = math.hypot(dx, dy)
            steps = max(1, int(seg_len / max_gap))
            for s in range(1, steps):
                t = s / steps
                dense.append((int(ax + t * dx), int(ay + t * dy)))
            dense.append(key_points[i])

        return dense

    # Conversions
    def world_to_grid(self, pos):
        x, y = pos
        gx = int((x - self.map_info.origin.position.x) /
                 self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) /
                 self.map_info.resolution)
        return (gx, gy)

    def grid_to_world(self, cell):
        x, y = cell
        wx = x * self.map_info.resolution + \
             self.map_info.origin.position.x
        wy = y * self.map_info.resolution + \
             self.map_info.origin.position.y
        return (wx, wy)

    def cells_to_path(self, cells):

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for cell in cells:
            pose = PoseStamped()
            pose.header = path.header
            wx, wy = self.grid_to_world(cell)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
