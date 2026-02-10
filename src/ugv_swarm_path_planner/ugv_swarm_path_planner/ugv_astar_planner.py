import rclpy
from rclpy.node import Node

import numpy as np
import heapq
import math

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('ugv_path_planner')

        # parameters
        self.start_world = (3.5, -6.5)  # fixed start (x, y) in meters
        self.obstacle_inflation_radius = 3  # cells
        self.occ_threshold = 50  # occupancy threshold

        # subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )


        # publishers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/ugv/path',
            10
        )

        self.map = None
        self.map_info = None
        self.goal_world = None

        self.get_logger().info("UGV A* Path Planner node started")

    
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )

    def goal_callback(self, msg: PoseStamped):
        if self.map is None:
            self.get_logger().warn("Map not received yet")
            return

        self.goal_world = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.plan_and_publish()


    # PLANNING
    def plan_and_publish(self):
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
        heapq.heappush(open_set, (0 + h(start, goal), 0, start))

        came_from = {}
        g_score = {start: 0}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
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


    # map processing
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


    # path smoothing
    def smooth_path(self, path):
        if len(path) < 3:
            return path

        smooth = [path[0]]
        for i in range(1, len(path) - 1):
            prev = smooth[-1]
            curr = path[i]
            nxt = path[i + 1]

            if (prev[0] - curr[0], prev[1] - curr[1]) == \
               (curr[0] - nxt[0], curr[1] - nxt[1]):
                continue
            smooth.append(curr)

        smooth.append(path[-1])
        return smooth


    # world to grid conversions
    def world_to_grid(self, pos):
        x, y = pos
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (gx, gy)

    def grid_to_world(self, cell):
        x, y = cell
        wx = x * self.map_info.resolution + self.map_info.origin.position.x
        wy = y * self.map_info.resolution + self.map_info.origin.position.y
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
