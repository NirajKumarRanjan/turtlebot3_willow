#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import heapq
import math

class AStarPlanner:
    def __init__(self):
        rospy.init_node('simple_astar_planner')

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # self.map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.init_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.map = None
        self.resolution = None
        self.origin = None

        self.start_x = None
        self.start_y = None

    def map_callback(self, msg):
        # Convert OccupancyGrid data to numpy array
        raw_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        rospy.loginfo("Map received: %d x %d" % (msg.info.width, msg.info.height))

        # Copy the map to inflate it
        inflated_map = raw_map.copy()

        # Choose inflation radius (in cells)
        inflation_radius_m = 0.2  # meters
        inflation_radius = int(inflation_radius_m / self.resolution)

        # Inflate all occupied cells
        for y in range(raw_map.shape[0]):
            for x in range(raw_map.shape[1]):
                if raw_map[y, x] > 50:  # Treat cells >50 as obstacles
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < raw_map.shape[1] and 0 <= ny < raw_map.shape[0]:
                                inflated_map[ny, nx] = 100  # Mark as occupied

        # Store the inflated map
        self.map = inflated_map


    def amcl_pose_callback(self, msg):
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        rospy.loginfo("AMCL pose: (%.2f, %.2f)" % (self.start_x, self.start_y))

    def initialpose_callback(self, msg):
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        rospy.loginfo("Received new initial pose: (%.2f, %.2f)" % (self.start_x, self.start_y))

    def goal_callback(self, goal):
        if self.map is None:
            rospy.logwarn("No map yet!")
            return

        if self.start_x is None or self.start_y is None:
            rospy.logwarn("No initial pose yet! Click 2D Pose Estimate in RViz.")
            return

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        rospy.loginfo("Planning from (%f, %f) to (%f, %f)" % (self.start_x, self.start_y, goal_x, goal_y))

        start_cell = self.world_to_map(self.start_x, self.start_y)
        goal_cell = self.world_to_map(goal_x, goal_y)

        path_cells = self.astar(start_cell, goal_cell)

        if path_cells is not None:
            # Smooth the raw path:
            smooth_cells = self.smooth_path(path_cells)

            path = Path()
            path.header = Header()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "map"

            for idx, cell in enumerate(smooth_cells):
                pose = PoseStamped()
                pose.header = path.header
                x, y = self.map_to_world(cell[0], cell[1])
                pose.pose.position.x = x
                pose.pose.position.y = y

                # Compute orientation:
                if idx < len(smooth_cells) - 1:
                    next_x, next_y = self.map_to_world(smooth_cells[idx + 1][0], smooth_cells[idx + 1][1])
                    yaw = math.atan2(next_y - y, next_x - x)
                else:
                    yaw = 0.0

                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                path.poses.append(pose)

            self.path_pub.publish(path)
            rospy.loginfo("Path published with %d points" % len(path.poses))
        else:
            rospy.logwarn("No path found!")

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return (mx, my)

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0]
        y = my * self.resolution + self.origin[1]
        return (x, y)

    def astar(self, start, goal):
        h, w = self.map.shape
        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start, []))
        closed = set()

        moves = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
        ]

        while open_list:
            est_total, cost, current, path = heapq.heappop(open_list)

            if current in closed:
                continue
            closed.add(current)

            path = path + [current]

            if current == goal:
                return path

            for dx, dy, move_cost in moves:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if self.map[ny][nx] == 0:
                        heapq.heappush(open_list, (
                            cost + move_cost + self.heuristic((nx, ny), goal),
                            cost + move_cost,
                            (nx, ny),
                            path
                        ))

        return None

    def heuristic(self, a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def bresenham(self, x0, y0, x1, y1):
        """ Bresenham's Line Algorithm """
        line = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                line.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                line.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        line.append((x, y))
        return line

    def is_clear(self, p1, p2):
        line = self.bresenham(p1[0], p1[1], p2[0], p2[1])
        for x, y in line:
            if self.map[y][x] != 0:
                return False
        return True

    def smooth_path(self, path):
        if not path:
            return []

        smoothed = [path[0]]
        idx = 0

        while idx < len(path) - 1:
            next_idx = len(path) - 1
            for j in range(len(path) - 1, idx, -1):
                if self.is_clear(path[idx], path[j]):
                    next_idx = j
                    break
            smoothed.append(path[next_idx])
            idx = next_idx

        return smoothed

if __name__ == '__main__':
    planner = AStarPlanner()
    rospy.spin()
