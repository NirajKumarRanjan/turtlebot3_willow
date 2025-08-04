#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import heapq

class AStarPlanner:
    def __init__(self):
        rospy.init_node('simple_astar_planner')

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
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
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        rospy.loginfo("Map received: %d x %d" % (msg.info.width, msg.info.height))

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

        path_cells = self.astar(start_cell, goal_cell) # Actual planning is done here!! Enters to while loop when both gets updated

        if path_cells is not None: #executes when a path is returned
            path = Path()
            path.header = Header()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "map"

            for cell in path_cells:
                pose = PoseStamped()
                pose.header = path.header
                x, y = self.map_to_world(cell[0], cell[1])
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
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
        # h, w  map height & width.
        # open_list  priority queue, with:
        # (estimated_total_cost, cost_so_far, current_node, path_so_far)
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start, []))
        closed = set() #It creates an empty Python set named closed.

        moves = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
        ]

        while open_list:
            est_total, cost, current, path = heapq.heappop(open_list) 

            if current in closed: # checking for visited node
                continue  # does not run any further line and goes for next while iteration
            closed.add(current) # If the statement is wrong, then execute this line.

            path = path + [current]

            if current == goal:
                return path # Final execution after reaching the goal and no more iteration in while loop

            for dx, dy, move_cost in moves:
                nx, ny = current[0] + dx, current[1] + dy # update current cell position according to neighour
                if 0 <= nx < w and 0 <= ny < h: # Checking for boundry condition of the map
                    if self.map[ny][nx] == 0: # checking for empty space inside occupancy grid
                        heapq.heappush(open_list, (
                            cost + move_cost + self.heuristic((nx, ny), goal),
                            cost + move_cost,
                            (nx, ny),
                            path
                        ))

        return None

    def heuristic(self, a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1]) # calculate the diagonal distance between current cell and goal

if __name__ == '__main__':
    planner = AStarPlanner()
    rospy.spin()
