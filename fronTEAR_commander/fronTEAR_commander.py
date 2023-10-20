#! /usr/bin/env python3
"""
FronTEAR_commander.py

Main file which compiles the frontier exploration modules to be deployed as a
ROS2 package.

This package uses Nav2 for use in ROS2

Author:
    METR4202 Sem 2 2023 Team 5

Recognition:
    Special thanks to Franklin 'Frankie' the Waffle-Pi for being our test
    subject.
"""

import sys
import time
import rclpy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg  import OccupancyGrid, Odometry
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.node import Node
from queue import PriorityQueue
from enum import Enum

OCC_THRESHOLD = 50
MIN_FRONTIER_SIZE = 20

FREE_SPACE = 0
NOGO_SPACE = 100
UNKNOWN_SPACE = -1
SCAN_RESOLUTION = 0.05

class OccupancyGrid2d():
    """
    OccupancyGrid2d

    Stores the map's occupancy grid and assigns them cost values.
    Also handles conversion from costmap coodinate to world coordinate.

    Methods
        (+) getCost(mx, my)
        (+) getSize()
        (+) getSizeX(mx, my)
        (+) getSizeY(mx, my)
        (+) mapToWorld(mx, my)
        (+) worldToMap(mx, my)

    Reference
        SeanReg - nav2_wavefront_frontier_exploration Repo
        https://github.com/SeanReg/nav2_wavefront_frontier_exploration

    """
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        """
        Initialises the occupancy grid by taking the OccupancyGrid from a
        map subscription callback function.
        """
        self.map = map

    def getCost(self, mx: int, my: int) -> int:
        """
        Returns the cost (from CostValues) of the tile at (mx, my) occupancy
        grid coordinates.

        Params:
            mx (int): x-coordinate on the occupancy grid
            my (int): y-coordinate on the occupancy grid

        Returns:
            int: cost of the tile
        """
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self) -> tuple:
        """
        Returns:
            tuple: (width, height) of the occupancy grid
        """
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self) -> int:
        """
        Returns:
            int: Width of the occupancy grid
        """
        return self.map.info.width

    def getSizeY(self) -> int:
        """
        Returns:
            int: Height of the occupancy grid
        """
        return self.map.info.height

    def mapToWorld(self, mx: float, my: float) -> tuple:
        """
        Converts occupancy grid map (x, y) coordinates to their respective world
        coordinates.

        Params:
            wx (float): occupancy grid x-coordinate
            wy (float): occupancy grid y-coordinate

        Returns:
            tuple: (x, y) on the world map
        """
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx: float, wy: float) -> tuple:
        """
        Converts world (x, y) coordinates to their respective map coordinates
        on the occupancy grid

        Params:
            wx (float): world x-coordinate
            wy (float): world y-coordinate

        Raises:
            Exception: Out-of-bounds if world coordinates do not map to an
                        occupancy grid coordinate

        Returns:
            tuple: (x, y) on the occupancy grid
        """

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

def centroid(arr: list, costmap: OccupancyGrid2d) -> tuple:
    """
    Determines the coordinate of the centre of the array

    Params:
        arr (list): list of (x, y) coordinates of frontier points
        costmap (OccupancyGrid2d): costmap of the current environment

    Returns:
        tuple: the (x, y) coordinate of the centre point of the array
    """
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])

    return sum_x/length, sum_y/length

class FronTEARCommander(Node):
    """
    Main node for frontier exploration.

    Methods:
        (+) nearUnknown() - identifies if near unknown points
        (+) seggregateFrontiers() - generates priority queue of cluster
        (+) getCluster() - determines the cluster from given point
        (+) moveToFrontier() - calls seggregateFrontiers and setWaypoints
        (+) setWaypoints() - adds the new waypoint(s) to the queue for publishing

    """

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_commanderer', namespace='')

        ### VARIABLES ###
        self.waypoints = None
        self.currentPose = None
        self.waypoint_counter = 0
        self.is_complete = False
        self.tree = False
        self.initial_pose_received = False
        self.costmap = None
        self.explored_waypoints = []
        self.costmap_updated = False
        self.recover = False

        ### SUBSCRIBERS ###
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.btLogCallback,
            10)

        self.costmapClient = self.create_client(GetCostmap,
                                                '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')

        self.model_pose_sub = self.create_subscription(Odometry,
                                                       '/odom',
                                                       self.poseCallback,
                                                       10)

        self.costmapSub = self.create_subscription(OccupancyGrid,
                                                   '/map',
                                                   self.occupancyGridCallback,
                                                   10)

        ### PUBLISHERS ###
        self.pose = self.create_publisher(PoseStamped,'goal_pose',10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Running FronTEAR_Commander...')

    ### CALLBACK FUNCTIONS ####################################################

    def btLogCallback(self, msg:BehaviorTreeLog):
        """ Behavior Tree Log Callback

        Checks whether the robot is in an IDLE state and moves to to the frontier.
        """
        self.tree = False

        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE':
                self.moveToFrontier()

        self.tree = False

    def occupancyGridCallback(self, msg):
        """
        Constructs the occupancy grid (-1, 0, 100) values of the global map
        """
        self.costmap = OccupancyGrid2d(msg)
        self.costmap_updated = True

    def poseCallback(self, msg):
        """
        Updates the current pose
        """
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True

    def near_unknown(self, x: int, y: int, costmap: OccupancyGrid2d) -> bool:
        """
        Checks whether at least one point near (x, y) is an unknown point.

        Params:
            x (int): x-coordinate on the occupancy grid
            y (int): y-coordinate on the occupancy grid
            costmap (OccupancyGrid2d): the current costmap of the environment

        Returns:
            bool: True if near an unknown square (i.e. is a frontier), false
                    otherwise.
        """
        for i in range(-1, 2):
            for j in range(-1, 2):
                try:
                    value = costmap.getCost(x + i, y + j)
                except IndexError:
                    return False
                if value == -1:
                    return True
        return False

    def seggregateFrontiers(self, costmap: OccupancyGrid2d) -> PriorityQueue:
        """
        Handles the frontier points, classifying them as good or shit waypoints,
        then generates the clusters and adds them to a priority queue, where the
        largest cluster has highest priority.

        Params:
            costmap (OccupancyGrid2d): _description_

        Returns:
            PriorityQueue: _description_
        """
        self.shit_points = []
        self.good_points = []
        n_rows = costmap.getSizeX()
        n_cols = costmap.getSizeY()
        current_x, current_y = costmap.worldToMap(self.currentPose.position.x, self.currentPose.position.y)

        # Divide good and shit points
        for i in range(n_rows):
            for j in range(n_cols):
                value = costmap.getCost(i, j)
                if value == FREE_SPACE and self.near_unknown(i, j, costmap):
                    self.good_points.append((i, j))
                elif value == NOGO_SPACE and self.near_unknown(i, j, costmap):
                    self.shit_points.append((i, j))
        self.num_good_points = len(self.good_points)
        print("Number of good points: ", len(self.good_points))

        # Cluster the frontiers
        frontier_groups = PriorityQueue() # (length, frontiers[])
        count = 0
        largest = 0
        total_clusters = 0

        while len(self.good_points) > 0:
            point = self.good_points.pop(0)
            cluster = self.getCluster(point)
            cluster_size = len(cluster)
            centre_x, centre_y = centroid(cluster, self.costmap)
            euclid_distance = math.sqrt((abs(centre_x - current_x)**2 + abs(centre_y - current_y)**2))
            manhattan_distance = abs(centre_x - current_x) + abs(centre_y - current_y)
            total_clusters += 1
            if (-1 * cluster_size < largest):
                largest = -1 * cluster_size

            # Do not search clusters that are too small
            if cluster_size > MIN_FRONTIER_SIZE:
                print(f"Euclidean distance: {euclid_distance}, Manhattan distance: {manhattan_distance}, Cluster size: {cluster_size}, Weight: {(manhattan_distance*2) - (0.3 * cluster_size)}")
                frontier_groups.put((manhattan_distance, cluster))
                count += 1
        print(f"Frontier size: {count}, number of possible clusters: {total_clusters}, largest cluster size: {largest}")

        return frontier_groups

    def getCluster(self, point: tuple) -> list:
        """
        Constructs a cluster of good waypoints starting at a particular point,
        using bread-first search (BFS).
        If a waypoint is added to a cluster, it is removed from the good waypoints list.

        Nearby points are defined as good points that are within a 5x5 grid
        from the robot's current position.

        Params:
            point (tuple): (x, y) coordinates of the starting waypoint

        Returns:
            list: list of (x, y) waypoints in the cluster
        """
        cluster = [point]
        nearby_points = set((a + point[0], b + point[1])
            for a in range(-2, 3, 1)
            for b in range(-2, 3, 1)
            if (a + point[0], b + point[1]) in self.good_points)

        while len(nearby_points) > 0:
            p = nearby_points.pop()
            self.good_points.remove(p)
            cluster.append(p)

            for a in range(-2, 3, 1):
                for b in range(-2, 3, 1):
                    if (a + p[0], b + p[1]) in self.good_points:
                        nearby_points.add((a + p[0], b + p[1]))

        return cluster

    def moveToFrontier(self):
        """
        Handles the moving to the next frontier.
        """

        # Do nothing when costmap is not loaded
        if self.costmap is None:
            return

        self.frontier_item = 0
        self.costmap_updated = False
        self.tree = True

        # Start sleep to give map time to update
        self.info_msg("Starting sleep...")
        time.sleep(1)
        self.info_msg("Awoken")

        # Create the clusters and get the priority queue of the clusters
        frontier = self.seggregateFrontiers(self.costmap)

        # If frontier is empty, end exploration
        if frontier.empty():
            print("No more frontiers")
            self.is_complete = True
            return

        # Loop through frontiers in the frontier priority queue until an
        # un-traversed waypoint is found
        while not frontier.empty():
            self.frontier_item = frontier.get()
            waypoint = centroid(self.frontier_item[1], self.costmap)

            # Get the world coordinates
            wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])
            w = round(wp[0], 2), round(wp[1], 2)

            # Check if waypoint created is already explored
            if w not in self.explored_waypoints:
                self.explored_waypoints.append(w)
                self.setWaypoints([wp])
                print(f"Publishing waypoint: ({wp[0], wp[1]})")
                self.pose.publish(self.waypoints[0])
                return

        print(abs(self.frontier_item[0]))

        # If there are more frontiers but exploration is complete, send it to
        # the last waypoint
        if self.num_good_points > 50:
            time.sleep(5)
            print("Starting Recovery...")
            waypoint = centroid(self.frontier_item[1], self.costmap)
            wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])
            w = round(wp[0], 2), round(wp[1], 2)

            self.explored_waypoints.append(w)
            self.setWaypoints([wp])
            print(f"Publishing waypoint: ({wp[0], wp[1]})")
            self.pose.publish(self.waypoints[0])
            return

        self.is_complete = True
        print("All frontiers searched")

    def setWaypoints(self, waypoints: list) -> None:
        """
        Converts the waypoints to PoseStamped values and sets the next waypoint
        to be published to the list provided.

        Params:
            waypoints (list): a list of (x, y) waypoints to be converted into
                                PoseStamped variables to be sent to the waypoint
                                follower.
        """
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            self.waypoints.append(msg)


    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()

    commander = FronTEARCommander()
    goal_counter = 0

    try:
        while rclpy.ok():
            # Your flag-checking logic here
            if goal_counter < commander.waypoint_counter:
                commander.info_msg(f"Goals submitted: {commander.waypoint_counter}")
                goal_counter += 1

            if commander.recover:
                print("Requested recovery")
                commander.recover = False
                commander.moveToFrontier()
                rclpy.spin_once(commander)

            if commander.is_complete:
                print(f"Sent {goal_counter} waypoints.")
                break  # Exit the loop to stop the node
            rclpy.spin_once(commander)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
