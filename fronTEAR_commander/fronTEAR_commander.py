#! /usr/bin/env python3
"""
Possible optimisation techniques:
    -   weighting the frontier clusters based on their proximity to the robot
        (currently goes back and forth prioritising biggest frontier)
    -   Changing costmap frontier scope to be more local to the robot
    -   not storing the shit points and reducing double counting
"""

import sys
import time
import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import *
from geometry_msgs.msg import Twist
from rclpy.node import Node
from queue import PriorityQueue
from enum import Enum


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

    Reference:
    """
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
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

    # cx = round(sum_x/length)
    # cy = round(sum_y/length)
    # new_x, new_y = check_for_collisions(costmap, cx, cy)
    # print(f"original: ({cx}, {cy}), new: ({new_x}, {new_y})")
    # return new_x, new_y
    return sum_x/length, sum_y/length

def check_for_collisions(costmap: OccupancyGrid2d, cx: int, cy: int) -> tuple:
    """
    Checks for collisions based on the given centroid coordinates on the occupancy
    grid and fixes it
    Params:
        costmap (OccupancyGrid2d): the occupancy grid of the environment
        cx (int): x-coordinate on the occupancy grid
        cy (int): y-coordinate on the occupancy grid

    Returns:
        tuple: (x, y) coordinates of the new centroid

    Note: NOT USED
    """
    ux = cx + 10
    lx = cx - 10
    uy = cy + 10
    ly = cy - 10

    shifts = [(-5, 0), (5, 0), (0, -5), (0, 5)]
    # start left shift
    while lx <= 0:
        lx += 5
        ux += 5
    while ux > costmap.getSizeX():
        lx -= 5
        ux -= 5

    while ly <= 0:
        ly += 5
        uy += 5
    while uy > costmap.getSizeY():
        ly -= 5
        uy -= 5

    valid = False
    while not valid:
        # check left edge
        if costmap.getCost(lx, ly) == 100 or costmap.getCost(lx, uy) == 100:
            lx += 5
            ux += 5

        elif costmap.getCost(ux, ly) == 100 or costmap.getCost(ux, uy) == 100:
            lx -= 5
            ux -= 5

        elif costmap.getCost(lx, ly) == 100 or costmap.getCost(lx, uy) == 100:
            ly += 5
            uy += 5

        elif costmap.getCost(ux, ly) == 100 or costmap.getCost(ux, uy) == 100:
            ly -= 5
            uy -= 5

        else:
            valid = True

    # return new centre point
    return ux - lx // 2, uy - ly // 2

class FronTEARCommander(Node):

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_commanderer', namespace='')

        ### VARIABLES ###
        self.visitedf = []
        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.lastWaypoint = None
        self.waypoint_counter = 0
        self.is_complete = False
        self.tree = False
        self.initial_pose_received = False
        self.goal_handle = None
        self.costmap = None
        self.explored_waypoints = []
        self.costmap_updated = False

        ### SUBSCRIBERS ###
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)


        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')

        self.model_pose_sub = self.create_subscription(Odometry,
                                                       '/odom', self.poseCallback, 10)
        # self.costmap_update_sub = self.create_subscription(Costmap, self.costmapUpdateCallback, '/global_')

        # self.costmapSub = self.create_subscription(Costmap(), '/global_costmap/costmap_raw', self.costmapCallback, 10)
        self.costmapSub = self.create_subscription(OccupancyGrid, '/map', self.occupancyGridCallback, 10)

        ### PUBLISHERS ###
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)

        self.pose = self.create_publisher(PoseStamped,'goal_pose',10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Running FronTEAR_Commander...')

    ### CALLBACK FUNCTIONS ####################################################
    def bt_log_callback(self, msg:BehaviorTreeLog):
        """ Behavior Tree Log Callback

        Checks whether the robot is in an IDLE state and moves to to the frontier.
        """
        self.tree = False

        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE':
                self.move_to_frontier()

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
      #  self.info_msg('Received amcl_pose')
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True

    ### NEW BANANA CODE #######################################################

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
        for i in range(-1, 1):
            for j in range(-1, 1):
                value = costmap.getCost(x + i, y + j)
                if value == -1:
                    return True
        return False

    def seggregate_frontiers(self, costmap: OccupancyGrid2d) -> PriorityQueue:
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

        # Divide good and shit points
        for i in range(n_rows):
            for j in range(n_cols):
                value = costmap.getCost(i, j)
                if value == FREE_SPACE and self.near_unknown(i, j, costmap):
                    self.good_points.append((i, j))
                elif value == NOGO_SPACE and self.near_unknown(i, j, costmap):
                    self.shit_points.append((i, j))
        print("Number of good points: ", len(self.good_points))

        # Cluster the frontiers
        frontier_groups = PriorityQueue() # (length, frontiers[])
        count = 0
        largest = 0
        total_clusters = 0

        while len(self.good_points) > 0:
            point = self.good_points.pop(0)
            cluster = self.get_cluster(point)
            cluster_size = len(cluster)
            total_clusters += 1
            if (-1 * cluster_size < largest):
                largest = -1 * cluster_size

            if cluster_size > 10:
                # print(f"cluster size: {-1 * cluster_size}")
                frontier_groups.put((-1 * cluster_size, cluster))
                count += 1
        print(f"Frontier size: {count}, number of possible clusters: {total_clusters}, largest cluster size: {largest}")

        return frontier_groups

    def get_cluster(self, point: tuple) -> list:
        """
        Constructs a cluster of good waypoints starting at a particular point.
        If a waypoint is added to a cluster, it is removed from the good waypoints list.

        Params:
            point (tuple): (x, y) coordinates of the starting waypoint

        Returns:
            list: list of (x, y) waypoints in the cluster
        """
        cluster = [point]
        nearby_points = set((a + point[0], b + point[1])
            for a in range(-2, 2, 1)
            for b in range(-2, 2, 1)
            if (a + point[0], b + point[1]) in self.good_points)

        while len(nearby_points) > 0:
            p = nearby_points.pop()
            self.good_points.remove(p)
            cluster.append(p)

            for a in range(-2, 2, 1):
                for b in range(-2, 2, 1):
                    if (a + p[0], b + p[1]) in self.good_points:
                        nearby_points.add((a + p[0], b + p[1]))

        return cluster

    def move_to_frontier(self):
        """
        Handles the moving to the next frontier.
        """
        if self.costmap is None:
            return

        self.costmap_updated = False
        self.tree = True
        self.info_msg("Starting sleep...")
        time.sleep(1)
        self.info_msg("Awoken")
        frontier = self.seggregate_frontiers(self.costmap)
        if frontier.empty():
            print("No more frontiers")
            self.is_complete = True
            return

        while not frontier.empty():
            waypoint = centroid(frontier.get()[1], self.costmap)
            wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])
            w = round(wp[0], 2), round(wp[1], 2)

            if w not in self.explored_waypoints:
                print(self.explored_waypoints)
                self.explored_waypoints.append(w)
                self.setWaypoints([wp])
                print(f"Publishing waypoint: ({wp[0], wp[1]})")
                self.pose.publish(self.waypoints[0])
                return

            print(f"frontier discarded: ({wp[0]}, {wp[1]})")

        self.is_complete = True
        print("All frontiers searched")

    def dumpCostmap(self):
        """
        Requests costmap data (not used)
        """
        costmapReq = GetCostmap.Request()
        self.get_logger().info('Requesting Costmap')
        costmap = self.costmapClient.call(costmapReq)
        self.get_logger().info(f'costmap resolution {costmap.specs.resolution}')

    def setInitialPose(self, pose):
        """
        Sets the initial pose

        Params:
            pose (tuple): takes a tuple containing (x, y) of the inital pose
        """
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        #self.publishInitialPose()
        #time.sleep(5)

        self.init_pose1 = PoseStamped()
        self.init_pose1.pose.position.x = pose[0]
        self.init_pose1.pose.position.y = pose[1]
        self.init_pose1.header.frame_id = 'map'
        self.currentPose = self.init_pose1.pose
        self.publishInitialPose()
        self.pose.publish(self.init_pose1)
        time.sleep(5)


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
           # msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def publishInitialPose(self):
        """
        Publishes the initial pose
        """
        #self.pose.publish(self.init_pose)
        self.initial_pose_pub.publish(self.init_pose)


    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()

    starting_pose = [-1.0, -0.5]

    commander = FronTEARCommander()
    commander.initial_pose_received = False

    print("Commander created")
    goal_counter = 0
    #commander.dumpCostmap()
    #commander.setWaypoints(wps)

    retry_count = 0
    retries = 2
    while not commander.initial_pose_received and retry_count <= retries:
        retry_count += 1
        commander.info_msg('Setting initial pose')
        commander.setInitialPose(starting_pose)
        rclpy.spin_once(commander, timeout_sec=1.0)  # wait for poseCallback

    # while commander.costmap == None:
        # commander.info_msg('Getting initial map')
        # rclpy.spin_once(commander, timeout_sec=1.0)

    try:
        while rclpy.ok():
            # Your flag-checking logic here
            if goal_counter < commander.waypoint_counter:
                commander.info_msg(f"Goals submitted: {commander.waypoint_counter}")
                goal_counter += 1

            if commander.is_complete:
                break  # Exit the loop to stop the node
            rclpy.spin_once(commander)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
