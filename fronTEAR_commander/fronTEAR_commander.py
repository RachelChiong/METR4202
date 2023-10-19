#! /usr/bin/env python3
"""
Welcome to the bananas-kms branch (K-Means-Squared)
Possible optimisation techniques:
    -   weighting the frontier clusters based on their proximity to the robot
        (currently goes back and forth prioritising biggest frontier)
    -   Changing costmap frontier scope to be more local to the robot
    -   not storing the shit points and reducing double counting
"""

import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg  import OccupancyGrid, Odometry
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from queue import PriorityQueue

from enum import Enum
import numpy as np
import math
from sklearn.cluster import KMeans

OCC_THRESHOLD = 50
MIN_FRONTIER_SIZE = 30

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
    radius = 50

    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getKMSTuples(self):

        grid = []

        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                world_x, world_y = self.mapToWorld(i, j)
                grid.append((world_x, world_y, self.getCost(i, j)))
        return grid

    def getLocalKMSTuples(self, current_x: int, current_y: int):

        grid = []

        # Determine local costmap bounds
        ux = current_x + self.radius if current_x + self.radius < self.map.info.width else self.map.info.width
        lx = current_x - self.radius if current_x - self.radius >= 0 else 0
        uy = current_y + self.radius if current_y + self.radius < self.map.info.height else self.map.info.height
        ly = current_y - self.radius if current_y - self.radius >= 0 else 0

        for i in range(lx, ux):
            for j in range(ly, uy):
                world_x, world_y = self.mapToWorld(i, j)
                grid.append((world_x, world_y, self.getCost(i, j)))
        return grid

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
        self.recover = False

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

        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.cameraCallback, 10)

        ### PUBLISHERS ###
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)

        self.pose = self.create_publisher(PoseStamped,'goal_pose',10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Running FronTEAR_Commander...')

    ### CALLBACK FUNCTIONS ####################################################

    def cameraCallback(self, msg):
        # TODO: Implement later for A1
        pass

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
        wx, wy = round(self.currentPose.position.x, 1), round(self.currentPose.position.y, 1)
        self.add_explored_waypoints((wx, wy))
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
        for i in range(-1, 2):
            for j in range(-1, 2):
                try:
                    value = costmap.getCost(x + i, y + j)
                except IndexError:
                    return False
                if value == -1:
                    return True
        return False

    def seggregate_frontiers(self, costmap: OccupancyGrid2d) -> PriorityQueue:
        """
        A new way to seggregate - but with KMS
        """
        current_x, current_y = costmap.worldToMap(self.currentPose.position.x, self.currentPose.position.y)
        # kmsTuples = costmap.getKMSTuples()
        kmsTuples = costmap.getLocalKMSTuples(current_x, current_y)
        kmeans = KMeans(n_clusters=12)
        kmeans.fit(kmsTuples)
        print("KMS Clusters: ", kmeans, kmeans.cluster_centers_)

        frontier_groups = PriorityQueue() # (length, frontiers[])
        min_cost = float('inf')
        for center in kmeans.cluster_centers_:
            cluster_x, cluster_y, _ = center
            map_cluster = costmap.worldToMap(cluster_x, cluster_y)
            manhattan = abs(map_cluster[0] - current_x) + abs(map_cluster[1] - current_y)
            if manhattan < 5 or (round(cluster_x, 1), round(cluster_y, 1)) in self.explored_waypoints:
                continue
            cost = self.calculate_cost(costmap, current_x, current_y, map_cluster[0], map_cluster[1])
            if cost < min_cost:
                min_cost = cost

            print(f"({cluster_x}, {cluster_y}, cost: {cost}, distance: {manhattan})")
            frontier_groups.put((cost, (cluster_x, cluster_y)))

        # if min_cost > 1000:
        #     frontier_groups = self.seggregate_frontiers(costmap)
        return frontier_groups

    def calculate_cost(self, costmap: OccupancyGrid2d, start_x: int, start_y: int, end_x: int, end_y: int) -> float:

        num_points = int(math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2))
        print(f"Distance points: {num_points}")
        cost = 0
        # Linear interpolation to find the points
        for i in range(num_points + 1):
            t = i / num_points
            x = round(start_x + t * (end_x - start_x))
            y = round(start_y + t * (end_y - start_y))
            cost_temp = costmap.getCost(x, y)

            if cost_temp == -1:
                cost_temp = -10
            elif cost_temp == 100:
                cost_temp = 100
            print(cost_temp)

            cost += cost_temp

        return cost


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

        self.frontier_item = 0

        while not frontier.empty():
            self.frontier_item = frontier.get()
            wp = (self.frontier_item[1][0], self.frontier_item[1][1])
            w = round(wp[0], 1), round(wp[1], 1)

            print(f"cost: {self.frontier_item[0]} Going to: ({w[0]}, {w[1]})")

            if w not in self.explored_waypoints:
                # print(self.explored_waypoints)
                self.add_explored_waypoints(w)
                self.explored_waypoints.append(w)
                self.setWaypoints([wp])
                print(f"Publishing waypoint: ({wp[0], wp[1]})")
                self.pose.publish(self.waypoints[0])
                return
            print("Moving to next waypoint...")

        print(abs(self.frontier_item[0]))
        if self.num_good_points > 50:
            time.sleep(5)
            print("Starting Recovery...")
            self.recover = True
            return
            waypoint = centroid(self.frontier_item[1], self.costmap)
            wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])
            w = round(wp[0], 2), round(wp[1], 2)

            self.explored_waypoints.append(w)
            self.setWaypoints([wp])
            print(f"Publishing waypoint: ({wp[0], wp[1]})")
            self.pose.publish(self.waypoints[0])
            return


            print(f"frontier discarded: ({wp[0]}, {wp[1]})")

        self.is_complete = True
        print("All frontiers searched")

    def add_explored_waypoints(self, waypoint: tuple) -> None:
        offsets = (-0.1, 0, 0.1)
        for offset_x in offsets:
            for offset_y in offsets:
                self.explored_waypoints.append((round(waypoint[0] + offset_x, 1), round(waypoint[1] + offset_y, 1)))


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
    goal_counter = 0
    #commander.dumpCostmap()
    #commander.setWaypoints(wps)

    retry_count = 0
    retries = 2
    while not commander.initial_pose_received and retry_count <= retries:
        retry_count += 1
        commander.info_msg('Setting initial pose')
        commander.setInitialPose(starting_pose)
        commander.info_msg('Waiting for amcl_pose to be received')
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

            if commander.recover:
                print("Requested recovery")
                commander.recover = False
                commander.move_to_frontier()
                rclpy.spin_once(commander)

            if commander.is_complete:
                break  # Exit the loop to stop the node
            rclpy.spin_once(commander)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
