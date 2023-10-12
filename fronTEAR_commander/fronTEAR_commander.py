#! /usr/bin/env python3

import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import statistics

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.time import Duration
from queue import SimpleQueue, PriorityQueue

from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 50
MIN_FRONTIER_SIZE = 30

FREE_SPACE = 0
NOGO_SPACE = 100
UNKNOWN_SPACE = -1

class RecoveryStrategy:
    def __init__(self):
        self.node = rclpy.create_node('recovery_strategy')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.stopped = False

        # Define the recovery behavior
        self.backup_duration = 2.0  # Duration to back up in seconds
        self.rotation_duration = 2.0  # Duration to rotate in seconds

    def stop_robot(self):
        self.stopped = True
        self.publisher.publish(Twist())

    def backup_and_rotate(self):
        # Back up the robot
        twist = Twist()
        twist.linear.x = -0.2  # Adjust linear velocity as needed
        self.publisher.publish(twist)

        # Calculate the target time for stopping
        stop_time = self.node.get_clock().now() + Duration(seconds=self.backup_duration)
        while self.node.get_clock().now() < stop_time:
            pass

        # Stop the robot
        self.stop_robot()


    def run_recovery(self):
        self.node.get_logger().info('Starting recovery strategy')
        self.backup_and_rotate()
        self.node.get_logger().info('Recovery strategy complete')


class Costmap2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 253
        LethalObstacle = 254
        NoInformation = 255

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.metadata.size_x, self.map.metadata.size_y)

    def getSizeX(self):
        return self.map.metadata.size_x

    def getSizeY(self):
        return self.map.metadata.size_y

    def __getIndex(self, mx, my):
        return my * self.map.metadata.size_x + mx

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        # if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
        #     raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

class FrontierCache():
    """_summary_

    Returns:
        _type_: _description_
    """
    cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}

    def __repr__(self) -> str:
        return list(self.cache)
        pass

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

def centroid(arr) -> tuple:
    """
    Determines the coordinate of the centre of the array

    Args:
        arr (_type_): _description_

    Returns:
        tuple: the (x, y) coordinate of the centre point of the array
    """
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

def findFree(mx: float, my: float, costmap: OccupancyGrid2d) -> tuple:
    """
    Finds the closest valid free space to the coordinate (mx, my)

    Args:
        mx (float): x-coordinate
        my (float): y-coordinate
        costmap (OccupancyGrid2d): _description_

    Returns:
        tuple: (x, y) coordinate of the frontier point is the costmap
    """
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        # if the coordinate provided is a
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose: PoseStamped, costmap: OccupancyGrid2d, logger) -> list:
    """
    Uses the Wavefront Frontier Detector (WFD) algorithm which uses two nested
    Bread-First Searches (BFS). It only scans the known regions of the occupancy
    grid at each run of the algorithm.

    1. Map-Open-List: points that have been enqueued by
        the outer BFS (mapPointQueue).
    2. Map-Close-List: points that have been dequeued by
        the outer BFS (mapPointQueue).
    3. Frontier-Open-List: points that have been enqueued
        by the inner BFS (frontierQueue).
    4. Frontier-Close-List: points that have been dequeued
        by the inner BFS (frontierQueue).

    Args:
        point (FrontierPoint): Current position of the robot
        costmap (OccupancyGrid2D): the current occupancy grid costmap
        logger (_type_): _description_

    Returns:
        list: _description_

    Reference:
    Adapted from https://github.com/SeanReg/nav2_wavefront_frontier_exploration
    originally by SeanReg. Used under MIT License.
    """

    fCache = FrontierCache()

    # Clear frontier Cache on each run
    fCache.clear()

    #TODO: change these lists to queues for optimisation
    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []
    print("Costmap size: ", costmap.getSizeX(), costmap.getSizeY())
    radius = min(abs(costmap.getSizeX() - mx), abs(my - costmap.getSizeY()))
    print(f"Position ({mx}, {my}) Size: ({costmap.getSizeX()}, {costmap.getSizeY()}), radius: {radius}")

    # While there are mapPoints to be explored (i.e. frontier nodes not determined)
    while len(mapPointQueue) > 0:
        # get the first point
        p = mapPointQueue.pop(0)

        # if the point provided has already been classified, skip
        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        # check whether the point is a valid point on the costmap
        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value


            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            # check whether the neighboring point is free space
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                # Append the neighboring point into the mapPointQueue if any of
                # the neighboring points (to this point) is a free space
                # (i.e. v becomes part of the frontier)
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        # Set the point classification as closed
        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


def getNeighbors(point: FrontierPoint, costmap: OccupancyGrid2d, fCache: FrontierCache) -> list:
    """
    Gets the neighboring points on the costmap from a specified point

    Args:
        point (FrontierPoint): provided frontier (x, y) point to check validty
        costmap (OccupancyGrid2D): the current occupancy grid costmap
        fCache (FrontierCache): cache which contains the costmap coordinates (x, y)
                                with a given point (x, y)

    Returns:
        list: containing all neighboring coordinates (x, y) to a given point in
                a format that can be converted into a PoseStamped variable
    """
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point: FrontierPoint, costmap: OccupancyGrid2d, fCache: FrontierCache) -> bool:
    """
    Checks whether the point (x, y) provided is a valid point in the costmap

    Args:
        point (FrontierPoint): provided frontier (x, y) point to check validty
        costmap (OccupancyGrid2D): the current occupancy grid costmap
        fCache (FrontierCache): cache which contains the costmap coordinates (x, y)
                                with a given point (x, y)

    Returns:
        bool: True if frontier is valid, otherwise False
    """
    # If the (x, y) point returns no information from the costmap, then it is an
    # invalid value
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

class WaypointFollowerTest(Node):

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_tester', namespace='')

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
                if self.costmap is None:
                    return

                self.tree = True
                frontier = self.seggregate_frontiers(self.currentPose, self.costmap)
                if frontier.empty():
                    print("No more frontiers")
                    self.is_complete = True
                    return

                waypoints = frontier.get()[1]
                waypoint = waypoints.pop(0)

                wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])

                while len(waypoints) > 0:
                    wp = self.costmap.mapToWorld(waypoint[0], waypoint[1])
                    w = round(wp[0], 1), round(wp[1], 1)
                    if w not in self.explored_waypoints:
                        self.explored_waypoints.append(w)
                        break
                    waypoint = waypoints.pop(0)

                self.setWaypoints([wp])
                self.pose.publish(self.waypoints[0])
                #self.moveToFrontiers()
                break

        self.tree = False

    def occupancyGridCallback(self, msg):
        """
        Constructs the occupancy grid (-1, 0, 100) values of the global map
        """
        self.costmap = OccupancyGrid2d(msg)

    def poseCallback(self, msg):
        """
        Updates the current pose
        """
      #  self.info_msg('Received amcl_pose')
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True

    def is_close_to_waypoint(self, position, waypoint: PoseStamped, tolerance: float) -> Bool:
        """
        Checks whether a waypoint is close to the current position by calculating
        the Euclidean distance between the two points and comparing to the tolerance
        distance.

        Args
            position (PoseStamped.pose.position): The current position of the robot
            waypoint (PoseStamped): The new waypoint check check proximity
            tolerance (float): The radius from the robot's current position which
                                is considered close to waypoint

        Returns
            True if distance is under tolerance, False otherwise
        """
    # Check if the current position is close to the waypoint within the specified tolerance
        distance = math.sqrt((position.x - waypoint.pose.position.x)**2 + (position.y - waypoint.pose.position.y)**2)
        return distance < tolerance

    ### NEW BANANA CODE #######################################################

    def get_waypoint(self, frontiers: PriorityQueue) -> PoseStamped:
        new_waypoint = PoseStamped()
        p = frontiers.get()
        new_waypoint.pose.position.x = p[1][0]
        new_waypoint.pose.position.y = p[1][1]

        return new_waypoint

    def near_unknown(self, x: int, y: int, costmap: OccupancyGrid2d) -> bool:
        for i in range(-1, 1):
            for j in range(-1, 1):
                value = costmap.getCost(x + i, y + j)
                if value == -1:
                    return True
        return False

    def seggregate_frontiers(self, pose: PoseStamped, costmap: OccupancyGrid2d) -> PriorityQueue:
        self.shit_points = []
        self.good_points = []
        n_rows = costmap.getSizeX()
        n_cols = costmap.getSizeY()

        for i in range(n_rows):
            for j in range(n_cols):
                value = costmap.getCost(i, j)
                if value == FREE_SPACE and self.near_unknown(i, j, costmap):
                    self.good_points.append((i, j))
                elif value == NOGO_SPACE and self.near_unknown(i, j, costmap):
                    self.shit_points.append((i, j))

        # Cluster the frontiers
        frontier_groups = PriorityQueue() # (length, frontiers[])
        count = 0
        while len(self.good_points) > 0:
            point = self.good_points.pop(0)
            cluster = self.get_cluster(point)
            cluster_size = len(cluster)
            if cluster_size > OCC_THRESHOLD: # should be MIN_FRONTIER_SIZE
                frontier_groups.put((-1 * len(cluster), cluster))
                count += 1
        print(f"Frontier size: {count}")
        return frontier_groups

    def get_cluster(self, point: tuple) -> list:
        # if len(points) == 1:
        #    if points[0] in self.good_points:
        #        self.good_points.remove(points[0])
        #        return points[0]
        cluster = []
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

    def moveToFrontiers(self):
        """
        Calls the getFrontier function and selects the frontier with the median
        distance to the robot's current position. Then, publishes the waypoint
        """

        if self.tree:
            twist = Twist()
            twist.angular.z = 0.5
            self.twist_pub.publish(twist)

            # # recovery = RecoveryStrategy()
            # # print(self.get_result())
            # # if self.get_result() == -1 or self.get_result() == 0:
            # #     print ("recovery")
            # #     recovery.run_recovery()

            # Get the current robot's position
            current_position = self.currentPose.position

            # Check if the current waypoint is already set and if it's close to the current position

            frontiers = self.seggregate_frontiers(self.currentPose, self.costmap)

            #frontiers = [x for x in frontier if x not in self.visitedf]

            if len(frontiers) == 0:
                self.info_msg('No more frontiers, exploration complete')
                self.is_complete = True
                return

            location = None
            largestDist = 0
            largestDist2 = 0
            all_loc = []

            self.info_msg(f'Frontiers: {frontiers}')
            for f in frontiers:
                dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))
                all_loc.append(dist)
                # if  dist > largestDist:
                #     largestDist = dist
                #     location = [f]

            med_value = statistics.median(all_loc)
            closest_el = min(all_loc, key=lambda x: abs(x - med_value))
            index = all_loc.index(closest_el)
            location = [frontiers[index]]

            self.info_msg(f'World points {location}')
            self.setWaypoints(location)

            if self.waypoints and self.is_close_to_waypoint(current_position, self.waypoints[0], tolerance=2):
                self.info_msg('Already at or close to the current waypoint')
                index = all_loc.index(max(all_loc))
                all_loc.pop(index)
                frontiers.pop(index)
                counter = -1
                for loc in all_loc:
                    counter +=1
                    if loc > largestDist2:
                        largestDist2 = loc
                        location = [frontiers[counter]]
                        self.info_msg('finding new waypoint...')
                self.info_msg('setting new waypoint')
                self.setWaypoints(location)
            # if self.waypoints and self.is_close_to_waypoint(current_position, self.waypoints[0], tolerance=0.1):
            #     self.info_msg('Already at or close to the current waypoint')
            #     all_loc.pop(index)
            #     frontiers.pop(index)
            #     closest_el = min(all_loc, key=lambda x: abs(x - med_value))
            #     index = all_loc.index(closest_el)
            #     location = [frontiers[index]]
            #     self.info_msg('finding new waypoint...')
            #     self.info_msg('setting new waypoint')
            #     self.setWaypoints(location)

            # self.visitedf.append(location[0])

            self.info_msg('Sending goal request...')

            self.pose.publish(self.waypoints[0])
            self.waypoint_counter += 1

            self.info_msg('Goal accepted')


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

    # wps = [[-0.52, -0.54], [0.58, -0.55], [0.58, 0.52]]
    starting_pose = [-1.0, -0.5]

    # wps = [[-20.52, -20.54], [20.58, -20.55], [20.58, 20.52]]
    # starting_pose = [-2.0, -2.0]

    test = WaypointFollowerTest()
    goal_counter = 0
    #test.dumpCostmap()
    #test.setWaypoints(wps)

    retry_count = 0
    retries = 2
    while not test.initial_pose_received and retry_count <= retries:
        retry_count += 1
        test.info_msg('Setting initial pose')
        test.setInitialPose(starting_pose)
        test.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(test, timeout_sec=1.0)  # wait for poseCallback

    while test.costmap == None:
        test.info_msg('Getting initial map')
        rclpy.spin_once(test, timeout_sec=1.0)

    #test.moveToFrontiers()
    try:
        while rclpy.ok():
            # Your flag-checking logic here
            if goal_counter < test.waypoint_counter:
                test.info_msg(f"Goals submitted: {test.waypoint_counter}")
                goal_counter += 1

            if test.is_complete:
                break  # Exit the loop to stop the node
            rclpy.spin_once(test)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
