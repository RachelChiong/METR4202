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
SCAN_RESOLUTION = 0.05

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

def centroid(arr: list, costmap: OccupancyGrid2d) -> tuple:
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

    # cx = round(sum_x/length)
    # cy = round(sum_y/length)
    # new_x, new_y = check_for_collisions(costmap, cx, cy)
    # print(f"original: ({cx}, {cy}), new: ({new_x}, {new_y})")
    # return new_x, new_y
    return sum_x/length, sum_y/length

RADIUS = 0.22

def check_for_collisions(costmap: OccupancyGrid2d, cx: int, cy: int) -> bool:
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


    def move_to_frontier(self):
        if self.costmap is None:
            return

        self.costmap_updated = False
        self.tree = True
        self.info_msg("Starting sleep...")
        time.sleep(1)
        self.info_msg("Awoken")
        frontier = self.seggregate_frontiers(self.currentPose, self.costmap)
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
        #     waypoint = waypoints.pop(0)
        self.is_complete = True
        print("All frontiers searched")
        #self.moveToFrontiers()

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
        print("Updated Occupancy Grid")
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
            if cluster_size > 0:
                print(f"cluster size: {-1 * cluster_size}")
                frontier_groups.put((-1 * cluster_size, cluster))
                count += 1
        print(f"Frontier size: {count}, number of possible clusters: {total_clusters}, largest cluster size: {largest}")
        return frontier_groups

    def get_cluster(self, point: tuple) -> list:
        # if len(points) == 1:
        #    if points[0] in self.good_points:
        #        self.good_points.remove(points[0])
        #        return points[0]
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
        #print(cluster)

        return cluster

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

    # while test.costmap == None:
        # test.info_msg('Getting initial map')
        # rclpy.spin_once(test, timeout_sec=1.0)

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