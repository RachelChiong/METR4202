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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.time import Duration 

from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 50
MIN_FRONTIER_SIZE = 30

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
        
        # if  (my > self.map.info.height or mx > self.map.info.width):
        #     raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

class FrontierCache():
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

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
           # print("hello")
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose, costmap, logger):

    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
  #  print (mx)
  #  print(my)
    freePoint = findFree(mx, my, costmap)
   # print("free point" + str(freePoint))
    start = fCache.getPoint(freePoint[0], freePoint[1])
   # print("start"+str(start))
    start.classification = PointClassification.MapOpen.value
   # print(start.classification)
    mapPointQueue = [start]
   # print("mapQ"+str(mapPointQueue))

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
           # print("yes")
           # print(frontierQueue)
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
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

   # print("frontiers" + str(frontiers))

    return frontiers
        

def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
      #  print("grid"+str(OccupancyGrid2d.CostValues.NoInformation.value))
      #  print("cost" +str(costmap.getCost(point.mapX, point.mapY)))
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
           # print("oops")
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

        self.visitedf = []
        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.lastWaypoint = None
        self.waypoint_counter = 0
    
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)

        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')
        self.initial_pose_received = False
        self.goal_handle = None

        self.model_pose_sub = self.create_subscription(Odometry,
                                                       '/odom', self.poseCallback, 10)

        # self.costmapSub = self.create_subscription(Costmap(), '/global_costmap/costmap_raw', self.costmapCallback, pose_qos)
        self.costmapSub = self.create_subscription(OccupancyGrid, '/map', self.occupancyGridCallback, 10)
        self.costmap = None

        self.pose = self.create_publisher(PoseStamped,'goal_pose',10)

        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)

        self.get_logger().info('Running Waypoint Test')

        self.tree = False

    #     self.nav = self.create_subscription(BasicNavigator, 'nav', self.get_result,  10)

    # def get_result(self, msg):
    #     t = BasicNavigator()
    #     result = t.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         print('Goal succeeded!')
    #         return 1
    #     elif result == TaskResult.CANCELED:
    #         print('Goal was canceled!')
    #         return -1
    #     elif result == TaskResult.FAILED:
    #         print('Goal failed!')
    #         return 0

    def bt_log_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
           # print(str(event.node_name)+str(event.current_status))
            if (event.node_name == 'NavigateRecovery' and \
            event.current_status == 'IDLE'):
                self.tree = True
                self.moveToFrontiers()
            else:
                self.tree = False
                

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)

    def is_close_to_waypoint(self, position, waypoint, tolerance):
    # Check if the current position is close to the waypoint within the specified tolerance
        distance = math.sqrt((position.x - waypoint.pose.position.x)**2 + (position.y - waypoint.pose.position.y)**2)
        return distance < tolerance

    def moveToFrontiers(self):
      
        if self.tree: 

            # # recovery = RecoveryStrategy()
            # # print(self.get_result())
            # # if self.get_result() == -1 or self.get_result() == 0:
            # #     print ("recovery")
            # #     recovery.run_recovery()

            # Get the current robot's position
            current_position = self.currentPose.position

            # Check if the current waypoint is already set and if it's close to the current position   

            frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger())
            
            #frontiers = [x for x in frontier if x not in self.visitedf]

            if len(frontiers) == 0:
                self.info_msg('No More Frontiers')
                return

            location = None
            largestDist = 0
            largestDist2 = 0
            all_loc = []


            for f in frontiers:
                dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))
                all_loc.append(dist)
                
            location = [frontiers[0]]    

            self.info_msg(f'World points {location}')
            self.setWaypoints(location)

            if self.waypoints and self.is_close_to_waypoint(current_position, self.waypoints[0], tolerance=1):
                self.info_msg('Already at or close to the current waypoint')
                index = all_loc.index(min(all_loc))
                all_loc.pop(index)
                frontiers.pop(index)
                counter = -1
                for loc in all_loc:
                    counter +=1  
                    if dist > largestDist2:
                        location = [frontiers[counter]]
                        self.info_msg('finding new waypoint...')
                self.info_msg('setting new waypoint')
                self.setWaypoints(location)        

            self.visitedf.append(location[0])

            self.info_msg('Sending goal request...')
    
            self.pose.publish(self.waypoints[0])
            self.waypoint_counter += 1

            self.info_msg('Goal accepted')
        

    def costmapCallback(self, msg):
        self.costmap = Costmap2d(msg)

        unknowns = 0
        for x in range(0, self.costmap.getSizeX()):
            for y in range(0, self.costmap.getSizeY()):
                if self.costmap.getCost(x, y) == 255:
                    unknowns = unknowns + 1
        self.get_logger().info(f'Unknowns {unknowns}')
        self.get_logger().info(f'Got Costmap {len(getFrontier(None, self.costmap, self.get_logger()))}')

    def dumpCostmap(self):
        costmapReq = GetCostmap.Request()
        self.get_logger().info('Requesting Costmap')
        costmap = self.costmapClient.call(costmapReq)
        self.get_logger().info(f'costmap resolution {costmap.specs.resolution}')

    def setInitialPose(self, pose):
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

    def poseCallback(self, msg):
      #  self.info_msg('Received amcl_pose')
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True
        
        
    
        

    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
           # msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)
        



    def publishInitialPose(self):
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

    wps = [[-0.52, -0.54], [0.58, -0.55], [0.58, 0.52]]
    starting_pose = [-2.0, -0.5]

    # wps = [[-20.52, -20.54], [20.58, -20.55], [20.58, 20.52]]
    # starting_pose = [-2.0, -2.0]

    test = WaypointFollowerTest()
    # test.dumpCostmap()
    test.setWaypoints(wps)

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

    rclpy.spin(test)

if __name__ == '__main__':
    main()