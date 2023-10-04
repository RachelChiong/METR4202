"""fronTEAR_commander.py

Main file which compiles the frontier exploration modules to be deployed as a
ROS2 package.

(Note: adapted from the waypoint_cycler.py implementation, so some artifacts
       may remain from that.)
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import *
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes, GetCostmap

# External packages (requirements)
from nav2_simple_commander.robot_navigator import *
from builtin_interfaces.msg import Duration
import tf_transformations as tft
import numpy as np
import math
from queue import *

# Custom packages
import fronTEAR_commander.nav2_waypoints as nav2_waypoints

OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5

class Costmap2d():
    """
    @reference: https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
    """
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
    """
    @reference: https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
    """
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
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierPoint():
    """
    Store the frontier coordinate an its classification. Used in FrontierCache.
    """
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y


class FrontierCache():
    """
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
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose: PoseStamped, costmap: OccupancyGrid2d):
    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

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
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


def getNeighbors(point, costmap, fCache) -> list:
    """
    Get adjacent points
    """
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point: FrontierPoint, costmap: Costmap, fCache: FrontierCache) -> bool:
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


def quaternion_to_euler(quaternion):
    import math
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class FronTEARCommander(Node):
    def __init__(self):
        super().__init__('fronTEAR_commander')

        ### Subscriptions ###
        self.subscription = self.create_subscription(
                                BehaviorTreeLog,
                                'behavior_tree_log',
                                self.bt_log_callback,
                                10)
        self.subscription

        # Map which will be used for exploration planning
        # Higher cost - obstacles/walls, lower cost - free space
        self.costmap_sub = self.create_subscription(
                            Costmap,
                            "costmap",
                            self.costmap_callback,
                            10)
        self.costmap_sub

        # Incremental updates on costmap
        self.costmap_updates_sub = self.create_subscription(
                                    OccupancyGridUpdate,
                                    "costmap_updates",
                                    self.costmap_updates_callback,
                                    10)
        self.costmap_updates_sub

        # Gets Robot's current position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        ### Publishers ###
        # Visualization of frontiers considered by exploring algorithm
        self.frontier = self.create_publisher(
                            MarkerArray,
                            "frontier",
                            10)

        # Publish waypoints
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)


        self.nav = BasicNavigator()
        self.nav.clearAllCostmaps()

        self.initial_pose = self.initialise_pose(-1.0, -0.5, 0.0, 0.0, 0.0)
        self.nav.setInitialPose(self.initial_pose)

        self.goal_counter = 0
        self.bt_status = 'IDLE'

        self.goal1 = self.initialise_pose(1.5, -1.0, 0.0, 0.0, 1.57)
        self.goal2 = self.initialise_pose(-1.5, 1.0, 0.0, 0.0, -1.57)
        self.nav.goToPose(self.initial_pose)

        # Store explored areas
        self.explored_waypoints = set()
        self.failed_waypoints = set()

        # Initialised properly in odometry callback
        self.current_position = 0
        self.current_orientation = 0
        self.is_complete = False
        self.pathToPose = 0
        self.same_position_count = 0

        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.lastWaypoint = None

        self.action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)

        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        self.costmapSub = self.create_subscription(OccupancyGrid(), '/map', self.occupancyGridCallback, 10)
        self.costmap = None
        # TODO: Add init code here...

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)
        print(f"Costmap size: {self.costmap.getSize()}, x {self.costmap.getSizeX()}, y {self.costmap.getSizeY()}")

    def get_result(self):
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    def get_frontier_poses(self) -> PriorityQueue:
        """
        Local costmap is 60 x 60 grid of 0.05 resolution tiles.\n
        Identifies 8 spaces that the robot can go to and determines their costs
        (lowest cost has most free space, etc.)

        @Returns: PriorityQueue with best move first
        """
        cm = self.nav.getLocalCostmap()
        gm = self.nav.getGlobalCostmap()
        print(f"Local costmap resolution: {cm.metadata.resolution}, x: {cm.metadata.size_x}, y: {cm.metadata.size_y} ")
        print(f"Global costmap resolution: {gm.metadata.resolution}, x: {gm.metadata.size_x}, y: {gm.metadata.size_y} ")

        data = cm.data
        queue = PriorityQueue()

        # Get all possible frontier points
        f = [(x, y) for x in range(5, cm.metadata.size_y - 5) for y in range(5, cm.metadata.size_x, 5)]
        f += [(x, y) for x in range(5, cm.metadata.size_y, 5) for y in range (5, cm.metadata.size_x - 5)]

        for el in f:
            # Get new centre point of the robot index
            index = el[0] * cm.metadata.size_y + el[1]

            # Total cost
            total = 0
            count = 0

            # Aggregate the cost of moving the robot to the new location,
            # A 5 x 5 square around the robot since the radius of the bot is 0.22,
            # and each square represents 0.05
            for x in range(-5, 5):
                for y in range(-5, 5):
                    new_index = index + (x * cm.metadata.size_y) + y
                    if new_index >= cm.metadata.size_y * cm.metadata.size_x:
                        total += 255
                    else:
                        total += data[new_index]
                    count += 1

            # Calculate the average cost of moving the robot to that location
            avg_cost = total / count

            queue.put((avg_cost, el))
            # print(f"Cost: single point - {data[index]}, average region - {avg_cost}, {el}")

        return queue


    def get_best_waypoint(self) -> PoseStamped:
        """
        Get the best waypoint from the priority queue of frontier poses

        Checks for:
        - too small value (most likely outside of the world)
        - location has already been searched

        @Returns the best
        """
        waypoints = self.get_frontier_poses()
        if self.currentPose != None and self.costmap != None:
            frontiers = getFrontier(self.currentPose, self.costmap)
            print("Frontier: ", frontiers)
            location = None
            largestDist = 0

            for f in frontiers:
                dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))
                print(self.currentPose.position.x, self.currentPose.position.y)
                if (round(f[0], 1),round(f[1],1)) in self.explored_waypoints:
                    continue
                if  dist > largestDist:
                    largestDist = dist
                    location = f
            pose = PoseStamped()
            pose.pose.position.x = location[0]
            pose.pose.position.y = location[1]
            pose.pose.orientation.w = 1.0

            self.explored_waypoints.add((round(location[0], 1), round(location[1], 1)))
            return pose


        while not waypoints.empty():
            best_move = waypoints.get()
            if best_move[0] < 50:
                continue

            # 60 x 60 grid so (30, 30) is centre
            x_diff = (best_move[1][0]) * 0.05
            y_diff = (best_move[1][1]) * 0.05

            # Round to single decimal place of precision to prevent repeat
            # movements
            x = round(self.current_position.x + x_diff, 1)
            y = round(self.current_position.y + y_diff, 1)
            print(f"Current: ({self.current_position.x}, {self.current_position.y})")
            print(f"New: ({x}, {y})")

            # Move on to next best waypoint if already explored
            threshold = [(tx, ty) for tx in (x - 0.1, x, x + 0.1) for ty in (y - 0.1, y, y + 0.1)]
            c = 0
            for el in threshold:
                if el in self.explored_waypoints:
                    self.explored_waypoints.add((x, y))
                    print("Explored waypoints: ", self.explored_waypoints)
                    c += 1
                    continue

            if c != 0:
                print(f"Count: {c}")
                continue

            self.explored_waypoints.add((x, y))

            # Don't need to determine roll, pitch, yaw because the robot's A*
            # Will figure that out itself
            return self.initialise_pose(x_diff, y_diff, 0, 0, 0)

        # Only get here if all waypoints have been pursued and explored
        # Return to original position
        self.is_complete = True
        print("Queue is empty so returning to original pose")
        return self.initial_pose


    def initialise_pose(self, x_pos: float, y_pos: float, roll: float, pitch: float, yaw: float) -> PoseStamped:
        """
        Initialises the Pose stamp with given parameters.
        """
        # you can use the map displayed in RViz to estimate the position (the grid is 1m x 1m)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()

        goal.pose.position.x = x_pos
        goal.pose.position.y = y_pos

        qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        return goal

    def setInitialPose(self, pose: list):
        """
        Params
        Pose (x, y): the initial pose to set to give x, y values
        """
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        time.sleep(5)

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def odom_callback(self, msg: Odometry):
        self.currentPose = msg.pose.pose

        new_position = msg.pose.pose.position
        new_orientation = msg.pose.pose.orientation

        isNew = False

        if self.current_position == 0:
            self.current_position = new_position
            self.current_orientation = new_orientation

        # Only output odom value when bot is stationary
        if self.bt_status != 'IDLE':
            return
        if abs(abs(new_position.x) - abs(self.current_position.x)) > 0.1 and \
            abs(abs(new_position.y) - abs(self.current_position.y)) > 0.1:
            isNew = True
            self.same_position_count = 0
        #     rotate_pose = self.initialise_pose(0.0, 0.0, 0.0, 0.0, 1.0)
        #     print("Sent new rotation pose")
        #     self.send_goal(rotate_pose)
        #     if self.nav.isTaskComplete():
        #         new_pose = self.get_best_waypoint()
        #         self.send_goal(new_pose)
        else:
            self.same_position_count += 1


        self.current_position = new_position
        self.current_orientation = new_orientation

        if not isNew and self.same_position_count > 4:
            #new_pose = self.get_best_waypoint()
            #self.send_goal(new_pose)
            self.same_position_count = 0
            return

        x = self.current_position.x
        y = self.current_position.y
        z = self.current_position.z

        self.explored_waypoints.add((round(x, 1), round(y, 1)))

        roll, pitch, yaw = quaternion_to_euler(self.current_orientation)

        print(f"Robot Position (x, y, z): ({round(x, 3)}, {round(y, 3)}, {round(z, 3)})")
        print(f"Robot Orientation (roll, pitch, yaw): ({round(roll, 3)}, {round(pitch, 3)}, {round(yaw, 3)})")
        return


    def bt_log_callback(self, msg: BehaviorTreeLog):
        """
        Behaviour Tree Log Callback

        Whenever an action is completed, this is called to complete the next
        action.
        """
        for event in msg.event_log:
           self.bt_status = event.current_status
           # Wait for first odom reading

           if event.node_name == 'NavigateRecovery' and \
               event.current_status == 'IDLE':
                # Get next node to explore and send robot there
                if self.current_position == 0:
                    pass
                # # new_pose = self.get_best_waypoint()
                # # self.send_goal(new_pose)
        return

    def send_goal(self, goal: PoseStamped):
        self.goal_counter += 1
        print(f"Publishing ({goal.pose.position.x}, {goal.pose.position.y})")
        self.publisher_.publish(goal)
        # self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
        self.get_result()
        # if self.nav.getResult() == TaskResult.FAILED:
        #     rotate_pose = self.initialise_pose(0.0, 0.0, 0.0, 0.0, 1.0)
        #     print("Sent new rotation pose")
        #     self.publisher_.publish(rotate_pose)

        if self.is_complete:
            print("Mapping is complete.")

    def costmap_callback(self, msg: Costmap):
        print("Costmap: ", msg.data)
        return

    def costmap_updates_callback(self):
        print("costmap updates callback")
        return


def main(args=None):
    rclpy.init(args=args)
    fronTEAR_commander = FronTEARCommander()
    fronTEAR_commander.setInitialPose([-2.0, -0.5])
    rclpy.spin_once(fronTEAR_commander, timeout_sec=1.0)

    while fronTEAR_commander.costmap == None:
        rclpy.spin_once(fronTEAR_commander, timeout_sec=1.0)

    while True:
        pose = fronTEAR_commander.get_best_waypoint()
        fronTEAR_commander.nav.goToPose(pose)
        while not fronTEAR_commander.nav.isTaskComplete():
            pass
        print(fronTEAR_commander.nav.getResult())
        #frontiers = getFrontier(fronTEAR_commander.currentPose, fronTEAR_commander.map)
    # pose = fronTEAR_commander.get_best_waypoint()
    # while pose is not None:
    #     fronTEAR_commander.nav.goToPose(pose)
    #     pose = fronTEAR_commander.get_best_waypoint()
    rclpy.spin(fronTEAR_commander)
    print("Running FronTEAR Commander...")


if __name__ == '__main__':
    main()