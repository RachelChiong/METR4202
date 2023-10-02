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
from geometry_msgs.msg import PoseStamped

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
        self.costmap = self.create_subscription(
                            Costmap,
                            "costmap",
                            self.costmap_callback,
                            10)
        self.costmap

        # Incremental updates on costmap
        self.costmap_updates = self.create_subscription(
                                    OccupancyGridUpdate,
                                    "costmap_updates",
                                    self.costmap_updates_callback,
                                    10)
        self.costmap_updates

        # Gets Robot's current position
        self.odom = self.create_subscription(
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

        # TODO: Add init code here...


    def get_result(self):
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')


    def get_frontier_poses(self):
        """
        Local costmap is 60 x 60 grid of 0.05 resolution tiles
        Skipping every 10
        """
        frontier = [(10, 10), (10, 30), (10, 50), (30, 10), (30, 50), (50, 10), (50, 30), (50, 50)]
        cm = self.nav.getLocalCostmap()
        gm = self.nav.getGlobalCostmap()
        print(f"Global costmap resolution: {gm.metadata.resolution}, x: {gm.metadata.size_x}, y: {gm.metadata.size_y} ")
        # print(self.nav.getGlobalCostmap().resolution, )
        data = cm.data
        queue = PriorityQueue()

        for el in frontier:
            index = el[0] * 60 + el[1]
            queue.put((data[index], el))
            print(f"Cost: {data[index]}, {el}")

        best_move = queue.get()

        # Wait until get a low value unknown space
        while best_move[0] < 40:
            best_move = queue.get()


        print(f"Best move: {best_move}")
        x = (30 - best_move[1][0]) * 0.05
        y = (30 - best_move[1][1]) * 0.05
        print(f"New waypoint: ({x}, {y}) ")

        with open("waypoints.txt", "a") as file:
            file.write(f"New waypoint: {best_move}\n")

        # Output this new waypoint
        return self.initialise_pose(x, y, 0, 0, 0)


    def get_frontier(pose: PoseStamped, costmap: Costmap, logger: BehaviorTreeLog) -> list:

        # points that have been enqueued by the outer BFS
        MAP_OPEN_LIST = 1

        # points that have been dequeued by the outer BFS
        MAP_CLOSE_LIST = 2

        # points that have been enqueued by the inner BFS
        FRONTIER_OPEN_LIST = 3

        # points that have been dequeued by the outer BFS
        FRONTIER_CLOSE_LIST = 4

        queue_m = SimpleQueue()
        queue_m.put(pose)
        poses = {}
        poses[pose] = MAP_OPEN_LIST

        while queue_m is not None:
            p = queue_m.get()

            if poses[pose] == MAP_CLOSE_LIST:
                continue

        print(costmap.data)


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


    def odom_callback(self, msg: Odometry):
        # Only output odom value when bot is stationary
        if self.bt_status != 'IDLE':
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x = position.x
        y = position.y
        z = position.z

        roll, pitch, yaw = quaternion_to_euler(orientation)

        print(f"Robot Position (x, y, z): ({x}, {y}, {z})")
        print(f"Robot Orientation (roll, pitch, yaw): ({roll}, {pitch}, {yaw})")
        with open("waypoints.txt", "a") as file:
            file.write(f"Robot Position (x, y, z): ({x}, {y}, {z})\n")
        return


    def bt_log_callback(self, msg: BehaviorTreeLog):
        """
        Behaviour Tree Log Callback

        Whenever an action is completed, this is called to complete the next
        action.
        """
        for event in msg.event_log:
           self.bt_status = event.current_status

           if event.node_name == 'NavigateRecovery' and \
               event.current_status == 'IDLE':
                # Get next node to explore and send robot there
                new_pose = self.get_frontier_poses()
                self.send_goal(new_pose)
        return

    def send_goal(self, goal: PoseStamped):
        self.goal_counter += 1
        self.publisher_.publish(goal)
        # if self.goal_counter % 2:
        #     self.publisher_.publish(self.goal1)
        #     # self.nav.goToPose(self.goal2)
        # else:
        #     self.publisher_.publish(self.goal2)
        #     # self.nav.goToPose(self.goal1)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
        self.get_result()

    def costmap_callback(self, msg: Costmap):
        print("Costmap: ", msg.data)
        return

    def costmap_updates_callback(self):
        print("costmap updates callback")
        return


def main(args=None):
    rclpy.init(args=args)
    fronTEAR_commander = FronTEARCommander()
    rclpy.spin(fronTEAR_commander)
    print("Running FronTEAR Commander...")


if __name__ == '__main__':
    main()