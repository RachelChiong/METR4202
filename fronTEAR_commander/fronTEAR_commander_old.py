"""fronTEAR_commander.py

Main file which compiles the frontier exploration modules to be deployed as a
ROS2 package.

(Note: adapted from the waypoint_cycler.py implementation, so some artifacts
       may remain from that.)
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

# Custom packages
import detect_unexplored

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
        self.costmap = self.create_subscription(
                            OccupancyGrid,
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

        ### Publishers ###
        # Visualization of frontiers considered by exploring algorithm
        self.frontier = self.create_publisher(
                            MarkerArray,
                            "frontier",
                            10)

        # TODO: Add init code here...

    def costmap_callback(self):
        print("costmap callback")
        return

    def costmap_updates_callback(self):
        print("costmap updates callback")
        return


def main(args=None):
    rclpy.init(args=args)
    fronTEAR_cmder = FronTEARCommander()
    rclpy.spin(fronTEAR_cmder)
    print("Running FronTEAR Commander...")

if __name__ == '__main__':
    main()