"""fronTEAR_commander.py

Main file which compiles the frontier exploration modules to be deployed as a
ROS2 package.

(Note: adapted from the waypoint_cycler.py implementation, so some artifacts
       may remain from that.)
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped

class FronTEARCommander(Node):
    def __init__(self):
        super().__init__('fronTEAR_commander')
        # TODO: Add init code here...


def main(args=None):
    rclpy.init(args=args)
    fronTEAR_cmder = FronTEARCommander()
    rclpy.spin(fronTEAR_cmder)
    print("Running FronTEAR Commander...")

if __name__ == '__main__':
    main()