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

from nav2_simple_commander.robot_navigator import *
from builtin_interfaces.msg import Duration
import tf_transformations as tft

# Custom packages
import fronTEAR_commander.nav2_waypoints as nav2_waypoints

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

        self.initial_pose = self.initialise_pose(-1.0, -0.5, 0.0, 0.0, 0.0)
        self.nav.setInitialPose(self.initial_pose)

        self.goal_counter = 0
        self.bt_status = 'IDLE'

        self.goal1 = self.initialise_pose(1.5, -1.0, 0.0, 0.0, 1.57)
        self.goal2 = self.initialise_pose(-1.5, 1.0, 0.0, 0.0, -1.57)
        self.nav.goToPose(self.goal2)

        # TODO: Add init code here...


    def get_result(self):
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')


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
                   self.send_goal()
        return

    def send_goal(self):
        self.goal_counter += 1
        if self.goal_counter % 2:
            self.publisher_.publish(self.goal1)
            # self.nav.goToPose(self.goal2)
        else:
            self.publisher_.publish(self.goal2)
            # self.nav.goToPose(self.goal1)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
        self.get_result()

    def costmap_callback(self):
        print("costmap callback")
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