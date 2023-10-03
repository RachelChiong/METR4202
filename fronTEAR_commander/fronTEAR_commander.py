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

        # Store explored areas
        self.explored_waypoints = set()
        self.failed_waypoints = set()

        # Initialised properly in odometry callback
        self.current_position = 0
        self.current_orientation = 0
        self.is_complete = False
        self.pathToPose = 0
        self.same_position_count = 0

        # TODO: Add init code here...


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
        frontier = [(10, 10), (10, 30), (10, 50), (30, 10), (30, 50), (50, 10), (50, 30), (50, 50)]


        cm = self.nav.getLocalCostmap()
        gm = self.nav.getGlobalCostmap()
        print(f"Local costmap resolution: {cm.metadata.resolution}, x: {cm.metadata.size_x}, y: {cm.metadata.size_y} ")
        print(f"Global costmap resolution: {gm.metadata.resolution}, x: {gm.metadata.size_x}, y: {gm.metadata.size_y} ")

        data = cm.data
        queue = PriorityQueue()

        # Get all possible frontier points
        f = [(x, y) for x in (5, cm.metadata.size_x - 5) for y in range(5, cm.metadata.size_y, 5)]
        f += [(x, y) for x in range(5, cm.metadata.size_x, 5) for y in range (5, cm.metadata.size_y - 5)]

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
        while not waypoints.empty():
            best_move = waypoints.get()
            if best_move[0] < 50:
                continue

            # 60 x 60 grid so (30, 30) is centre
            x_diff = (30 - best_move[1][0]) * 0.05
            y_diff = (30 - best_move[1][1]) * 0.05

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


    def odom_callback(self, msg: Odometry):

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
            new_pose = self.get_best_waypoint()
            self.send_goal(new_pose)
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
                new_pose = self.get_best_waypoint()
                print(f"Sending pose {new_pose}")
                self.send_goal(new_pose)
        return

    def send_goal(self, goal: PoseStamped):
        self.goal_counter += 1
        self.publisher_.publish(goal)

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
    rclpy.spin(fronTEAR_commander)
    print("Running FronTEAR Commander...")


if __name__ == '__main__':
    main()