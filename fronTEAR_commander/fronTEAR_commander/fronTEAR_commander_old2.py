import rclpy
from nav2_simple_commander.robot_navigator import *
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import *
import tf_transformations as tft
import time

# def to_pose_stamped(x: float, y: float, yaw: float, z:
#                     float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> PoseStamped:
#     pose = PoseStamped()
#     pose.header.frame_id = 'map'
#     # pose.header.stamp = nav.get_clock().now().to_msg()

#     pose.pose.position.x = x
#     pose.pose.position.y = y
#     pose.pose.position.z = z

#     qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
#     pose.pose.orientation.x = qx
#     pose.pose.orientation.y = qy
#     pose.pose.orientation.z = qz
#     pose.pose.orientation.w = qw

#     return pose


# install dependencies sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations  python3-transforms3d
# this requires gazebo to be running (see programming_with_nav2.md):
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# cd ~/workspace/nav2_ws
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_slam_toolbox.yaml

# def main():
#     print()
#     rclpy.init()

#     nav = BasicNavigator()

#     # set the initial pose
#     initial_pose = to_pose_stamped(x=-1.0, y=-0.5, yaw=0)

#     # !!! you only need to run it the first time !!! otherwise the initial position will be set wrongly
#     nav.setInitialPose(initial_pose)

#     # this waits until the message has been received and executed
#     nav.waitUntilNav2Active()

#     # you can use the map displayed in RViz to estimate the position (the grid is 1m x 1m)
#     goal1 = to_pose_stamped(x=-0.9, y=-1.4, yaw=0.0)
#     goal2 = to_pose_stamped(x=1.4, y=-1.4, yaw=1.57)
#     goal3 = to_pose_stamped(x=1.4, y=1.4, yaw=3.14)
#     goal4 = to_pose_stamped(x=-1.4, y=1.4, yaw=-1.57)
#     goal5 = to_pose_stamped(x=-1.9, y=-0.0, yaw=-0.785)

#     waypoints = [goal1, goal2, goal3, goal4, goal5]

#     goal_counter = 0
#     while True:
#         nav.followWaypoints(waypoints)

#         while not nav.isTaskComplete():
#             pass
#             # print(nav.getFeedback())

#         print('Nav result: ', nav.getResult())

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

def get_next_goal(x_pos: float, y_pos: float, roll: float, pitch: float, yaw: float) -> PoseStamped:
    # you can use the map displayed in RViz to estimate the position (the grid is 1m x 1m)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()

    goal.pose.position.x = x_pos
    goal.pose.position.y = y_pos

    qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    return goal

def main():
    print("Starting frontear commander...\n")
    rclpy.init()

    global nav
    nav = BasicNavigator()

    # set the initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.position.z = 0.0

    # you can use this to convert Euler to Quat:
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, 0)

    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw

    # !!! you only need to run it the first time !!! otherwise the initial position will be set wrongly
    nav.setInitialPose(initial_pose)

    # this waits until the message has been received and executed
    # nav.waitUntilNav2Active()

    goal_counter = 0

    #goal1 = get_next_goal(1.5, -1.0, 0.0, 0.0, 1.57)
    goal2 = get_next_goal(-1.5, 1.0, 0.0, 0.0, -1.57)

    nav.goToPose(goal2)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # if feedback.navigation_time > Duration(sec=600):
        #     nav.cancelTask()
        #     print("Task too long. Cancelled")

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    # while True:
    #     nav.goToPose(goals[goal_counter % 2])
    #     goal_counter += 1

    #     while not nav.isTaskComplete():
    #         print(nav.getFeedback())
    #         pass
    #     print('Nav result: ', nav.getResult())
    #     break

    rclpy.shutdown()

if __name__ == '__main__':
    main()