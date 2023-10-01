import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations as tft

def to_pose_stamped(x: float, y: float, yaw: float, z: 
                    float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # pose.header.stamp = nav.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


# install dependencies sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations  python3-transforms3d
# this requires gazebo to be running (see programming_with_nav2.md):
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# cd ~/workspace/nav2_ws
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_slam_toolbox.yaml

def main():
    rclpy.init()

    nav = BasicNavigator()

    # set the initial pose
    initial_pose = to_pose_stamped(x=-2.0, y=-0.5, yaw=0)

    # !!! you only need to run it the first time !!! otherwise the initial position will be set wrongly
    nav.setInitialPose(initial_pose)

    # this waits until the message has been received and executed
    nav.waitUntilNav2Active()

    # you can use the map displayed in RViz to estimate the position (the grid is 1m x 1m)
    goal1 = to_pose_stamped(x=-1.0, y=-1.5, yaw=0.0)
    goal2 = to_pose_stamped(x=1.5, y=-1.5, yaw=1.57)
    goal3 = to_pose_stamped(x=1.5, y=1.5, yaw=3.14)
    goal4 = to_pose_stamped(x=-1.5, y=1.5, yaw=-1.57)
    goal5 = to_pose_stamped(x=-2.0, y=-0.0, yaw=-0.785)

    waypoints = [goal1, goal2, goal3, goal4, goal5]

    goal_counter = 0
    while True:
        nav.followWaypoints(waypoints)

        while not nav.isTaskComplete():
            pass
            # print(nav.getFeedback())

        print('Nav result: ', nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()