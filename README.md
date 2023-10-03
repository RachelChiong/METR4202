# METR4202 :

## Resources
- Inpsired greatly by this repo: https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot/tree/main
- Unexplored area detection: http://wiki.ros.org/explore_lite

## Requirements
- nav2_simple_commander
- tf_transformations

These can be installed using
```
sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d
```

## Build the package
```
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select fronTEAR_commander
```
Then, source:
```
source /opt/ros/humble/setup.bash
. source install/setup.bash
```
## Open the world and SLAM
```
ros2 launch turtlebot3_gazebo [WORLD_NAME].py
ros2 launch slam_toolbox online_async_launch.py
ros2 launch turtlebot3_navigation navigation2.launch.py
```
## Run the module
```
ros2 run fronTEAR_commander fronTEAR_commander
```

Note, this works best on the world map.


