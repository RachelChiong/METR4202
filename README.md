# METR4202 :

## Resources
- Inpsired greatly by this repo: https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot/tree/main
- And: https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
- Unexplored area detection: http://wiki.ros.org/explore_lite

## Requirements
- nav2_simple_commander

These can be installed using
```
sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d
```

## Build the package
```
cd ~/turtlebot3_ws/src
git clone <link>
cd ~/turtlebot3_ws/
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
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True slam_params_file:=config/slamparams.yaml
```
## Run the module
```
ros2 run fronTEAR_commander fronTEAR_commander
```


