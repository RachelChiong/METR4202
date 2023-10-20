# METR4202 :

## Resources
- The 'OccupancyGrid2d' class is from the following repo: https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
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
## Open the world and SLAM (simulation)
```
ros2 launch turtlebot3_gazebo [WORLD_NAME].py
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True
```
## Open the world and SLAM (hardware) 
```
ros2 launch turtlebot3_gazebo [WORLD_NAME].py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False
ros2 launch slam_toolbox online_async_launch.py
```
## Run the module
```
ros2 run fronTEAR_commander fronTEAR_commander
```


