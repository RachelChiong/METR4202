# METR4202 :

## Resources
- The 'OccupancyGrid2d' class is from the following repo: https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
- Unexplored area detection: http://wiki.ros.org/explore_lite
- ARUCO Marker detection: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main#readme

## Requirements
- nav2_simple_commander

These can be installed using
```
sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d
```
- ros2_aruco 

pip3 install opencv-contrib-python transforms3d

## Build the package
```
cd ~/turtlebot3_ws/src
git clone <link>
cd ~/turtlebot3_ws/
colcon build --symlink-install 
```
Then, source:
```
source /opt/ros/humble/setup.bash
. source install/setup.bash
```
## Level 0
# Open the world and SLAM (simulation)
```
ros2 launch turtlebot3_gazebo [WORLD_NAME].py
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True slam_params_file:=config/slamparams.yaml
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

## ARUCO Marker Detection 

1. Using the launch file - parameters will be loaded from _aruco\_parameters.yaml_.
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
2. As a single ROS 2 node - you can specify parameter values at startup by adding `--ros-args -p marker_size:=.05`, for example.
```
ros2 run ros2_aruco aruco_node
```
In rviz, add aruco_poses topic to see the position of the aruco marker.

