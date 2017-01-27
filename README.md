# ros_drone_teleop
ROS package teleoperation of drones

Prerequisites
---
Depends on the model type, need to have the relevant ROS messages installed
For instance:
```
sudo apt-get install ros-indigo-ardrone-autonomy
```

Installation
---
1. Create a catkin workspace or use an exiting one
2. ``` cd <your catkin workspace>/src ```
3. ``` git clone https://github.com/omershalev/ros_drone_teleop ```
4. Build the workspace (```catkin_make```) and source the setup.bash script

Launch
---
For AR-Drone:
```
roslaunch ros_drone_teleop ardrone_keyboard_teleop.launch
```
For Bebop:
```
roslaunch ros_drone_teleop bebop_keyboard_teleop.launch
```
