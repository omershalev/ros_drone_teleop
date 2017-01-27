# ros_drone_teleop
ROS package for teleoperation of aerial vehicles

Prerequisites
---
Depending on the model, you will need to have the relevant ROS messages installed. For instance (AR-Drone on Indigo):
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
For generic drone:
```
roslaunch ros_drone_teleop generic.launch
```
