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
AR-Drone with keyboard:
```
roslaunch drone_keyboard_teleop ardrone_keyboard_teleop.launch
```
Bebop with keyboard:
```
roslaunch drone_keyboard_teleop bebop_keyboard_teleop.launch
```
Generic drone with keyboard:
```
roslaunch drone_keyboard_teleop generic.launch
```
Bebop with joystick:
```
roslaunch drone_joystick_teleop bebop.launch
```
Generic drone with joystick:
```
roslaunch drone_joystick_teleop generic.launch
```
