An inverse and forward kinematics generator for a 6 DOF robot arm in C++. Built with existing libraries and tools of the robot and utilizing ROS.
Stores the joint degrees into a text file to be used in a Jupiter Notebook that controls the robot arm.

## Getting Started

#### Run current demo 

```
source ~/dofbot_ws/devel/setup.bash
roslaunch dofbot_config demo.launch
rosrun dofbot_moveit dofbot_kinematics
```
### Run altered code 

To run altered code, namely in the main function, please clean up and rebuild the workspace with `catkin_make`

```
catkin_make clean
catkin_make
```

## To do
* Fasten handle for singing bowl
* Grip
* Test rake at sideways angles

## Notes
Coordinates of the singing bowl in cm, given the arm is drawing the circle at 45 degrees on servo 1. Note that the red bar is the X axis and the green bar is the Y axis.
```
X coordinates(cm)： 15.026564	Y coordinates (cm)： 15.112140	Z coordinates (cm)： 9.625512
```

![Ros Arm at 45 degrees](https://raw.githubusercontent.com/robinpx/chronus/main/arm-movement/rosarm45.png)

