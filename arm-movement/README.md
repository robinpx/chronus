An inverse and forward kinematics generator for a 6 DOF robot arm in C++. Built with existing libraries and tools of the robot and utilizing ROS.
Stores the joint degrees into a text file to be used in a Jupiter Notebook that controls the robot arm.

## Getting Started

#### Running code as is

```
source ~/dofbot_ws/devel/setup.bash
roslaunch dofbot_config demo.launch
rosrun dofbot_moveit dofbot_kinematics
```

To run altered code, namely in the main function, please clean up and rebuild the workspace with `catkin_make`

```
catkin_make clean
catkin_make
```
