## Getting Started

#### Running code as is

```
source ~/dofbot_ws/devel/setup.bash
roslaunch dofbot_config demo.launch
rosrun dofbot_moveit dofbot_kinematics
```

To run altered code, namely in the main function, please clean up and rebuild the workspace with catkin_make

```
catkin_make clean
catkin_make
```
