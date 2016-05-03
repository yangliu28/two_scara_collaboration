# two_scara_collaboration

Simulation of two SCARA robots collaboratively sort items from a conveyor belt with collision avoidance and task distribution.

## Test command
```
roslaunch two_scara_collaboration initialize.launch
```
Gazebo simulation is paused when initialized, in case the arms of the two scara robots appear at same location and bounce away each other. Click start button to start.

## Progress and problems (May 3, 2016)
The scara robots, conveyor belt and cylinder blocks were modeled in urdf.
A spawner node can automatically spawn cylinder blocks and let them slide on conveyor with an initial speed.
A cylinder blocks position publisher node was added to publish the 3-D position of all current cylinders.
A gripper control action server node can keep gripper at desired location, and receiving command on the gripper action.
---


problem with ros_control



