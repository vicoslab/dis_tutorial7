# Tutorial 7: Adding a robot arm and a camera to the Turtlebot4

#### Development of Inteligent Systems, 2024

Currently there is a static camera added to the robot. It is looking top down, to be used for the parking procedure.

## Install packages

For many different tasks, segmenting the ground plane, or finding other dominant planes in a point cloud is importaint. This is implemented in the `planes.cpp` node. After building the package, you can run it with:
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control
```

## Start robot with additional camera

You can start the simulatation with:

```
ros2 launch dis_tutorial7 sim_turtlebot_nav.launch.py
```

One the simulation is running, you can notice there is an additional camera in front of the robot. This camera can be accessed the same way the `oakd` camera is accessed (the same topics are available), but the prefix is `top_camera`.