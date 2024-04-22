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

One the simulation is running, you can notice there is an additional camera in front of the robot. This camera can be accessed the same way the `oakd` camera is accessed (the same topics are available), but the prefix is `top_camera`. The arm is controlled by the `arm_controller` controller, which should be started after the `diffdrive_controller`. You can verify that the arm controller is working if, after unpasuing the simulation, you see a printout in the terminal `[spawner_arm_controller]: Configured and activated arm_controller`.

## Setting the arm position

The node `arm_mover_actions.py` is a node that you can use to set the arm positon in a simple way. The node is communicating with the `arm_controller` through an action interface, just like we set a goal for the robot in the Nav2 stack (tutorials 3 and 4). You can include this node in some launch file, or start it with `ros2 run`:
```
ros2 run dis_tutorial7 arm_mover_actions.py
```

This node listens for a command of type `String` on the `/arm_command` topic. Currently, there are four approximate positions that are hard-coded:
```
self.arm_poses = {'look_for_parking':[0.,0.4,1.5,1.2],
                  'look_for_qr':[0.,0.6,0.5,2.0],
                  'garage':[0.,-0.45,2.8,-0.8],
                  'up':[0.,0.,0.,0.],
                  'manual':None}
```

This positions are approximate positions that might be suitable for different tasks. The "garage" posiotion is for packing the arm so we do not hit something while driving the robot. The "up" position just sets all the joints to 0. The "look_for_parking" is a position that might be suitable for parking the robot. The "look_for_qr" position is a position that might be suitable for reading the QR code on top of the cyllinder. You are highly encouraged to modify these postions as you see fit! The last key in the dictionary - "manual" is there for debugging purposes, so that you are able to quicly test a configuration of the arm.

The position of the arm should be set from your code, and you can test the positions by using the 'ros2 topic' interface:
```
ros2 topic pub --once /arm_command std_msgs/msg/String "{data: garage}"
```
or
```
ros2 topic pub --once /arm_command std_msgs/msg/String "{data: look_for_parking}"
```
or
```
ros2 topic pub --once /arm_command std_msgs/msg/String "{data: look_for_qr}"
```

Be careful when using the "manual" keyword, as you can easily crash the node with the wrong format. To set a manual position send a `String` message in the format 'manual:[pos1, pos2, pos3, pos3]' where pos1-4 are floats (the numbers should contain a . ). For example:
```
ros2 topic pub --once /arm_command std_msgs/msg/String "{data: 'manual:[0.,0.6,0.5,2.0]'}"
```