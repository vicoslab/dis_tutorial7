# Tutorial 7: Adding a robot arm and a camera to the Turtlebot4

#### Development of Inteligent Systems, 2024

This exercise will show a few examples of how to use the [Point Cloud Library (PCL)](https://pointclouds.org/) and OpenCV to extract information from the RGBD camera. The PCL project contains a large number of [tutorials](https://pcl.readthedocs.io/projects/tutorials/en/master/) demonstrating how to use the library. From the code in this tutorial you can extrapolete how to use the PCL library in ROS2. For out purposes, the tutorials on PointCloud [segmentation](https://pcl.readthedocs.io/projects/tutorials/en/master/#segmentation) are the most relevant. The given examples use the RANSAC algorithm to find planes and cylinders, and extract the inliers. 

## Install packages

For many different tasks, segmenting the ground plane, or finding other dominant planes in a point cloud is importaint. This is implemented in the `planes.cpp` node. After building the package, you can run it with:
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control
```

## Start robot with additional camera
```
ros2 launch dis_tutorial7 sim_turtlebot_nav.launch.py
```