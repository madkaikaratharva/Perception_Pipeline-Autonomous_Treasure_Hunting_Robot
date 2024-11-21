# Perception Pipeline for Autonomous Treasure Hunting Robot


NUS ME5400A FINAL PROJECT
> Authors: Atharva Madkaikar


![cover_image](src/media/cover_image.jpeg)


## Overview

The project involved developing and testing a Mini Autonomous Driving Platform, focusing on hardware development, perception, localization, and navigation. To address these core areas, we designed and implemented an Autonomous Treasure Hunting Robot capable of exploring an unknown environment and locating a specified treasure object hidden within.

This repository contains the robot's perception stack, which includes two ROS packages: `perception` and `realsense_ros`. The stack can identify static and dynamic obstacles and locate, recognize, and track the specified treasure object when deployed in an environment.

The stack can also be used as a general purpose perception stack for a wide range of AMRs (Autonomous Mobile Robots).


## Dependencies

* System Requirements:
    * Ubuntu 20.04, JetPack 5.1 (other versions not tested)
    * ROS Noetic
    * Python 3.8 and above
    * CUDA
    * CUDNN
* Required Python Packages:
    * `opencv-python`
    * `numpy`
    * `pyrealsense2`
    * `deep-sort-realtime`
    * `torch`
    * `torchvision`
    * `tkinter`

**Note:** A pre-trained model of YOLO from Ultralytics has been included in this repo for real-time object detection.
          DeepSort framework has been implemented for real-time object tracking. You can use you custom detection and tracking algorithms
          by making the necessary changes in the source code.

To install these pacakges, run
```bash
# Ultralytics YOLO
pip install ultralytics

# DeepSort Tracking
pip install deep-sort-realtime
```
* Required standard ROS packages:
    * `rospy`
    * `rviz`
    * `std_msgs`
    * `geometry_msgs`
    * `sensor_msgs`
    * `vision_msgs`
    * `cv_bridge`
* Hardware requirements:
    * Nvidia Jetson Orin Nano (will work with other versions)
    * Intel RealSense D435, D435i




