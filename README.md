# Kauil
Repository for Kauil; a rescue robot.

<br>

<div style="text-align: center">
  <img src="./misc/kauil.jpeg" alt=kauil width="100%">
</div>

## Index 
- [Kauil](#kauil)
  - [Index](#index)
  - [Folders](#folders)
  - [Installation](#installation)
  - [Requirements](#requirements)
  - [Use](#use)
  - [Authors](#authors)
  - [Project status](#project-status)
## Folders
- Arduino
  - Code and tests for Arduinos in Kauil.
- catkin_ws
  - Catkin workspace in Kauil.
- misc
  - Miscellaneous information of the project.

<div align ="right">
<a href="#kauil">Go to top</a>
</div>

## Installation
Download the Arduino files into the Arduinos and clone the repository into Kauil. Then pass the packages into Kauil's `~/catkin_ws/src` and in `~/catkin_ws` run `catkin_make`.

Other packages and Python libraries are required. To install them, follow the next steps:
- `pip install numpy==1.13.3`
- `sudo apt install ros-melodic-ros-numpy`
- `sudo apt install ros-melodic-vision-msgs`
- `sudo apt install ros-melodic-cv-bridge`
  - `cd /opt/ros/melodic/share/cv_bridge/cmake`
  - `sudo gedit cv_bridgeConfig.cmake`
  - The change the line
    - `set(_include_dirs "include;/usr/include;/usr/include/opencv")`
    - to
    - `set(_include_dirs "include;/usr/include;/usr/include/opencv4")`
    - Save the file and exit.
- `sudo apt install ros-melodic-aruco-detect`
- `sudo apt install ros-melodic-image-transport`
- `sudo apt install ros-melodic-compressed-image-transport`
- `sudo apt install ros-melodic-rplidar`

<div align ="right">
<a href="#kauil">Go to top</a>
</div>

## Requirements
- Python 2.7.17
  - rospy
  - numpy==1.13.3
  - opencv==4.2.0
- ROS Melodic
  - ros_numpy
  - cv_bridge
  - vision_msgs
  - aruco_detect
  - image_transport
  - compressed_image_transport

Note: To ensure the satisfaction of the stated requirements be sure to follow the steps declared on [installation](#installation).

<div align ="right">
<a href="#kauil">Go to top</a>
</div>

## Use
1. Connect all batteries to Kauil.
2. Launch the files corresponding to the usage you want to give to kauil. (The packages are described in the `README.md` file inside each).

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)

<div align ="right">
<a href="#kauil">Go to top</a>
</div>

## Project status
The project is finished as of June 2023 and no further development will be made by the current owners of the repository.

<div align ="right">
<a href="#kauil">Go to top</a>
</div>