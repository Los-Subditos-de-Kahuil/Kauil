# kauil_joy
Transform `joy` messages to the requiered messages for various topics that control different aspects of Kauil.

## Index
- [kauil\_joy](#kauil_joy)
  - [Index](#index)
  - [Files](#files)
  - [Requirements](#requirements)
  - [Use](#use)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/launch`
  - [joy.launch](./launch/joy.launch)
    - Launch all nodes in these package to enable all features of joy comtrol.
- `/src`
  - [joy_to_cmd_vel.py](./src/joy_to_cmd_vel.py)
    - Read `joy` message and transform it to a `cmd_vel` message to move Kauil.
  - [joy_to_cmd_l_cam.py](./src/joy_to_cmd_l_cam.py)
    - Read `joy` message and transform it to a `/left_camera/cmd` message to move the left camera of Kauil.
  - [joy_to_cmd_r_cam.py](./src/joy_to_cmd_r_cam.py)
    - Read `joy` message and transform it to a `/right_camera/cmd` message to move the right camera of Kauil.

## Requirements
- `/src/*`
  - Controller connected. 
  - Permissions for controller. [How to](#further-information).
  - Kauil's computer connected to a battery.
  - Motors connected to a batery.
  - Router connected to a batery.

- `/src/joy_to_cmd_*_cam.py`
  - Cameras powered and connected to router.
  - Personal computer and Kauil's computer connected to Kauil Neta.

## Use
For this package in specific you need to follow the next steps.
- With everything connect as stated in the [requirments](#requirements) and both computers connected to KauilNeta.
- In your computer in `catkin_ws` 
  - Run `source devel/setup.bash`
  - Then `roslaunch kauil_joy joy.launch`
- In another terminal in `catkin_ws` and after `source devel/setup.bash`
  - Run `roslaunch kauil_vision camera_movement.launch`
- Then open a new terminal and connect with ssh to Kauil
  - `ssh kauilpc@192.168.10.101`
  - Type Kauil's password
- Then connect from Kauil to your ROS master.
  - `export ROS_MASTER_URI=http://192.168.10.100:11311`
  - `export ROS_IP=192.168.10.101`
- Then from the terminal connected to Kauil run
  - `roslaunch kauil_control control.launch`
- If you also want to see the cameras you can run in another terminal from the `catkin_ws` and with `source devel/setup.bash`
  - `rosrun kauil_vision image_publisher`

Note: information regarding *kauil_vision* package and its nodes can be found [here](../kauil_vision/README.md) and information about *kauil_control* [here](../kauil_control/README.md).

## Further information
- Permissions for controller.
  - First check the assigned port by running `ls /dev/input/`, it must be preceded by the letters **js** (for instance *js0*)
  - Once you've identified it, check if it's the right one by running `sudo jstest /dev/input/jsX` (where *X* is the number you identified before)
    - If you do not have `jstest` installed, do so
    - This will display/print some information on your terminal. Move your joysticks and press your buttons to make sure a) it's working and b) it's the right one
  - Afterwards check the permisions it already has, all users must have read-write (*rw*) permissions. Do so by running `ls -l /dev/input/jsX`, after which you'll see something similar to `crw-rw-XX- 1 root ...`
    - If *XX* is *rw*, you're good to go
    - If not, then you must run sudo chmod a+rw `/dev/input/jsX`

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)