# kauil_joy
Transform `joy` messages to the requiered messages for various topics.

## Index
- [kauil\_joy](#kauil_joy)
  - [Index](#index)
  - [Files](#files)
  - [Requirements](#requirements)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/src`
  - joy_to_cmd_vel.py
    - Read `joy` message and transform it to a `cmd_vel` message to move Kauil.
  - joy_to_cmd_l_cam.py
    - Read `joy` message and transform it to a `/left_camera/cmd` message to move the left camera of Kauil.
  - joy_to_cmd_r_cam.py
    - Read `joy` message and transform it to a `/right_camera/cmd` message to move the right camera of Kauil.
- `/launch`
  - joy.launch
    - Launch all nodes from `/src`

## Requirements
- `/src/*`
  - Controller connected. 
  - Permissions for controller. [How to](#further-information).

- `/src/joy_to_cmd_*_cam.py`
  - Kauil's router connected.
  - Cameras powered and connected to router.
  - Computer, personal or Kauil's, connected to Kauil Neta

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