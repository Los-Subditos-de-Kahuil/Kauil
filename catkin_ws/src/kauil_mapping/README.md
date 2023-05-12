# kauil_mapping
Generate a map from `scan` readings and `odom`

## Index
- [kauil\_mapping](#kauil_mapping)
  - [Index](#index)
  - [Files](#files)
  - [Requierements](#requierements)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/src`
  - fake_odom.py
    - Generate a fake odometry based on `cmd_vel`
  - mapping.py
    - Create a map based on `scan` and `odom`
    - As a temporal solution to the lack of communication with the encoders `fake_odom` is used.
    - The map can be visualized in Rviz directly from the topic with a map visualizer.


## Requierements
- `/src/mapping.py`
  - fake_odom.py running
  - Hokuyo connected and comminicating. [How to](#further-information)

## Further information
- How to connect Hokuyo
  - Connect Hokuyo to power and by usb to a computer
  - Run `sudo su`
    - Type in the password
  - Go to `ls /dev`
    - Note the port of Hokuyo. Usualy ttyUSB*
  - Run `roscore`
  - From this workspace run `rosrun urg_node getID.cpp /dev/ttyUSB*`. Change ttyUSB* to the port of Hokuyo you noted before.
    - A message with an ID will be printed.
    - Note the ID
  - Then run `rosrun urg_node urg_node.cpp PORT_ID` for **PORT_ID** use the ID you noted before.
    - A topic `scan` should be created.

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)