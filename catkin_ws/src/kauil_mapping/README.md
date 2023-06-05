# kauil_mapping
Generate a map from `scan` readings and `odom`.

## Index
- [kauil\_mapping](#kauil_mapping)
  - [Index](#index)
  - [Files](#files)
  - [Requierements](#requierements)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/src`
  - [laser_calibrator](./src/laser_calibrator.py)
    - Node used to check the LiDAR angles. 
    - <mark>Note</mark>: Hokuyo's 0° is right in front of itself, with 120° of vision to its left and to its right. Going positive angles to de left and negative to the right. The LiDAR scans anti-clockwise.
  - [mapping.py](./src/mapping.py)
    - Create a map based on `scan` and `odom`.
    - The map can be visualized in Rviz directly from the topic with a map visualizer.


## Requierements
- [laser_calibrator.py](./src/laser_calibrator.py) and [mapping.py](./src/mapping.py)
  - Hokuyo connected and comminicating. [How to](#further-information)
- [mapping.py](./src/mapping.py)
  - A node publishing `/odom`

## Further information
- How to connect Hokuyo
  - Connect Hokuyo to power and by usb to a computer
  - Run `sudo su`
    - Type in the password
  - Go to `ls /dev`
    - Note the port of Hokuyo. Usualy ttyUSB*
  - Run `roscore`
  - From this workspace run `rosrun urg_node getID /dev/ttyUSB*`. Change ttyUSB* to the port of Hokuyo you noted before.
    - A message with an ID will be printed.
    - Note the ID
    - <mark>Note</mark>: Kauil Hokuyo's id by default is H1316884.
  - Then run `rosrun urg_node urg_node PORT_ID` for **PORT_ID** use the ID you noted before.
    - A topic `/scan` should be created.

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)