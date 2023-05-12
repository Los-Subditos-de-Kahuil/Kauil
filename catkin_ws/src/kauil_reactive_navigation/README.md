## kauil_reactive_navigation
Reactive navigation algorithms for Kauil

## Index
- [kauil\_reactive\_navigation](#kauil_reactive_navigation)
- [Index](#index)
- [Files](#files)
- [Requierements](#requierements)
- [Further information](#further-information)
- [Authors](#authors)

## Files
- `/src`
  - bug0.py
    - Implementation fo bug0 algorithm

## Requierements
- `/src/bug0.py`
  - Hokuyo connected and publishing. [How to](#further-information)
  - Odom publisher. [How to](#further-information)

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

- Stablish a odom publisher
  - No odom publisher developed yet. Srry :(

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)