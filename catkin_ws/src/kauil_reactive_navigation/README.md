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
- `/launch`
  - [bug20.launch](./launch/bug20.launch)
    - Launch bug 20 algorithm and services for goal change.
- `/resources`
  - [goals.txt](./resources/goals.txt)
    - File containing goals to be followed in a round robin matter.
  - `/scripts`
    - [node_server.py](./scripts/node_server.py)
      - Node to keep the service of [getNewGoal](#getNewGoal)
- `/src`
  - [kauil_bug20.py](./src/kauil_bug20.py)
    - Implementation for bug 20 algorithm; algorithm developed by the owners of the repo.
- `/srv`
  - <span id="getNewGoal">[getNewGoal.srv](./srv/getNewGoal.srv)</span>
    - Service to get a new goal.

Note: Only bug20 is updated to work with Kauil's dimensions and the goal changer service. For further development on [bug0.py](./src/bug0.py) and [bug2.py](./src/bug2.py) please use [kauil_bug20.py](./src/kauil_bug20.py) as a template. The same goes for their corresponding launch files.
## Requierements
- [kauil_bug20.py](./src/kauil_bug20.py)
  - Hokuyo connected and publishing. [How to](#further-information)
  - Odom publisher
    - sugestion is to run kalman filter
      - `roslaunch kauil_slam slam.launch`

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
    - Hokuyo's default ID is H1316884
  - Then run `rosrun urg_node urg_node.cpp PORT_ID` for **PORT_ID** use the ID you noted before.
    - A topic `/scan` should be created.

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)