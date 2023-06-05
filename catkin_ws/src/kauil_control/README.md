# kauil_control
Implementation of the nodes required to establish control on Kauil's movement.

## Index
- [kauil\_control](#kauil_control)
  - [Index](#index)
  - [Files](#files)
  - [Requierements](#requierements)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/launch`
    - [control.launch](./launch/control.launch)
      - Launches a node to manage the `/cmd_velocity` topic to move Kauil and another to read the information from Kail's encoders and publish them in `/wr` and `/wl`.
- `/src`
  - [serial_reader.py](./src/serial_reader.py)
    - Read the rpm of each wheel transmitted by the Arduino and translate them to angular velocities. Then publish them to `/wr` and `/wl`.
  - [twist_listener.py](./src/twist_listener.py)
    - Reads a `cmd_vel` message and translates it to two counters' duty cycles so that Kauil moves.

## Requierements
- `/launch/*` and `/src/*`
  - Battery connected to Kauil's computer.
  - Battery connected to Kauil's motors.

Note: all Arduinos already have access granted usb ports. If access is required please follow the next [steps](#further-information). Also, sometimes, when reconnecting the Arduinos, their id is changed, thus it is deeply suggested to connect one by one and check their assigned ports and, most importantly, revise the ports assigned in the code. These must coincide.

## Further information
- Grant access to Arduino's usb port
  - With arduino connected, in a terminal run ` sudo su`
  - Type the password for Kauil.
  - Run `cd /dev`
  - Run `ls`
  - Note the port of the Arduino. It should be ttyACM* or ttyUSB*
    - If it is not ttyUSB0 correct in `/src/twist_listener.py`
  - Run `chown kauilpc tty*` make shure to change tty* to the port you noted before.
  - `exit`

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)