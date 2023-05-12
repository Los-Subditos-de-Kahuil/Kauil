# kauil_control
Manages a `cmd_vel` to move Kauil accordingly.

## Index
- [kauil\_control](#kauil_control)
  - [Index](#index)
  - [Files](#files)
  - [Requierements](#requierements)
  - [Further information](#further-information)
  - [Authors](#authors)

## Files
- `/src`
  - twist_listener.py
    - Reads a `cmd_vel` message and translates it to two counters' duty cycles so that Kauil moves.

## Requierements
- `/src/twist_listener.py`
  - Installed in Kauil's `catkin_ws`.
  - Arduino connected to Kauil and Sabertooth.
  - Access to the Arduino's usb port. [How to](#further-information).

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