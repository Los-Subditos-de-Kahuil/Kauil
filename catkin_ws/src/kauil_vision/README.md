# kauil_vision
Kauil vision nodes.

## Index
- [kauil\_vision](#kauil_vision)
  - [Index](#index)
  - [Files](#files)
  - [Requierements](#requierements)
  - [Authors](#authors)

## Files
- `/src`
  - image_publisher.py
    - Image publisher for Kauil's cameras.
  - l_camera_position_listener.py
    - Read from `/left_camera/cmd` and do a request to move de left camera.
  - r_camera_position_listener.py
    - Read from `/right_camera/cmd` and do a request to move the right camera.
- `/launch`
  - camera_movement.launch
    - Launch `/src/l_camera_position_listener.py` and `/src/r_camera_position_listener.py`

## Requierements
- `/src/*_camera_position_listener.py`
  - Router connected
  - Cameras connected to router and powered
  - Laptop connected to Kauil Neta
- `/launch/camera_movement.launch`
  - Same as `/src/*_camera_position_listener.py` [requierements](#requierements)

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)