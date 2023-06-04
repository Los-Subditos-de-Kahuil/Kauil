# Arduino
Files corresponding to Kauil's Arduino controll system.

## Index
- [Arduino](#arduino)
  - [Index](#index)
  - [Files](#files)
  - [Authors](#authors)


## Files
- [ENCODER.ino](ENCODER.ino)
  - Count pulses from the encoders, adding when turning in one direction and substracting when going in the other.
- [EncoderCuadratura.ino](EncoderCuadratura.ino)
  - Check enconder counts with high precision.
- [EncoderCuadraturaVelocidadAngular.ino](EncoderCuadraturaVelocidadAngular.ino)
  - Calculate angular velocity from encoder information.
- [EncoderCuadraturaVelocidadLineal.ino](EncoderCuadraturaVelocidadLineal.ino)
  - Calculate linear velocity from encoder information.
- [EncoderCuadraturaVelocidadRPM.ini](ROSEncoderCuadraturaVelocidadRPM.ino)
  - Calculate the velocity of the wheels in rpm from the information of the encoders.
- [ManualPWM.ino](ManualPWM.ino)
  - Read serial duty cycle required and update. This will move the motors as it  simulates a message from the radiowave controller.
- [ROSEncoderCuadraturaVelocityRPM.ino](ROSEncoderCuadraturaVelocidadRPM.ino)
  - Calculate the velocity of the wheels in rpm from the information of the encoders and transmited through ROS.

## Authors
- [Nancy García Jiménez](https://github.com/nansnova)
- [D. Alberto Anaya Márquez](https://github.com/A01379375) 
- [Alejandro Domínguez Lugo](https://github.com/AlDomL9)