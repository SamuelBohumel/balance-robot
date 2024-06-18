Balance robot using ESP32 board and RTOS

## Hardware architecture 
This system is categorized as a Hard Real Time system. If the tasks 
tasks are not completed in sufficient time, the robot will lose equilibrium and fall, i.e. a failure will occur. 
### Esp32  
A development board with an integrated ESP32 microcontroller. It has the possibility of connecting to WiFi and Bluetooth, which 
allows wireless communication and remote control. 
### MPU6050  
Tri-axis gyroscope and accelerometer combined in one chip. This sensor provides information on 
tilt and acceleration of the robot, which is essential for maintaining balance. It can also measure temperature. 
### LN298N 
Bridge H for controlling DC motors. This circuit enables the control and power supply of DC motors and provides 
feedback for robot stabilization and motion. It also has a 5V output which will power the ESP32 board. 
### LED 
The LED is used for information purposes. 
### Button 
Used to enable/disable the robot functionality. 
### DC motor 
The drive elements of the robot. The DC motors are responsible for the movement of the robot and their speed and direction is controlled 
by signals generated from the rest of the system components 
### Battery 
The robot is powered by a battery that provides the energy for its operation. 
### Wiring 
F-F type cables are used to connect all devices.

## Photo
![image](https://github.com/SamuelBohumel/balance-robot/assets/55780851/d0532f5b-9e06-487a-aabe-29a47a2b6200)

## Connections 

![image](https://github.com/SamuelBohumel/balance-robot/assets/55780851/e0599892-695e-4a18-ad9b-c38827b5366f)
