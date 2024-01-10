# Robotic Arm Upper Machine

This repo contains a Windows GUI software and controller of a servo-driven robotic arm.

The Windows GUI software is a QT-based software used to control the arm hardware on PC, and the robotic arm is driven by 5 servos, controller by Arduino Uno R3.

[![demo gif](https://i.imgur.com/ko9ZEhB.gif)](https://www.bilibili.com/video/BV1Ce411U72K/)

## Develop Platform

- Qt 8.0.2 Community(Qt 5.15.2 MinGW 64-bit)

- Arduino IDE 2.2.1

## QT GUI software

The GUI software is built using QT for controlling arm through communication with Arduino Uno R3. 

The software includes 2 windows, one for 3D model display,  and the other for serialport configuration.

![software windows](https://i.imgur.com/AGJUmFt.png)

### 3D Model Display

For 3D model display, I use Qt3D module which is embedded in QT. **But it is notable that I only implemented the display and joint rotation of the 3D model, I didn't find any docs about collision in QT3D.** And the examples and blogs on QT3D using C++ are few.

### Serial Communication

For serial communication, I use QSerialPort module which is embedded in QT. A simple communication protocol is designed to exchange message safely between devices. The Protocol decomposition is illustrated as below.

![communication protocol decomposition](https://i.imgur.com/0leL5Yx.png)

 ## Arm Hardware

The arm is driven by 5 servos, and servos are controlled through PCA9685 module.

<img src="https://imgur.com/Je9ozRk.jpg" width = "400" height = "300" alt="arm image"/>

 ## Arduino Controller

The controller is a Arduino Uno R3. It receives the message from PC and controls the servos to drive the arm.

I adopted a simple main_loop style for the Arduino controller. The control flow chart is as below.

<img src="https://i.imgur.com/hyL5ENg.png" width="200" height="600" alt="arduino control flow chart"/>