Ressources for engineering 
==========================

This repository is dedicated to collect and share ressources about engineering for robotic, embedded system and everything about the maker community!
If you are an hobbyist, an enthusiast, a maker or an engineer this repository is for you!
You can use this repository for engineering purposes, for things like [Eurobot](https://www.eurobot.org/) or in a [FabLabs](https://www.fablabs.io/).

Ressources are mostly in english :gb:, but some of them are in french :fr: (I'm french 🙂).

If your file is in this repo and you want to pull out this file, please contact me !
To participate to this repo -> [New pull request](https://github.com/CorentinLAMY/Engineering-ressources/pulls)

Table of contents:
=================

* [Embedded Systems](#1-embedded-systems)
  * [Maker MCUs](#11-maker-mcus)
  * [Industrial MCUs](#12-industrial-mcus)
  * [Linux singleboard computers](#13-linux-singleboard-computers)
  * [FPGA](#14-fpga)
  * [Accessoiries and other stuff](#15-accessoiries-and-other-stuff)
* [Electronical Engineering](#2-electronical-engineering)
  * [Ressources](#21-ressources)
* [Mechanical Engineering](#3-mechanical-engineering)
  * [Ressources](#31-ressources)
* [Robotics](#4-robotics)
  * [Frameworks and tools](#41-frameworks-and-tools)
* [Manufacturing](#5-manufacturing)
  * [test](#test)

# 1. Embedded Systems
Needed knowleadges: 
- C/C++
- Basics of electronic -> [Advance electronics](#2-electronical-engineering)
- Python


## 1.1. Maker MCUs
* [Arduino](https://www.arduino.cc/) is one of the most famous microcontroller in the world. It's Open-Source/Hardware and clones are very cheap in China.
* Raspberry-Pi Fondation designed their first microcontroller the [Raspberry-Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/).
* [ESP32](https://www.espressif.com/en/products/socs/esp32) can be program with the arduino framework.

### 1.1.1 Books :books:
* [Le grand livre d'ARDUINO](https://) :fr:
* [Programming Arduino: Getting started with sketches](https://) :gb:
* [Arduino CookBook](https://) :gb: 

### 1.1.2 Development Environment
* [Arduino IDE](http://...) :gb: & :fr: Open-Source IDE
* [Arduino Pro](http://...) :gb: & :fr: Open-Source IDE for pro grade Arduino boards
* [PlatformIO](http://...) :gb: Open-Source IDE for more than 1000 boards!!!

## 1.2. Industrial MCUs
[STM32](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html),
[PIC32](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/32-bit-mcu),
[ESP32](https://www.espressif.com/en/products/socs/esp32)
Why 32 bits microcontroller? Why not ! Powerfull, Cheap and use in a lot of systems !

### 1.2.1 Books :books:
* [RTOS with Microcontrollers](http://...) :gb:
* [Advanced Programming with STM32 Microcontrollers](http://...) :gb:
* [Nucleo Boards Programming with the STM32CubeIDE](http://...) :gb:

### 1.2.2 Development Environment
You can program on STM32 with:
* [STM32CubeMX & IDE](http://...) :gb: Development environment provide by STMicroelectronics
* [PlatformIO STM32 tutorial](https://docs.platformio.org/en/latest/tutorials/ststm32/stm32cube_debugging_unit_testing.html#tutorial-stm32cube-debugging-unit-testing) :gb: Open-Source IDE

You can program on PIC microcontrollers with:
* [MPLAB X IDE](https://) :gb: For development
* [MPLAB X IPE](https://) :gb: To transfert program
* [MPLAB Harmony](https://) :gb: For project creation with libraries

Most (not all) MCUs like Arduino, STM32, PIC32 and ESP32 can be program and debug with:
* [PlatformIO](https://...) :gb: One of the best IDE available for free.

## 1.3. Linux singleboard computers
* [Raspberry-Pi](https://...)
* [Nvidia Jetson-Nano](https://...)

## 1.4. FPGA
FPGA development for embedded system design.

## 1.5. Accessoiries and other stuff
* Debugger and programmer for MCUs
  * [PICkit 4](https://) for PIC8/16/32 bits. The minimal PIC programmer and debugger.
  * [ICD4](https://) for PIC8/16/32 bits. The best PIC programmer and debugger.
  * [ST-Link/V3](https://) for STM8/32. Made by STMicroelectronic for STM-Microcontroller.
  * [SEGGER J-Link](https://) The best one but expensive.
  * [Tag-Connector](https://) PCB connector without component.


# 2. Electronical Engineering
## 2.1. Ressources
### 2.1.1 Books :books:
### 2.1.2 Videos
### 2.1.3 CAD Softwares
* [Eagle](https://...) Professionnal CAD Software available for free with restriction (Fusion360 is used to visualize the 3D model)
* [KiCad EDA](https://...) Open-Source / Free 
* [EasyEDA](https://...) Free online software

### 2.1.4 Simulation Softwares
* [LTSpice](https://...)

# 3. Mechanical Engineering
## 3.1. Ressources
### 3.1.1 Books :books:
### 3.1.2 Videos
### 3.1.3 CAD Softwares
* [SOLIDWORKS](http://...) Professionnal CAD Software
* [Fusion360](http://...) Free plan for hobbyist (not all function available)
* [OnShape](http://...) Free plan for hobbyist (save accessible by everyone, no private folder)
* [FreeCAD](http://...) Open-Source / Free 

### 3.1.4 Simulation Softwares
* [ANSYS](https://)
* [SOLIDWORKS Simulation](https://)

# 3.2. Standard components
* Bearing
  * [SKF](https://...)
  * [NTN-SNR](https://...)
* Plastic mecanical linkage
  * [igus](https://www.igus.com/) 

# 4. Robotics

## 4.1. Frameworks and tools
### 4.1.1 R.O.S
### 4.1.2 Micro-ROS
### 4.1.3 ROSSerial Library

## 4.2. Mobile robot
### 4.2.1 Frame
The frame of your robot depend of your application but some technologies are often used.
* [MakerBeam](https://www.makerbeam.com/) 10x10mm and 15x15mm aluminium profile kits.
* Lazer cutting parts
* 3D printed parts
* 
### 4.2.2 Wheel
### 4.2.3 Odometry

## 4.3. Robot Arm
### 4.3.1 Type of robot
### 4.3.2 Mathematic ressources

## 4.4. Vision
For vision application, I recommand OpenCV. This library is available in Python and in C/C++.
The C/C++ version is more complex to use but you obtain better performance and you can use it with Nvidia CUDA cores.
* [OpenCV Website](https://docs.opencv.org/master/index.html)
* [OpenCV Python tutorials](https://opencv24-python-tutorials.readthedocs.io/en/latest/)

