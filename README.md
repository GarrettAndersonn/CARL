# Carl Autonomous Research Platform

## Overview

Carl is a 25.6 V distributed drive-by-wire autonomous research vehicle built around:

- NVIDIA Jetson Orin Nano Super
- STM32 NUCLEO-F767ZI master controller
- Teensy 4.1 subsystem nodes
- CAN bus communication
- Jetson-to-STM32 UART communication
- BTS7960 brushed DC motor drivers
- ROS 2 Jazzy
- Differential-drive vehicle control

## System Architecture

```text
Development Laptop
        |
        | Ethernet / SSH
        v
Jetson Orin Nano Super
        |
        | UART
        v
STM32 NUCLEO-F767ZI
        |
        | CAN
        v
Teensy 4.1 Subsystem Nodes
        |
        v
Motor Drivers, Motors, Encoders, and Sensors
