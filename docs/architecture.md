# Carl System Architecture

## Purpose

Carl is a distributed autonomous research platform designed around modular embedded controllers.

The primary goals are:

- Deterministic vehicle control
- Modular software
- Distributed embedded architecture
- Safe autonomous operation
- Expandable research platform

---

# High Level Hardware Architecture

Development Laptop
        │
        │ Ethernet / SSH
        ▼
Jetson Orin Nano Super
        │
        │ UART
        ▼
STM32 NUCLEO-F767ZI
        │
        │ CAN Bus
        ▼
Teensy Nodes
        │
        ▼
Motor Drivers
        │
        ▼
Motors

---

# Responsibility Allocation

## Jetson

High-level intelligence

Responsibilities

- ROS2
- Teleoperation
- Computer Vision
- Localization
- Navigation
- Logging
- Diagnostics
- UART Communication

---

## STM32

Vehicle Supervisor

Responsibilities

- Safety
- Watchdog
- Heartbeat
- Command Validation
- UART Parsing
- CAN Distribution

---

## Teensy Nodes

Subsystem Controllers

Responsibilities

- PID
- Encoder Reading
- Motor Driving
- Steering
- Sensor Collection
- CAN Communication

---

# Design Philosophy

Each processor performs exactly one level of responsibility.

The Jetson never directly controls motors.

The STM32 never performs perception.

The Teensys never perform navigation.

Each processor specializes in one task.