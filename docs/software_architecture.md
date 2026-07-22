# Carl Software Architecture

## Purpose

This document defines the software responsibilities, data flow, ROS 2 package layout, and embedded-controller boundaries for the Carl autonomous research platform.

The architecture preserves the following principles:

- High-level autonomy remains on the Jetson
- Deterministic safety and arbitration remain on the STM32
- Local actuator control remains on Teensy subsystem nodes
- No high-level process directly commands raw motor PWM
- Every communication layer includes health monitoring and timeout behavior
- Subsystems are validated incrementally before vehicle-level integration

---

# Software Layers

## Layer 1 — Development and Supervision

Platform:

- Windows development laptop

Responsibilities:

- VS Code Remote-SSH development
- Source-code management
- Remote diagnostics
- Teleoperation supervision
- ROS 2 visualization
- Log retrieval
- Software deployment

Primary connection:

```text
Laptop
    |
    | Ethernet / SSH
    v
Jetson Orin Nano Super