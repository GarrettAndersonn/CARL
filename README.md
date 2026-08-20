CARL — Drive-by-Wire Platform for Autonomous Ground Vehicle Research

CARL is a modular distributed drive-by-wire research platform developed through the AVL Summer Cohort at California State Polytechnic University, Pomona.

The project provides the embedded hardware, communications, safety, and actuator-control foundation required to connect high-level autonomous vehicle software running in ROS 2 with real-time vehicle actuation.

Rather than controlling the vehicle directly from the autonomy computer, CARL uses a layered architecture consisting of an NVIDIA Jetson Orin Nano Super, an STM32 NUCLEO-F767ZI master controller, and distributed Teensy 4.1 actuator nodes connected over CAN.

The result is a reusable research platform for continued development in autonomous navigation, closed-loop vehicle control, state estimation, perception, and sensor fusion.

⸻

System Overview

CARL separates high-level autonomy from deterministic embedded vehicle control.

Remote Operator / Autonomous Software
                │
                │ Wi-Fi / SSH / ROS 2
                ▼
      NVIDIA Jetson Orin Nano Super
   ROS 2 • Teleoperation • Diagnostics
                │
                │ CRC-Protected UART
                ▼
        STM32 NUCLEO-F767ZI
 Command Arbitration • Safety • Watchdog
        Protocol Parsing • CAN Manager
                │
                │ CAN 2.0
                ▼
   ┌───────────────────────────────┐
   │      Distributed Nodes        │
   │                               │
   │ Teensy 4.1        Teensy 4.1 │
   │ Throttle/Drive     Steering   │
   └───────────────────────────────┘
          │               │
          ▼               ▼
      Motor Drive      Steering
       Actuation       Actuation
          ▲               ▲
          │               │
      Encoders       Position / Limits
          │
          └──── Vehicle Feedback ────►

⸻

Major Hardware

High-Level Compute

NVIDIA Jetson Orin Nano Super

The Jetson serves as the high-level vehicle computer and hosts:

* ROS 2
* Keyboard teleoperation
* Vehicle-interface nodes
* Diagnostics
* Encoder and actuator feedback
* Future perception and navigation algorithms
* Future autonomous control

Wireless connectivity allows development and operation without requiring a physical operator connection to the vehicle.

⸻

Master Vehicle Controller

STM32 NUCLEO-F767ZI

The STM32 acts as the master embedded controller / VCU.

Primary responsibilities include:

* Receiving commands from the Jetson
* CRC-protected UART communication
* Command parsing
* Command arbitration
* Safety-state management
* Heartbeat supervision
* Watchdog behavior
* Emergency-stop handling
* CAN message generation
* Distributed-node communication
* Sensor and actuator feedback forwarding

The STM32 isolates high-level ROS 2 software from the vehicle’s real-time actuator network.

⸻

Distributed Actuator Controllers

Teensy 4.1

Dedicated Teensy 4.1 nodes are used for distributed actuator control.

Current architecture includes:

Node 0x02 — Throttle / Drive
Node 0x03 — Steering

The throttle node handles:

* CAN command reception
* Four-motor drive commands
* Differential-drive control
* Motor output generation
* Encoder acquisition
* Wheel-speed estimation
* Throttle-status reporting
* Heartbeat supervision
* Emergency-stop response
* Command timeout behavior

The steering node provides the foundation for electronic steering actuation, position feedback, and limit protection.

⸻

Electrical Architecture

CARL uses a 24 V electrical system with dedicated fused power distribution.

The power architecture separates:

* High-current actuator power
* Embedded controller power
* Compute power
* Sensor power

DC-DC converters provide regulated voltage rails for embedded electronics while isolating sensitive compute and control hardware from high-current motor loads.

Major power-system components include:

* 24 V battery system
* Main system fuse
* Fused branch distribution
* Jetson power conversion
* STM32 power conversion
* Teensy power conversion
* Motor-driver power
* Sensor / peripheral power
* Common chassis / ground distribution

The modular distribution architecture supports easier debugging, maintenance, and future hardware expansion.

⸻

Drive System

CARL uses four independently driven electric motors.

Motor actuation is performed using:

* 4× electric drive motors
* BTS7960 H-bridge motor drivers
* Teensy 4.1 throttle controller
* CAN-based commands
* Quadrature encoder feedback

Supported drive behavior includes:

* Forward
* Reverse
* Differential left turn
* Differential right turn
* Neutral
* Soft stop
* Emergency stop

Motor commands are represented using normalized per-mille values.

Example:

0      = stopped
250    = approximately 25% command
500    = approximately 50% command
1000   = maximum command

⸻

ROS 2 Integration

CARL uses ROS 2 as the high-level software interface.

The ROS 2 bridge communicates with the STM32 through a custom UART companion protocol.

Current ROS functionality includes:

* Vehicle command transmission
* Keyboard teleoperation
* Heartbeat monitoring
* Throttle-status feedback
* Encoder feedback
* Diagnostics

Typical CARL topics include:

/carl/heartbeat
/carl/throttle_status
/carl/encoders

⸻

Companion UART Protocol

Communication between the Jetson and STM32 uses a custom binary serial protocol.

The protocol includes:

* Message framing
* Command packets
* CRC protection
* Bidirectional communication
* Command forwarding
* Sensor / status forwarding

Current serial configuration:

Baud Rate: 115200
Transport: Serial

A persistent Linux device path is preferred over /dev/ttyACM#, because ACM device numbers may change after reconnection or reflashing.

Example configuration:

/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_...

This prevents the ROS bridge from accidentally connecting to the Teensy when USB enumeration changes.

⸻

CAN Network

The STM32 communicates with distributed embedded controllers using CAN 2.0.

The CAN network is responsible for real-time communication between:

STM32 Master / VCU
       │
       ├── Throttle Teensy
       ├── Steering Teensy
       └── Vehicle Sensors / Future Nodes

CAN traffic includes:

* Heartbeats
* Throttle commands
* Steering commands
* Emergency-stop state
* Encoder feedback
* Throttle status
* Sensor information
* Diagnostic data

The architecture allows additional vehicle controllers to be added without redesigning the high-level software interface.

⸻

Safety Architecture

Safety behavior is implemented across multiple levels of the system.

CARL currently incorporates:

* Emergency-stop handling
* Heartbeat monitoring
* Communication watchdogs
* Command timeout
* Armed / disarmed state
* Motor-command limiting
* Fail-safe zero output
* Distributed-node supervision
* Incremental subsystem validation

If valid command or heartbeat communication is lost, actuator control returns to a safe state rather than maintaining the last commanded output.

Typical healthy throttle-controller diagnostics include:

haveHb=1
hbLost=0
estop=0
armed=1

⸻

Encoder Feedback

Quadrature encoders are used to measure rear-wheel rotation.

Encoder testing was performed using controlled manual rotations to determine independent counts-per-revolution values for each wheel.

Measured calibration:

Left Encoder  ≈ 12,031 counts/revolution
Right Encoder ≈ 12,181 counts/revolution

Because the two encoders do not produce exactly the same number of counts per revolution, each side is calibrated independently.

⸻

Wheel-Speed Estimation

CARL performs wheel-speed estimation directly on the throttle Teensy.

The processing chain is:

Encoder Counts
      │
      ▼
Count Difference
      │
      ▼
Counts / Second
      │
      ▼
Wheel RPM
      │
      ▼
Linear Wheel Speed
      │
      ▼
Speed Comparison / Control Feedback

Wheel diameter:

13 inches

The embedded estimator provides:

* Counts per second
* Wheel RPM
* Linear wheel speed
* Left/right wheel-speed comparison
* Speed mismatch percentage

⸻

Experimental Characterization

Open-loop testing showed measurable left/right drive differences.

At approximately a 25% motor command, testing showed:

Wheel-speed mismatch ≈ 22–24%

At approximately a 50% motor command, testing showed:

Wheel-speed mismatch ≈ 6–8%

These results demonstrated that equal electrical motor commands do not necessarily produce equal wheel velocities.

This motivated development of closed-loop wheel-speed control.

⸻

Closed-Loop Control Development

Wheel-speed control is being introduced incrementally rather than enabling an unvalidated controller on the full vehicle immediately.

Development progression:

Pass-Through Motor Control
          │
          ▼
Encoder Feedback
          │
          ▼
Wheel-Speed Estimation
          │
          ▼
Target-Speed Generation
          │
          ▼
Shadow-Mode PI Calculations
          │
          ▼
Restricted Closed-Loop Validation
          │
          ▼
Future Fully Tuned Closed-Loop Control

Current controller development includes:

* Target RPM generation
* Wheel-speed error calculation
* Proportional/integral control framework
* Output correction limiting
* Diagnostic correction reporting
* Speed-validity checking

The controller has been intentionally developed using staged hardware validation so that new control behavior can be verified without compromising an otherwise working drive-by-wire system.

Fully tuned closed-loop PI control remains an active development milestone.

⸻

Steering

The steering subsystem provides the foundation for electronic steering control.

The architecture includes:

* Dedicated Teensy 4.1 steering controller
* Steering actuator
* Position sensing
* Mechanical limit protection
* CAN command interface
* Safety integration

Continued work includes steering calibration, closed-loop positioning, and integration with higher-level autonomous control.

⸻

IMU / Vehicle Feedback

CARL includes inertial sensing for vehicle-motion feedback.

Planned and ongoing vehicle-state uses include:

* Linear acceleration
* Vehicle orientation
* Stability analysis
* State estimation
* Sensor fusion
* Autonomous-control validation

Future work will combine wheel odometry and IMU information for improved vehicle-state estimation.

⸻

Repository Structure

The project is organized into embedded firmware, ROS 2 software, diagnostics, and supporting documentation.

CARL/
│
├── configs/
│
├── datasets/
│
├── diagnostics/
│
├── docs/
│
├── firmware/
│   └── firmware/
│       ├── master/
│       ├── throttle/
│       │   └── src/
│       │       ├── main.cpp
│       │       ├── wheel_controller.cpp
│       │       └── wheel_controller.h
│       ├── steering/
│       ├── test/
│       └── platformio.ini
│
├── ros2_ws/
│   └── src/
│       ├── carl_bridge/
│       ├── carl_bringup/
│       ├── carl_msgs/
│       └── carl_teleop/
│
├── scripts/
├── teleop/
└── README.md

Directory contents may evolve as development continues.

⸻

Building the Teensy Throttle Firmware

Move to the PlatformIO project:

cd ~/CARL/firmware/firmware

Clean:

pio run -e throttle -t clean
rm -rf .pio/build/throttle

Build:

pio run -e throttle

Upload:

pio run -e throttle -t upload

Press the Teensy Program button once if PlatformIO requests the device.

⸻

Serial Device Identification

Always identify USB devices before assuming an ACM number:

pio device list

Persistent device paths can be viewed with:

ls -l /dev/serial/by-id/

Typical devices include:

Teensyduino USB Serial
STM32 ST-Link VCP

The ROS bridge should connect to the STM32, not the Teensy USB serial interface.

⸻

Starting the ROS 2 Bridge

Open a terminal on the Jetson:

cd ~/CARL/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch carl_bringup carl.launch.py

A successful startup should report that the CARL bridge is running on the serial transport.

Only one bridge instance should be active.

Verify:

ros2 node list

Expected:

/carl_bridge

Check heartbeat publishers:

ros2 topic info /carl/heartbeat --verbose

Expected:

Publisher count: 1

⸻

Verifying Heartbeat

In another terminal:

source /opt/ros/jazzy/setup.bash
source ~/CARL/ros2_ws/install/setup.bash

Then:

timeout 10 ros2 topic hz /carl/heartbeat

Expected heartbeat rate:

approximately 20 Hz

⸻

Keyboard Teleoperation

Start the teleoperation node:

source /opt/ros/jazzy/setup.bash
source ~/CARL/ros2_ws/install/setup.bash
ros2 run carl_teleop keyboard_teleop

Controls:

1 / 2 / 3 / 4   Speed selection
w               Forward
s               Reverse
a               Left
d               Right
space           Soft stop
n               Neutral
g               Drive
x               Emergency stop
r               Reset emergency stop
q               Quit

For initial bench testing:

1
g
w
space

Use the lowest command level and secure or raise the vehicle before commanding wheel motion.

⸻

Teensy Diagnostics

Identify the Teensy persistent path:

ls -l /dev/serial/by-id/

Then monitor it at:

pio device monitor \
  -p /dev/serial/by-id/usb-Teensyduino_USB_Serial_18638140-if00 \
  -b 115200

Typical healthy output includes:

rx increasing
hb increasing
cmd increasing
haveHb=1
hbLost=0
estop=0
armed=1
speedValid=1

⸻

Engineering Validation

CARL development follows an incremental verification methodology.

Typical validation sequence:

Firmware Build
      ↓
Controller Flash
      ↓
Local Serial Diagnostics
      ↓
CAN Verification
      ↓
Heartbeat Verification
      ↓
ROS 2 Bridge Verification
      ↓
Feedback Verification
      ↓
Low-Speed Bench Test
      ↓
Safety Verification
      ↓
Commit / Restore Point

Git restore points are created after major verified milestones to reduce the risk of firmware regressions during continued development.

⸻

Common Troubleshooting

/carl/heartbeat is not available

Check:

ros2 node list

If /carl_bridge is missing, start the ROS bridge.

⸻

Multiple /carl_bridge entries

Check:

pgrep -af "bridge_node|ros2 launch carl_bringup"

Only one bridge instance should be running.

Multiple bridge instances can compete for the serial device and create duplicate ROS publishers.

⸻

USB ACM numbers changed

Do not assume:

/dev/ttyACM0
/dev/ttyACM1

always correspond to the same hardware.

Run:

pio device list

or use:

ls -l /dev/serial/by-id/

Persistent by-id paths should be used where possible.

⸻

No STM32 serial device

Verify the ST-Link USB connection using:

lsusb
pio device list
ls -l /dev/serial/by-id/

The STM32 should appear as an STMicroelectronics / ST-Link VCP device.

⸻

Teensy CAN communication is missing

Verify Teensy diagnostics.

Healthy communication should show increasing:

rx
hb
cmd

and:

haveHb=1
hbLost=0

⸻

Current Project Status

Validated

* 24 V vehicle electrical architecture
* Distributed embedded controller architecture
* Jetson ROS 2 environment
* CRC-protected Jetson ↔ STM32 UART communication
* STM32 master-controller operation
* CAN 2.0 communication
* Teensy throttle controller
* Four-motor actuation
* Differential-drive operation
* Forward and reverse commands
* Left/right turning
* Emergency-stop behavior
* Heartbeat supervision
* Watchdog / fail-safe behavior
* Throttle-status feedback
* Quadrature encoder acquisition
* Independent encoder calibration
* Embedded wheel-speed estimation
* ROS 2 encoder feedback
* Wireless teleoperation
* Hardware diagnostics
* Incremental firmware validation

In Development

* Restricted closed-loop wheel-speed control
* PI-controller tuning
* Steering integration and calibration
* Additional vehicle-state feedback

Future Work

* Fully tuned closed-loop wheel-speed control
* Closed-loop steering
* Wheel odometry
* IMU fusion
* Vehicle-state estimation
* Autonomous navigation
* Perception integration
* Closed-loop trajectory control
* Low-speed autonomous vehicle testing

⸻

Engineering Lessons

Several system-integration issues encountered during development influenced the final architecture.

Examples include:

* Independent encoder calibration was required because the left and right encoders produced different counts per revolution.
* Encoder polarity and direction required hardware-level verification.
* Open-loop motor commands produced measurable left/right wheel-speed differences.
* CAN communication was validated incrementally at each controller layer.
* UART and ROS 2 communication were tested independently before full-system integration.
* Linux /dev/ttyACM# assignments changed after reconnecting devices, motivating persistent /dev/serial/by-id configuration.
* Multiple ROS bridge instances could compete for the same serial interface, demonstrating the importance of process verification.
* Git-based restore points and incremental hardware testing reduced the impact of firmware regressions.

These lessons shaped CARL into a more maintainable and fault-tolerant research platform.

⸻

Project Outcome

The primary result of the CARL project is not simply motor actuation.

The project established a modular bridge between:

High-Level Autonomous Software
              │
              ▼
Distributed Real-Time Vehicle Control

CARL now provides an embedded research foundation that can support continued development in autonomous navigation, perception, vehicle control, odometry, sensor fusion, and full-system autonomous vehicle testing.

⸻

AVL Summer Cohort

This work was conducted as part of the AVL Summer Cohort in the Autonomous Vehicle Laboratory (AVL) at California State Polytechnic University, Pomona, under the direction of Dr. Behnam Bahr.

The project was developed through collaboration among Summer Cohort participants and members of the Autonomous Vehicle Laboratory.

⸻

Acknowledgments

We thank:

* Dr. Behnam Bahr
* Cal Poly Pomona Autonomous Vehicle Laboratory
* AVL
* AVL Summer Cohort participants
* California State Polytechnic University, Pomona
* Project contributors and collaborators

for their guidance, support, and contributions to the development of CARL.

⸻

Project Demonstration

CARL has been demonstrated using ROS 2 keyboard teleoperation through the complete control path:

Operator
   ↓
ROS 2
   ↓
Jetson Orin Nano Super
   ↓
CRC-Protected UART
   ↓
STM32 Master Controller
   ↓
CAN 2.0
   ↓
Teensy Actuator Controller
   ↓
Motor Driver
   ↓
Vehicle Motion
Encoder / Status Feedback
   ↑
   └──────── back through the architecture

This end-to-end path forms the foundation for future autonomous control.