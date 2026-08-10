# VEX-U Push Back 2026 Competition Repository

## Overview
This repository contains the code used for the VEX-U Push Back 2026 competition, including both operator-control and autonomous programs.

> [!NOTE]
> This branch is an experimental build for testing **PROS 4.2.2** and **LemLib**. It replaces the previous PROS 3.x and OkapiLib-based setup.

## Features
- **Competition Programs**: Complete autonomous and operator-control lifecycle support.
- **Autonomous Routines**: Multiple match, skills, parking, and robot-specific routines.
- **LemLib Integration**: Chassis control, odometry, move-to-point, move-to-pose, heading turns, motion cancellation, and path following.
- **Calibration and Testing**: Sensor calibration, low-speed turn tests, heading-error reporting, and tracking-wheel offset tests.
- **Multiple Robot Configurations**: Selectable small- and big-robot hardware configurations with separate motor, tracking-wheel, and drivetrain parameters.
- **Multiple Drivetrains**: Differential, X-drive, mecanum, and H-drive implementations.
- **Motion Algorithms**: PID, pure pursuit, holonomic motion, trapezoidal profiles, S-curve profiles, and exponential profiles.
- **Odometry and Sensors**: Tracking-wheel odometry plus encoder, IMU, distance, vision, proximity, and color-sensing support.
- **Gated Sensor Fusion**: Allocation-free three-wheel pose propagation and a
  fixed 3-state EKF with IMU and optional GPS corrections. The fused LemLib
  adapter and GPS remain disabled until physical validation.
- **Mechanism Control**: Intake, elevator, scoring, cart, trapdoor, lever, piston, and ring-handling controls.
- **Automated Sorting**: Ring scanning, color sorting, rejection, storage, and scoring-height control.
- **Driver Controls**: Custom controller mappings, joystick scaling, brake-mode configuration, and mechanism toggles.
- **Brain GUI**: Alliance and autonomous selection for red, blue, and skills routines.
- **Debug GUI**: Autonomous runner, live graphing, runtime variable adjustment, data monitoring, field mapping, and reset handlers.
- **Autonomous Safety**: Emergency motor-stop support while testing autonomous routines.
- **Utilities**: Logging, JSON support, moving averages, timers, vectors, function registration, and reusable math helpers.
- **Modular Architecture**: Separate configuration, robot lifecycle, autonomous actions, controls, sensing, mechanisms, and tooling modules.

## Getting Started
Developing from both the lab laptop and another PC requires a clean handoff on
the shared `Testing` branch. Read [the current development handoff](docs/CURRENT_HANDOFF.md)
and follow [the two-computer workflow](docs/TWO_COMPUTER_WORKFLOW.md) before
editing from a different machine or development session.

### Localization contract

Localization uses inches and degrees at public interfaces, with `+X` right,
`+Y` forward at heading zero, and clockwise-positive heading. Autonomous code
continues to consume LemLib's preserved pose path until the physical integration
gate authorizes the fused adapter. See
[the localization validation and benchmark procedure](docs/testing/localization-benchmark.md)
for equations, tuning values, assumptions, and required robot measurements.

### Requirements
- VEX V5 Brain & Motors
- PROS 4.2.2 ([Installation Guide](https://pros.cs.purdue.edu/))
- LemLib
- Computer with PROS CLI or PROS Editor

### Cloning the Repository
```sh
git clone https://github.com/AON-Robotics/Push-Back.git
cd "Push-Back"
```

### Building & Uploading Code
To build and upload the code to the VEX V5 Brain, run:
```sh
pros build
pros upload
```
For real-time debugging:
```sh
pros terminal
```

#### For ASCII Art [Click Here](https://patorjk.com/software/taag/#p=display&f=Small&t=Type%20Something)
