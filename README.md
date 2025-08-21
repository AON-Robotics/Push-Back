# VEX-U High Stakes 2025 Competition Repository

## Overview
This repository contains all the code used for the VEX-U High Stakes 2025 competition, including both the operator-controlled and autonomous period programs. The code is built using the **PROS (Purdue Robotics Operating System) framework** and incorporates **OkapiLib** for motion control and path planning.

## Features
- **Autonomous Period**: Pre-programmed routines for different strategies.
- **Operator-Controlled Period**: Custom control mappings for optimal driver performance.
- **Sensor Integration**: Utilizing encoders, IMUs, LiDAR sensors, and vision sensors for precise movement.
- **Motion Planning**: OkapiLib implementation for smooth and accurate navigation.
- **Modular Code Structure**: Organized codebase for easy modifications and improvements.

## Getting Started
### Requirements
- VEX V5 Brain & Motors
- PROS Framework ([Installation Guide](https://pros.cs.purdue.edu/))
- OkapiLib (Included with PROS)
- Computer with PROS CLI or PROS Editor

### Cloning the Repository
```sh
git clone https://github.com/yourusername/high-stakes-2025.git
cd high-stakes-2025
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
