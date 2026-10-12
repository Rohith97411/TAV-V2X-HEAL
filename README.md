# TAV — Toyota ADAS Verification

## V2X-HEAL Camera Fault Recovery Prototype

### Project Overview

TAV is a proposed ADAS health verification and evidence-management framework for pre-owned and serviced vehicles.

V2X-HEAL is an academic software proof of concept for demonstrating a simulated camera calibration fault scenario, fault communication, and software compensation.

### Prototype Features

- CARLA vehicle and environment simulation
- ROS 2 fault communication
- Controlled front-camera yaw fault injection
- Software compensation and recovery-state visualization
- Pygame monitoring dashboard
- Fault severity and event recording
- Optional UiPath webhook integration for maintenance notifications

### Technology Stack

- CARLA 0.9.15
- ROS 2 Humble
- Python 3.10
- Pygame and NumPy
- UiPath and Gmail
- Windows 11 with WSL2 Ubuntu 22.04

### Demonstration Workflow

CARLA Simulation → ROS 2 Fault Injection → Recovery Logic → Dashboard and Event Recording → Optional Maintenance Notification

### Project Status

Academic software prototype and proposed architecture. This is not an official Toyota product or a production-ready ADAS safety system.

### Security

Never commit credentials, webhook URLs, access tokens, or other secrets to the repository.
