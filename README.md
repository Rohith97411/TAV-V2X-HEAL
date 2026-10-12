# TAV - Toyota ADAS Verification
## V2X-HEAL Camera Fault Recovery Prototype

### Project Overview
TAV is a proposed ADAS health verification and evidence-management framework.
V2X-HEAL is a software proof of concept demonstrating a simulated
post-verification camera calibration fault scenario.

### Features
- CARLA vehicle and environment simulation
- ROS 2 fault communication
- Front-camera yaw fault injection
- Software compensation and recovery visualization
- Pygame monitoring dashboard
- Fault severity and event recording
- Optional UiPath webhook integration for Gmail maintenance alerts

### Technology Stack
- CARLA 0.9.15
- ROS 2 Humble
- Python 3.10
- Pygame and NumPy
- UiPath and Gmail
- Windows 11 with WSL2 Ubuntu 22.04

### Demonstration Workflow
CARLA Simulation -> ROS 2 Fault Injection -> Recovery Logic
-> Dashboard and Event Recording -> UiPath Webhook -> Gmail Alert

### Example
Raw camera yaw: +8 degrees
Software correction: -8 degrees
Calculated effective yaw: 0 degrees

Effective yaw = Raw yaw + Software correction

### Technical Limitations
- The Toyota Prius is the simulation vehicle used in this prototype.
- CARLA Traffic Manager controls vehicle motion; the front-camera
  feed is not directly controlling steering in the current implementation.
- Compensation is simulated and does not physically repair or recalibrate a camera.
- Fault values and severity categories are demonstration settings,
  not Toyota production thresholds.
- Real-vehicle deployment requires physical inspection, approved calibration,
  and appropriate safety validation.

### Build and Run
Start the CARLA 0.9.15 server on Windows.

In WSL Ubuntu:

    source /opt/ros/humble/setup.bash
    cd ~/v2x_heal_ws
    colcon build --symlink-install
    source install/setup.bash

    WIN_HOST=$(ip route | awk '/default/ {print $3; exit}')

    ros2 launch v2x_heal highway_demo.launch.py \
      host:=$WIN_HOST fault_yaw_deg:=0.0

Configure UiPath locally through the UIPATH_WEBHOOK_URL environment variable.
Never upload the actual webhook URL or other credentials.

### Project Status
Academic software prototype and proposed architecture.
Not an official Toyota product or a production-ready ADAS safety system.
