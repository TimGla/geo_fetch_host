# GeoFetch Host System

This repository contains the host-side software for the GeoFetch system, designed to run on an **ASUS NUC PRO** located in the IMI Lab. The system expects **Ubuntu 22.04** with **ROS 2 Humble** installed. It serves as the central hub connecting the robot's high-level movement (Go2), the firmware on the ESP32, and a web-based user dashboard.

## 📂 Repository Structure

* **`web_dashboard/`**: A lightweight web interface for system monitoring and manual control.
    * `index.html`: The dashboard UI layout.
    * `dashboard.js`: Logic for Rosbridge communication, status monitoring, and service calls.
* **`setup.sh`**: A convenience script to launch all system components (Micro-ROS, Rosbridge, HTTP Server, and Go2 Bringup) in a managed `tmux` session.

---

## 🛠 Setup & Installation

### 1. NUC Connection
Connect to the NUC via the KIT VPN and SSH:
```bash
ssh nuc-{name}@imi-nuc-{name}.imi.kit.edu
# Default Password: 123
```
*Note: Replace `{name}` with the specific identifier of your Go2 robot.*

### 2. Micro-ROS Agent Setup
The ESP32 communicates via Micro-ROS. The agent must be built on the host:
```bash
# 1. Create workspace and clone setup tools
mkdir -p microros_ws/src
cd microros_ws
git clone -b $ROS_DISTRO [https://github.com/micro-ROS/micro_ros_setup.git](https://github.com/micro-ROS/micro_ros_setup.git) src/micro_ros_setup

# 2. Update dependencies and install pip
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip

# 3. Build micro-ROS tools
colcon build
source install/local_setup.bash

# 4. Create and build the agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### 3. Rosbridge Installation
Ensure the Rosbridge suite is installed:
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server
```

---

## 🚀 Launching the System

The system can be launched manually or via the automated `setup.sh` script.

### Option A: Automated Launch (Recommended)
Run the setup script to initialize a `tmux` session with all necessary terminals:
```bash
./setup.sh
```
**⚠️ Important:** Hardware ports (e.g., `/dev/ttyUSB0`) and server ports (e.g., `7000`) are **hardcoded** in `setup.sh`. If your hardware configuration differs, adjust the `BASE_PATH` and device paths in the script before running.

### Option B: Manual Launch
If debugging, run these in separate terminals:
1. **Micro-ROS Agent:**
   `source microros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200`
2. **Rosbridge:** `ros2 launch rosbridge_server rosbridge_websocket_launch.xml call_services_in_new_thread:=True default_call_service_timeout:=5.0`
3. **Web Server:** `python3 -m http.server 7000` (inside `web_dashboard/`)
4. **Go2 Bringup:** `ros2 launch go2_bringup go2.launch.py`

---

## 🌐 Accessing the Dashboard

Since the NUC operates behind a firewall, you must create SSH tunnels from your local machine to access the interface:

1. **Dashboard Tunnel (HTTP):**
   ```bash
   ssh -L 8080:127.0.0.1:7000 nuc-{name}@imi-nuc-{name}.imi.kit.edu
   ```
   Access the UI at: `http://localhost:8080`

2. **Rosbridge Tunnel (WebSocket):**
   ```bash
   ssh -L 9999:127.0.0.1:9090 nuc-{name}@imi-nuc-{name}.imi.kit.edu
   ```
   Connect the dashboard via the address input field using: `ws://localhost:9999`

---

## 🔍 Troubleshooting

### ESP32 Permission Error
If the Micro-ROS agent fails with `errno: 13 (Permission denied)` when accessing the USB port:
```bash
sudo chown nuc-{name} /dev/ttyUSB0
sudo chgrp nuc-{name} /dev/ttyUSB0
```

### Port Conflicts
If port `7000` (Web Server) or `9090` (Rosbridge) is already in use, you must modify the port assignments in `setup.sh` and the dashboard connection settings.