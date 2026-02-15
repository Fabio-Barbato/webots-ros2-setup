# 🤖 Webots + ROS2 Bridge Setup (macOS)

Complete setup guide for running Webots simulator on macOS with ROS2 in Docker, connected via a file-based bridge.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Setup](#quick-setup)
- [System Architecture](#system-architecture)
- [Usage](#usage)

---

## Overview

This setup creates a bidirectional bridge between:
- **Webots simulator** running natively on macOS
- **ROS2 Humble** running in Docker (ARM64)

Communication happens via JSON files in a shared folder, enabling:
- Webots → ROS2: Sensor data published as ROS2 topics
- ROS2 → Webots: Velocity commands controlling robot motors

---

## Prerequisites

### Required

- **macOS** 11.0+
- **Docker Desktop** for Mac ([Download](https://www.docker.com/products/docker-desktop))
- **Webots** R2023b+ ([Download](https://cyberbotics.com/))
- **8GB RAM** minimum (16GB recommended)
- **10GB free disk space**
- Apple Silicon (M1/M2/M3) - Native ARM64

---

## Setup

### Step 1: Clone Repository

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/webots-ros2-setup.git
cd webots-ros2-setup
```

### Step 2: Build Docker Image

```bash
cd docker
./build.sh
```

This will:
- Build ROS2 Humble Docker image
- Install necessary tools

### Step 3: Setup Workspace

```bash
cd ..
./setup.sh
```

This creates:
- `~/webots_ros2_bridge/` - Shared folder for JSON files
- `~/Desktop/webots_robot_project/` - Webots project
- Docker container with ROS2 workspace

### Step 4: Start Container

```bash
./start.sh
```

You're now inside the Docker container!

### Step 5: Build ROS2 Workspace

```bash
# Inside container
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Starting the System

**1. Start Docker container:**

```bash
# On Mac
cd ~/webots-ros2-setup
./start.sh
```

**2. Launch ROS2 nodes (3 terminals):**

**Terminal 1 - Sensor Publisher:**
```bash
docker exec -it ros2_webots bash
source ~/ros2_ws/install/setup.bash
ros2 run webots_bridge sensor_publisher
```

**Terminal 2 - Your Algorithm (obstacle avoider example):**
```bash
docker exec -it ros2_webots bash
source ~/ros2_ws/install/setup.bash
ros2 run webots_bridge obstacle_avoider
```

**Terminal 3 - Command Subscriber:**
```bash
docker exec -it ros2_webots bash
source ~/ros2_ws/install/setup.bash
ros2 run webots_bridge command_subscriber
```

**3. Launch Webots:**

```bash
# On Mac
open /Applications/Webots.app
```

In Webots:
1. Open: `~/Desktop/webots_robot_project/worlds/arena.wbt`
2. Verify robot controller is set to `bridge_controller`
3. Press **▶️ Play**

**🎉 Your robot should now move and avoid obstacles!**

---

## 📊 System Architecture

```
┌─────────────────────────────────────────┐
│ WEBOTS (macOS Native)                   │
│                                         │
│ bridge_controller.py                    │
│  ↓ Reads 8 proximity sensors            │
│  ↓ Writes sensor_data.json              │
│  ↓ Reads cmd_vel.json                   │
│  ↓ Controls robot motors                │
└──────────┬──────────────────────────────┘
           │
           │ ~/webots_ros2_bridge/
           │ (Shared Volume)
           │
           ▼
┌──────────────────────────────────────────┐
│ DOCKER (Linux ARM64/AMD64)               │
│                                          │
│ ROS2 Nodes:                              │
│                                          │
│ sensor_publisher                         │
│  ↓ Reads sensor_data.json                │
│  ↓ Publishes /sensor_0.../sensor_7       │
│                                          │
│ obstacle_avoider (your algorithm)        │
│  ↓ Subscribes /sensor_0                  │
│  ↓ Decides: forward or turn              │
│  ↓ Publishes /cmd_vel                    │
│                                          │
│ command_subscriber                       │
│  ↓ Subscribes /cmd_vel                   │
│  ↓ Writes cmd_vel.json                   │
└──────────────────────────────────────────┘
```

### Data Flow

**Sensors (Webots → ROS2):**
1. Webots reads sensors every 32ms
2. Writes `sensor_data.json`
3. `sensor_publisher` reads JSON (10 Hz)
4. Publishes ROS2 topics `/sensor_0` ... `/sensor_7`

**Commands (ROS2 → Webots):**
1. Algorithm publishes `/cmd_vel` (Twist message)
2. `command_subscriber` receives it
3. Writes `cmd_vel.json`
4. Webots reads JSON and moves motors

### File Formats

**sensor_data.json:**
```json
{
  "timestamp": 1771186500.123,
  "distances": [1486.4, 257.5, 65.9, 68.3, 67.5, 65.0, 248.3, 1464.9],
  "min_distance": 65.0,
  "max_distance": 1486.4
}
```

**cmd_vel.json:**
```json
{
  "linear": 0.2,
  "angular": 0.0
}
```

---

## 🛠️ Available ROS2 Nodes

### sensor_publisher

Converts Webots sensor data to ROS2 topics.

**Published Topics:**
- `/sensor_0` to `/sensor_7` (sensor_msgs/Range)

### command_subscriber

Converts ROS2 velocity commands to Webots format.

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist)

### obstacle_avoider

Example algorithm that avoids obstacles.

**Logic:**
- Sensor > 0.2m → Obstacle detected → Turn
- Sensor ≤ 0.2m → Clear → Go forward

**To create your own algorithm:**

```bash
# Copy the template
cp ~/ros2_ws/src/webots_bridge/webots_bridge/obstacle_avoider.py \
   ~/ros2_ws/src/webots_bridge/webots_bridge/my_algorithm.py

# Edit your algorithm
nano ~/ros2_ws/src/webots_bridge/webots_bridge/my_algorithm.py

# Add entry point in setup.py
nano ~/ros2_ws/src/webots_bridge/setup.py
# Add: 'my_algorithm = webots_bridge.my_algorithm:main'

# Rebuild
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run
ros2 run webots_bridge my_algorithm
```

---

**Happy robot building! 🤖✨**
