# Robocon 2026 — Team Phoenix, IIT Patna

Software stack for **DD Robocon 2026 National Finals**

## 📂 Repository Structure

```
Robocon26/
├── Robot_1/                    # Manual / Teleoperated Robot
│   ├── firmware/
│   │   └── teensy.ino          # Teensy 4.x low-level controller
│   └── ros2_ws/
│       ├── src/                # ROS2 pkgs for manual robot
│       │   └── r1_joy_serial/  # Joystick → Serial bridge package (ROS 2)
│       ├── start_r1_robot.sh   # Boot startup script
│       └── r1_robot.service    # systemd auto-start unit
│
├── Robot_2/                    # Autonomous Robot
│   ├── firmware/
│   │   └── Arduino_Mega.ino    # Arduino Mega low-level controller
│   └── ros2_ws/
│       ├── src/                      # ROS2 pkgs for autonmous robot
│       │   ├── robocon_behaviour/    # BehaviorTree.CPP mission executor 
│       │   ├── robocon_bringup/      # Launch files & config (hw + sim) 
│       │   ├── robocon_controller/   # Omni-drive controller (hw + sim)
│       │   ├── robocon_description/  # URDF, meshes, Gazebo world
│       │   ├── robocon_interfaces/   # Custom ROS 2 msgs & srvs
│       │   ├── robocon_planner/      # Dijkstra-based high-level planner
│       │   ├── robocon_state/        # Zone/block state publisher
│       │   ├── robocon_vision/       # YOLO-based KFS & spear detection
│       │   └── r2_joy_serial/        # Serial bridge (R2)
│       └── 3rd_party_pkgs/           # Git submodules (see below)
│
└── docs/                       # Official rulebook & appendix PDFs
```

---

## 🤖 Robot 1 — Manual (Teleop)

| Component | Details |
|---|---|
| **Compute** | Raspberry Pi 5 (ROS 2 Jazzy) |
| **Microcontroller** | Teensy 4.1 |
| **Drive** | 4-wheel omni `+` configuration |
| **Actuators** | Dual arm lead-screw steppers, weapon stepper, arm servos, weapon gripper servo, pneumatic gripper relay |
| **Control** | Gamepad controller → ROS 2 `joy` → serial commands over USB |

The Teensy firmware receives string commands (`CMD_VEL`, `ARM_STEP`, `SERVO_ARM_STEP`, `GRIPPER_ON/OFF`, etc.) over `Serial1` and directly drives motors/servos/steppers in a non-blocking loop.

Auto-starts on boot via the included `systemd` service.

---

## 🤖 Robot 2 — Autonomous

| Component | Details |
|---|---|
| **Compute** | Jetson Orin Nano (ROS 2 Jazzy) |
| **Microcontroller** | Arduino Mega |
| **Drive** | 4-wheel omni `X` configuration |
| **LiDAR** | Unitree L2 |
| **SLAM** | Point-LIO (3D LiDAR-Inertial Odometry) |
| **IMU** | BNO085 (onboard, read via I2C at 200 Hz) |
| **Distance** | VL53L0X ToF sensor (weapon rack alignment) |
| **Vision** | YOLOv8 (KFS classification) + YOLOv8 TensorRT (spear/fist/palm detection) |
| **Control** | Joystick override + fully autonomous mission via Behavior Trees |

### Software Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    MISSION EXECUTOR                         │
│              (BehaviorTree.CPP - mission.xml)               │
│   Sequences: scan → pick KFS → navigate → exit → place     │
└──────────────────────┬─────────────────────────────────────┘
                       │ GetPlan srv
┌──────────────────────▼─────────────────────────────────────┐
│                  HIGH-LEVEL PLANNER                          │
│          (Dijkstra-based forest path planning)              │
│   Maintains internal maze memory (12 blocks)                │
│   Handles harvest, relocate, and exit strategies            │
└──────────────────────┬─────────────────────────────────────┘
                       │ MoveToBlock / FaceDirection / PickKFS srvs
┌──────────────────────▼─────────────────────────────────────┐
│                 HARDWARE CONTROLLER                          │
│       Omni X-drive kinematics + serial command bridge       │
│   PID goal-pose tracking with Point-LIO odometry            │
└──────────────────────┬─────────────────────────────────────┘
                       │ CMD_VEL serial
                ┌──────▼──────┐
                │ Arduino Mega │
                └─────────────┘
```

**Parallel subsystems:**
- `robocon_vision` — YOLO inference for KFS classification (real/fake) and spear alignment with proportional velocity control
- `robocon_state` — Zone & block state publisher using LIO odometry + game field map
- `Point-LIO` — Real-time 3D LiDAR-inertial odometry via Unitree L2

### Key ROS 2 Interfaces

| Type | Name | Purpose |
|---|---|---|
| **msg** | `ZoneState` | Current zone, facing block, height, slope status |
| **srv** | `GetPlan` | BT queries planner for next action |
| **srv** | `ScanBlock` | Trigger YOLO KFS detection on a specific block |
| **srv** | `MoveToBlock` | Navigate between adjacent forest blocks |
| **srv** | `FaceDirection` | Rotate to face a specific adjacent block |
| **srv** | `PickKFS` / `PlaceKFS` | Pick up or place a Kung Fu Scroll |

### Vision Pipeline

- **KFS Detection** (`kfs_detection.py`) — YOLOv8 `.pt` model classifies Kung Fu Scrolls as `r2_kfs_real`, `r2_kfs_fake`, or `r1_kfs`. Runs at 10 Hz with CUDA/FP16. Maintains a live vision-to-planner memory update loop.
- **Spear Alignment** (`spear_detection.py`) — YOLOv8 TensorRT `.engine` for real-time spear/fist/palm detection. Implements proportional velocity control for lateral (pixel error) and forward (ToF distance) alignment. Includes a 2-second stable-alignment confirmation timer before triggering the next action.

### Autonomous Mission Flow

1. **Forest Entry** — Navigate to the first checking location
2. **KFS Scanning** — YOLO vision scans blocks, waits if opponent's KFS is detected
3. **Harvest Loop** — Dijkstra planner explores the 4×3 grid, picks real KFS (up to 2), relocates blocking scrolls
4. **Forest Exit** — Plans shortest path to exit blocks
5. **Ramp Traverse** — Retracts extrusions, navigates ramp into Martial Club
6. **Scroll Placement** — Places scrolls on the Tic-Tac-Toe rack
7. **Lift Preparation** — Gets into lifting position for R1

---

## 🔧 3rd Party Dependencies (Git Submodules)

| Submodule | Purpose |
|---|---|
| [unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk) | Unitree L2 LiDAR driver |
| [point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) | Point-LIO SLAM (ROS 2 port) |

```bash
git clone --recurse-submodules https://github.com/Anany444/Robocon26.git
# or if already cloned:
git submodule update --init --recursive
```

---

## 🚀 Quick Start

### Prerequisites
- ROS 2 (Jazzy for RPi / Humble or Jazzy for Jetson)
- Gazebo Harmonic (for simulation only)
- Python 3.10+, PyTorch, Ultralytics (YOLOv8)
- Arduino IDE / PlatformIO (for firmware flashing)

### Build

```bash
# Robot 1 (on Raspberry Pi)
cd Robot_1/ros2_ws
colcon build --symlink-install

# Robot 2 (on Jetson Orin Nano)
cd Robot_2/ros2_ws
colcon build --symlink-install
```

### Launch

```bash
# Robot 2 — Hardware (full autonomous stack)
ros2 launch robocon_bringup hw.launch.py team:=red

# Robot 2 — Simulation
ros2 launch robocon_bringup sim.launch.py
```

### Robot 1 — Auto-Start on Boot

```bash
sudo cp Robot_1/ros2_ws/r1_robot.service /etc/systemd/system/
sudo systemctl enable r1_robot.service
sudo systemctl start r1_robot.service
```

---
