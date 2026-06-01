# 🤖 TurtleBot3 Semi-Autonomous Visual Servoing with Sliding Mode Control

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4-red)
![Control](https://img.shields.io/badge/Control-SMC-green)
![Python](https://img.shields.io/badge/Python-3.10-yellow)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

---

> A semi-autonomous mobile robot system combining human-directed navigation with autonomous visual servoing. The operator drives freely via teleop; when a colored fiducial marker is detected, the robot autonomously aligns, approaches, and stops — powered by Sliding Mode Control.

---

## 📽️ Demo


https://github.com/user-attachments/assets/b22fccbc-473e-4455-bf2d-c72cf6341a84


---

## ✨ Features

- **Semi-autonomous shared control** — human drives freely; SMC takes over on target detection
- **Velocity multiplexer (mux)** — SMC node forwards teleop in WAITING, overrides in active states
- **Robust color detection** — two-stage HSV saturation + circularity filter eliminates brick wall false positives
- **Sliding Mode Control** — nonlinear controller with chattering suppression for smooth visual alignment
- **Active braking** — counters Gazebo inertia for clean stops in front of target
- **Real-time state machine** — explicit transitions: WAITING → ALIGNING → APPROACHING → HOLDING
- **Multi-node ROS 2 architecture** — decoupled perception and control nodes

---

## 🏗️ System Architecture

```
Teleop Keyboard
      │ /cmd_vel_teleop
      ▼
┌─────────────────────────────────────────────────────────┐
│               smc_controller_node                       │
│  WAITING    → forwards teleop to /cmd_vel               │
│  ALIGNING   → SMC angular control (overrides teleop)    │
│  APPROACHING → SMC linear + angular control             │
│  HOLDING    → active braking (overrides teleop)         │
└─────────────────────────────────────────────────────────┘
      ▲                            │ /cmd_vel
      │ /color_follower/*          ▼
┌──────────────────┐          TurtleBot3
│ color_detector   │
│   (Perception)   │◄── /camera/image_raw
└──────────────────┘
```

### ROS 2 Topic Graph

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Robot camera stream |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Human teleop input |
| `/color_follower/detected` | `std_msgs/Bool` | Target found |
| `/color_follower/normalized_error` | `std_msgs/Float32` | Lateral error [-1, 1] |
| `/color_follower/target_area` | `std_msgs/Float32` | Target size [0, 1] |
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity to robot |
| `/color_follower/state` | `std_msgs/String` | Current state |

---

## 🔄 State Machine

```
WAITING ──detected──► ALIGNING ──centered──► APPROACHING ──close──► HOLDING
   ▲                      │                       │                     │
   └──────────── LOST ◄───┴───────────────────────┘           active brake
```

| State | Behavior |
|-------|----------|
| **WAITING** | Human drives freely — SMC forwards teleop commands to `/cmd_vel` |
| **ALIGNING** | Target detected — SMC centers it in frame using angular control |
| **APPROACHING** | Target centered — robot moves forward while maintaining alignment |
| **HOLDING** | Target close — active braking stops robot in front of target |
| **LOST** | Target disappeared — brief stop, returns to WAITING |

---

## 🔍 Robust Perception: Solving False Positives

**Challenge:** The house environment has reddish-brown brick walls that triggered false detections with naive HSV filtering.

**Two-stage filter:**

1. **Saturation >= 200** — Bricks score S=130-160; pure red marker scores S=255. Bricks rejected.
2. **Circularity >= 0.60** — Wall segments score C=0.3-0.5; spherical marker scores C=0.80+. Rejected.

**Result:** 0% false positives on brick walls in a realistic environment.

---

## 🧮 Sliding Mode Control

The controller implements SMC for lateral visual alignment during approach:

- **Sliding surface:** `s = ė + λe`
- **Control law:** `u = -k_s · tanh(s/φ) - k_eq · e`
- **Chattering suppression:** tanh boundary layer (φ = 0.1)

| Parameter | Value | Description |
|-----------|-------|-------------|
| λ | 1.5 | Sliding surface slope |
| k_s | 0.5 | Switching gain |
| k_eq | 0.3 | Equivalent control gain |
| φ | 0.1 | Boundary layer thickness |

---

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| Robot OS | ROS 2 Humble Hawksbill |
| Robot | TurtleBot3 Waffle Pi |
| Simulation | Gazebo Classic 11 |
| Vision | OpenCV 4 |
| Control | Sliding Mode Control (SMC) |
| Language | Python 3.10 |
| Platform | Ubuntu 22.04 / WSL2 |

---

## 🚀 Getting Started

### Prerequisites

```bash
sudo apt install ros-humble-desktop ros-humble-turtlebot3* \
  ros-humble-gazebo-ros-pkgs python3-opencv ros-humble-cv-bridge
```

### Build

```bash
git clone https://github.com/Sara-Esm/turtlebot3_color_follower.git
cd turtlebot3_color_follower
colcon build
source install/setup.bash
```

### Run

**Terminal 1 — Gazebo house environment:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2 — Color follower system (auto-spawns red marker):**
```bash
source install/setup.bash
ros2 launch turtlebot3_color_follower color_follower.launch.py
```

**Terminal 3 — Teleop (remapped for SMC velocity mux):**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_teleop
```

### Demo Flow

1. Drive robot toward garbage bin using keyboard
2. Robot detects red marker → `WAITING → ALIGNING → APPROACHING → HOLDING`
3. Robot autonomously aligns and stops cleanly in front of target
4. Drive away → returns to `WAITING`, human control restored

---

## 📊 Results

| Metric | Value |
|--------|-------|
| Detection rate | 30 Hz |
| False-positive rate (brick walls) | 0% |
| Circularity of red sphere target | ~0.82 |
| Max angular speed | 0.6 rad/s |
| Approach speed | 0.06 m/s |
| Control method | Sliding Mode Control |

---

## 🔑 Key Implementation Details

**Velocity Multiplexer** (`smc_controller_node.py`)
- Subscribes to `/cmd_vel_teleop` and `/color_follower/*`
- In WAITING state: forwards teleop Twist directly to `/cmd_vel`
- In active states: SMC commands override teleop completely
- Active braking on HOLDING entry counteracts Gazebo differential drive inertia

**Color Detector** (`color_detector_node.py`)
- Dual HSV range for red (0-10° and 170-180°) handles hue wrap-around
- EMA smoothing (α=0.5) reduces detection noise
- Rejected candidates drawn in grey on debug image for transparency
- Publishes normalized error [-1,1] and normalized area [0,1]

**SMC Controller** (`smc_controller_node.py`)
- Sliding surface with derivative and proportional error terms
- tanh boundary layer eliminates high-frequency chattering
- center_tolerance threshold triggers APPROACHING from ALIGNING

---

## 🔭 Future Work

- [ ] YOLO-based object detection (detect unmarked garbage bins)
- [ ] Depth camera integration for metric distance estimation
- [ ] Real TurtleBot3 hardware deployment
- [ ] Multi-target tracking and prioritization
- [ ] Autonomous recovery behaviors when target is lost
- [ ] ROS 2 lifecycle node management

---

## 👩‍💻 Author

**Sara Esmaeili** — Robotics Software Engineer 
GitHub: [@Sara-Esm](https://github.com/Sara-Esm)  

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.
