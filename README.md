# TurtleBot3 Semi-Autonomous Visual Servoing with SMC

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![OpenCV](https://img.shields.io/badge/OpenCV-4-red) ![Control](https://img.shields.io/badge/Control-SMC-green) ![License](https://img.shields.io/badge/License-MIT-brightgreen)

> A semi-autonomous mobile robot combining human-directed navigation with autonomous visual servoing. The operator drives freely; when a colored fiducial marker is detected (e.g. on a garbage bin), the robot autonomously aligns and approaches using Sliding Mode Control.

---

## 📽️ Demo

**Scenario:** Robot patrols a house environment. Operator drives toward garbage bin marked with a red fiducial tag. Robot autonomously detects, aligns, approaches, and stops in front of the target.

---

## 🏗️ Architecture

```
/camera/image_raw
        │
        ▼
┌──────────────────┐   /color_follower/*    ┌──────────────────┐
│ color_detector   │──(detected,error,area)─►│ smc_controller   │
│  (Perception)    │                        │ (Control + SM)   │
└──────────────────┘                        └────────┬─────────┘
                                                     │ /cmd_vel
                                                     ▼
                                                TurtleBot3
```

### Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/camera/image_raw` | Image | Robot camera stream |
| `/color_follower/detected` | Bool | Target found (yes/no) |
| `/color_follower/normalized_error` | Float32 | Lateral error [-1, 1] |
| `/color_follower/target_area` | Float32 | Normalized target area [0, 1] |
| `/cmd_vel` | Twist | Velocity commands to robot |
| `/color_follower/state` | String | Current state for monitoring |

---

## 🎮 State Machine

```
WAITING ──detected──► ALIGNING ──centered──► APPROACHING ──close──► HOLDING
   ▲                      │                       │
   └────────── LOST ◄─────┴───────────────────────┘
```

- **WAITING:** Human has full teleop control. SMC is dormant. Robot moves freely.
- **ALIGNING:** Target detected. SMC computes angular velocity to center target in frame.
- **APPROACHING:** Target centered. Robot moves forward while maintaining alignment.
- **HOLDING:** Target reached (area threshold exceeded). All motion stops.
- **LOST:** Target disappeared. Brief pause, returns to WAITING for human recovery.

This mirrors real-world shared-autonomy systems where humans handle strategic navigation and the robot handles tactical precision tasks.

---

## 🔍 Robust Perception: Solving False Positives

**Engineering Challenge:** The house environment contains reddish-brown brick walls. Naive HSV filtering produced false positives on wall contours causing erratic robot behavior.

**Solution — Two-stage filtering:**
1. **Saturation threshold ≥ 200** — Bricks (S~130-160) rejected; pure red marker (S~255) accepted
2. **Circularity filter ≥ 0.60** — Irregular wall edges (C~0.3-0.5) rejected; spherical marker (C~0.8) accepted

**Result:** Zero false positives in a cluttered real-world environment.

---

## 🧮 SMC Control Law

- **Sliding surface:** `s = ė + λe`
- **Control law:** `u = -k_s · tanh(s/φ) - k_eq · e`
- **Chattering suppression:** tanh boundary layer (φ = 0.1)

| Parameter | Value | Role |
|-----------|-------|------|
| λ | 1.5 | Sliding surface slope |
| k_s | 0.5 | Switching gain |
| k_eq | 0.3 | Equivalent control gain |
| φ | 0.1 | Boundary layer thickness |

---

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| Robot OS | ROS 2 Humble |
| Robot | TurtleBot3 Waffle Pi |
| Simulation | Gazebo 11 |
| Vision | OpenCV 4 |
| Control | Sliding Mode Control (SMC) |
| Language | Python 3 |
| Platform | Ubuntu 22.04 / WSL2 |

---

## 🚀 Run

**Terminal 1 — Gazebo:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2 — Color follower (auto-spawns red marker at garbage bin):**
```bash
cd ~/projects/turtlebot3_color_follower
source install/setup.bash
ros2 launch turtlebot3_color_follower color_follower.launch.py
```

**Terminal 3 — Teleop:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

**Expected behavior:**
1. Drive robot manually toward the garbage bin
2. Red marker detected → state transitions: `WAITING → ALIGNING → APPROACHING → HOLDING`
3. Robot autonomously aligns and stops in front of target
4. Drive away → state returns to `WAITING`, human control restored

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Detection latency | ~33ms (30 Hz camera) |
| Control loop rate | 20 Hz |
| False-positive rate (brick walls) | 0% |
| Max angular speed | 0.6 rad/s |
| Linear approach speed | 0.15 m/s |

---

## 👩‍💻 Author

**Sara Esmaeili** — Robotics Software Engineer 
GitHub: [@Sara-Esm](https://github.com/Sara-Esm)  
Email: zesmaeili85@gmail.com

---

## 📄 License

MIT License — see LICENSE for details.
