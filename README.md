cat > ~/projects/turtlebot3_color_follower/README.md << 'READMEOF'
# 🤖 TurtleBot3 Semi-Autonomous Visual Servoing System

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4-red)
![Control](https://img.shields.io/badge/Control-SMC-green)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

> A semi-autonomous mobile robot that combines human-directed navigation with autonomous visual servoing. The operator drives the robot freely; when a colored fiducial marker is detected, the robot autonomously aligns and approaches the target using Sliding Mode Control.

---

## 📽️ Demo



---

## ✨ Key Features

- **Semi-autonomous shared control** — human navigation + autonomous precision approach
- **Robust color detection** — HSV saturation filtering (200) + circularity (0.60) rejects false positives from brick walls
- **Sliding Mode Control (SMC)** — advanced nonlinear control with chattering suppression for smooth visual alignment
- **Fiducial-based approach** — uses colored markers for object identification (industry-standard practice)
- **Real-time state machine** — explicit state transitions: WAITING → ALIGNING → APPROACHING → HOLDING
- **Multi-node ROS 2 architecture** — decoupled perception and control nodes

---

## 🏗️ System Architecture
