# 🧠  TurtleBot3 Color Detector in Custom Indoor World

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/) [![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/) [![Gazebo 11](https://img.shields.io/badge/Gazebo-11-brightgreen)](https://gazebosim.org/) [![WSL2 + WSLg](https://img.shields.io/badge/WSL2-WSLg-lightgrey)](https://learn.microsoft.com/windows/wsl/)

**Real-time computer vision & robotics simulation using ROS 2 Humble, Gazebo 11, and OpenCV**  
*Developed on Ubuntu 22.04 (WSL2 + WSLg) with TurtleBot3 Waffle Pi*
---

## 🚀 Overview

This project simulates a **TurtleBot3 robot navigating a custom Gazebo house environment**, detecting **red-colored objects** in real-time using onboard camera feeds and OpenCV processing.

- Real-time image processing via `cv_bridge` + `OpenCV`
- Manual teleoperation control
- Visualizations in **Gazebo**, **RViz2**, and **rqt_image_view**
- Custom ROS 2 launch systems
- ROS 2 Humble + Gazebo 11 integration under **WSL2 + WSLg**

---

## 🗂️ Repository Structure

```

turtlebot3\_color\_detector\_ws/
├── src/
│   ├── turtlebot3\_gazebo/              # Official TB3 sim package
│   │   └── worlds/turtlebot3\_house.world  # Custom house environment
│   └── turtlebot3\_color\_detector/      # Custom package
│       ├── color\_detector/
│       │   └── red\_detector.py         # Vision node for red detection
│       └── launch/
│           ├── house\_world.launch.py   # Gazebo + RViz + CV
│           └── color\_node.launch.py    # Vision node only
├── README.md

````

---

## 🧪 What You'll See

| Feature                    | Description                                      |
|----------------------------|--------------------------------------------------|
| 🏠 **Gazebo**              | Custom indoor environment with TurtleBot3       |
| 🎮 **Teleoperation**       | Keyboard-based manual robot control             |
| 🔴 **Red Detection**       | Real-time HSV blob detection using OpenCV       |
| 🧭 **RViz2**               | Robot + camera + transform frames               |
| 📸 ***Camera Feed**        | Raw image stream and processed result (rqt)     |

---

## ⚙️ Setup Instructions

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3* \
  ros-humble-rqt-image-view \
  ros-humble-cv-bridge \
  python3-opencv
````

### 2. Set TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build the Workspace

```bash
cd ~/projects/turtlebot3_color_detector_ws
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ How to Run the Project

### 🔄 Full Simulation (Gazebo + RViz2 + CV Node)

```bash
ros2 launch turtlebot3_color_detector house_world.launch.py
```

### 🎮 Manual Teleop Control

```bash
source ~/projects/turtlebot3_color_detector_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use `W/A/S/D/X` keys to move the robot.

### 👁️  Camera View via rqt_image_view

```bash
rqt_image_view /camera/image_raw
```

### 💡 Vision Node Only (If Gazebo is already running)

```bash
ros2 run turtlebot3_cv color_detector
```

## 🧠 How It Works (Red Object Detection)

1. Subscribes to: `/camera/image_raw`
2. Converts images from ROS to OpenCV format (`cv_bridge`)
3. Applies HSV thresholding to isolate red
4. Finds the largest red blob
5. Logs position as `LEFT`, `CENTER`, or `RIGHT` based on centroid

```bash
[INFO] Red object detected at x=325 → CENTER
```

---

## 📸 Screenshots

| Gazebo Simulation    | RViz2 Visualization | Red Blob Detection      |
| -------------------- | ------------------- | ----------------------- |
| ![](docs/gazebo.png) | ![](docs/rviz.png)  | ![](docs/detection.png) |

---

## 🔧 Future Work

* ✅ Make robot follow red objects automatically
* 🔲 Add multi-color detection (e.g., green, blue)
* 🔲 Shape detection using contours
* 🔲 YOLOv8 or TinyML-based detection
* 🔲 Add SLAM + Navigation2 stack
* 🔲 Use depth camera to estimate object distance

---

## ✅ Tested Environment

| Component  | Version / Config           |
| ---------- | -------------------------- |
| OS         | Ubuntu 22.04 (WSL2 + WSLg) |
| ROS 2      | Humble Hawksbill           |
| Gazebo     | Version 11                 |
| GPU        | Intel UHD 620              |
| GUI Server | WSLg (no need for VcXsrv)  |

---

## ⚖️ License

This project is licensed under the [MIT License](LICENSE).

```
