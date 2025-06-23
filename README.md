# 🧠 TurtleBot3 Color Detector in Custom House World (ROS 2 Humble + Gazebo)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/) [![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/) [![Gazebo 11](https://img.shields.io/badge/Gazebo-11-brightgreen)](https://gazebosim.org/) [![WSL2 + WSLg](https://img.shields.io/badge/WSL2-WSLg-lightgrey)](https://learn.microsoft.com/windows/wsl/)

**Real-time robotics & computer vision project built with ROS 2 Humble, Gazebo 11, and OpenCV on Ubuntu 22.04 (WSL2 + WSLg)**
---

## 🚀 Project Overview

This project simulates a **TurtleBot3 robot navigating a custom indoor Gazebo world**, detecting red-colored objects in real time using a USB camera and OpenCV. It demonstrates:

- Real-time image processing with `cv_bridge` and OpenCV
- Manual robot control
- Custom launch systems
- Full RViz2 and Gazebo visualization

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

| Feature                     | Description                                      |
|----------------------------|--------------------------------------------------|
| 🏠 **Gazebo**              | Custom indoor environment with TurtleBot3       |
| 🎮 **Manual Control**      | Teleop using keyboard                            |
| 🔴 **Red Detection**       | OpenCV node detects red blobs in live camera feed |
| 🧭 **RViz2**               | Visualize robot, camera, TF tree, etc.           |
| 📸 **rqt_image_view**      | View raw camera image + detection overlay        |

---

## ⚙️ Setup Instructions

### 1. Install Required Packages

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

### 🔄 Full Simulation (Gazebo + RViz + CV)

```bash
ros2 launch turtlebot3_color_detector house_world.launch.py
```

### 🎮 Manual Teleoperation

```bash
source ~/projects/turtlebot3_color_detector_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use `W/A/S/D/X` keys to move the robot.

### 👁️ Live Camera Feed

```bash
rqt_image_view /camera/image_raw
```

### 💡 Vision Node Only

(Assuming Gazebo is already running)

```bash
ros2 run turtlebot3_cv color_detector
```

---

## 🧠 How Red Color Detection Works

The node in `color_detector/red_detector.py`:

* Subscribes to `/camera/image_raw`
* Converts images to OpenCV format using `cv_bridge`
* Applies **HSV color thresholding**
* Finds the largest red blob
* Logs **centroid direction** (LEFT, CENTER, RIGHT)

Example output:

```
[INFO] Red object detected at x=325 → CENTER
```

---

## 📸 Screenshots

| Gazebo Simulation    | RViz Visualization | Red Blob Detection      |
| -------------------- | ------------------ | ----------------------- |
| ![](docs/gazebo.png) | ![](docs/rviz.png) | ![](docs/detection.png) |

---

## 🛠️ Potential Improvements

* ✅ Autonomous red object following
* 🟡 Multi-color & shape detection
* 🟡 Object classification (YOLOv8 / TinyML)
* 🟡 Integrate with SLAM & Navigation2
* 🟡 Use RGB-D data for depth-aware detection

---

## ✅ Tested Environment

| Component | Version / Info             |
| --------- | -------------------------- |
| OS        | Ubuntu 22.04 (WSL2 + WSLg) |
| ROS 2     | Humble Hawksbill           |
| Gazebo    | 11                         |
| GPU       | Intel UHD 620              |
| GUI       | WSLg (no VcXsrv needed)    |

---

## ⚖️ License

This project is licensed under the [MIT License](LICENSE).

```
