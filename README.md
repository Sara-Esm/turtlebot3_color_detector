# 🧠 TurtleBot3 Color Detector in Custom House World (ROS 2 Humble + Gazebo)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/) [![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/) [![Gazebo 11](https://img.shields.io/badge/Gazebo-11-brightgreen)](https://gazebosim.org/) [![WSL2 + WSLg](https://img.shields.io/badge/WSL2-WSLg-lightgrey)](https://learn.microsoft.com/windows/wsl/)

A real-time robotics & computer vision project: a TurtleBot3 navigates a custom indoor house environment, detecting red objects via camera and OpenCV. Built using ROS 2 Humble on Ubuntu 22.04 (WSL2 + WSLg, Intel UHD 620).

---

## 🚀 Why This Project?

I developed this to deepen my skills in robotics and CV using ROS 2. It showcases real-time visual detection, manual control, and 3D simulation.

---

## 🗂️ Repository Structure

```text
turtlebot3_color_detector_ws/
├── src/
│   ├── turtlebot3_gazebo/              ← Official simulation package
│   │   └── worlds/turtlebot3_house.world  ← Custom indoor Gazebo world
│   └── turtlebot3_color_detector/      ← Custom package
│       ├── color_detector/
│       │   └── red_detector.py         ← Vision node: red color detection
│       ├── launch/
│       │   ├── house_world.launch.py   ← Gazebo + RViz2 + CV
│       │   └── color_node.launch.py    ← Only the CV node
├── README.md
````

---

## 🧪 What You’ll See

![Gazebo & RViz Demo](docs/demo_gazebo_rviz.png)

1. **TurtleBot3 roams** around a house-like Gazebo world
2. **Camera stream** shown in RViz2 or `rqt_image_view`
3. **Red object detection** via HSV color filter (console logs highlight centroid)

```bash
[INFO] Red object detected at x=325 → CENTER
```

---

## ⚙️ Setup & Build Instructions

**1. Install dependencies**

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3* \
  ros-humble-rqt-image-view \
  ros-humble-cv-bridge \
  python3-opencv
```

**2. Set TurtleBot3 model**

```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

**3. Build the workspace**

```bash
cd ~/projects/turtlebot3_color_detector_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🧭 Run the Project

### 🔄 Full Simulation (Gazebo + RViz + CV)

```bash
ros2 launch turtlebot3_color_detector house_world.launch.py
```

### 🎮 Manual Teleop Control

In a new terminal:

```bash
source ~/projects/turtlebot3_color_detector_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use `w/a/s/d/x` keys to move!

### 👁️ Live Camera View

```bash
rqt_image_view /camera/image_raw
```

### 👁️ Vision Node Only

If Gazebo already running:

```bash
ros2 launch turtlebot3_color_detector color_node.launch.py
```

---

## 🧠 How Red Detection Works

* **Subscribes** to `/camera/image_raw`
* **Converts** images via `cv_bridge`
* **Applies** HSV thresholds to isolate red
* **Finds** and highlights the biggest red blob
* **Logs** centroid position: LEFT/CENTER/RIGHT

🔗 Check the code: [red\_detector.py](https://github.com/Sara-Esm/turtlebot3_color_detector/blob/main/src/turtlebot3_color_detector/color_detector/red_detector.py)

---

## 🔧 Future Enhancements

* Autonomous following of red objects using blob centroid
* Multi-color or shape-based detection (Green, blue…)
* Integrate ROS Navigation2 for goal-based movement
* Depth estimation via depth camera
* Advanced object detection: YOLO‑Tiny or TensorFlow Lite

---

## ✅ Environment Compatibility

| Component | Version/Tested             |
| --------- | -------------------------- |
| OS        | Ubuntu 22.04 (WSL2 + WSLg) |
| ROS 2     | Humble Hawksbill           |
| Gazebo    | 11                         |
| GPU       | Intel UHD 620              |
| GUI       | WSLg (no VcXsrv needed)    |

---

## 🙋‍♀️ About Me

**Sara Esmaeili**
Electrical & Control Engineer | Robotics & AI Enthusiast

* 🔗 GitHub: [Sara‑Esm](https://github.com/Sara-Esm)
* 💼 LinkedIn: [linkedin.com/in/sara‑esmaeili‑](https://www.linkedin.com/in/sara-esmaeili-/)

---

## ⚖️ License

This project is shared under the [MIT License](LICENSE).

