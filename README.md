# 🤖 TurtleBot3 Semi-Autonomous Color Follower

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4-red)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

> A perception-triggered autonomous behaviour system for TurtleBot3. The robot continuously processes its camera stream and autonomously switches into object-following mode when a red target enters its field of view — no human input required once the target is detected.

---

## 📽️ Demo

*

---

## ✨ Features

- **Perception-triggered autonomy** — robot switches from manual to autonomous mode on object detection
- **Real-time HSV colour filtering** — OpenCV detects red objects with tuned threshold masks
- **Centroid tracking** — calculates object center and aligns robot heading dynamically
- **Reactive velocity control** — publishes `/cmd_vel` commands based on object position in frame
- **Live debug visualization** — rqt_image_view shows processed camera stream with detection overlay

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────┐
│              Gazebo Simulation                  │
│  ┌──────────────────┐   ┌─────────────────────┐ │
│  │  TurtleBot3      │   │   Camera Sensor     │ │
│  │  waffle_pi       │   │  /camera/image_raw  │ │
│  └────────┬─────────┘   └──────────┬──────────┘ │
└───────────┼──────────────────────  ┼────────────┘
            │                        │
            │           ┌────────────▼───────────┐
            │           │    color_detector.py   │
            │           │                        │
            │           │  cv_bridge conversion  │
            │           │  BGR → HSV filtering   │
            │           │  contour detection     │
            │           │  centroid calculation  │
            │           └────────────┬───────────┘
            │                        │
            └────────────────────────▼
                     /cmd_vel
              (geometry_msgs/Twist)
```

### ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Robot camera feed |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to robot |

---

## 📦 Package Structure

```
turtlebot3_color_follower/
└── src/
    └── turtlebot3_color_follower/
        ├── launch/
        │   └── color_detector.launch.py
        ├── turtlebot3_color_follower/
        │   └── color_detector.py      # Main perception + control node
        ├── package.xml
        └── setup.py
```

---

## 🤖 Robot Behavior

The robot follows a perception-control loop:

```
[MANUAL MODE] Teleoperate through environment
      │
      ▼
[PERCEPTION] Camera stream → HSV filtering → contour detection
      │
      ├── No red object → continue manual mode
      │
      └── Red object detected
            │
            ▼
      [AUTO MODE] Calculate centroid → align heading → move forward
            │
            └── Object lost → stop and return to manual mode
```

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Robot OS | ROS 2 Humble Hawksbill |
| Computer Vision | OpenCV 4, HSV color filtering |
| Robot Model | TurtleBot3 Waffle Pi |
| Simulation | Gazebo Classic 11 |
| Language | Python 3 |
| Platform | Ubuntu 22.04 / WSL2 |

---

## 🚀 Getting Started

### Prerequisites

```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install python3-opencv ros-humble-cv-bridge
```

### Build

```bash
cd ~/projects/turtlebot3_color_follower
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Run

**Terminal 1 — Gazebo simulation:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2 — Color follower node:**
```bash
source install/setup.bash
ros2 launch turtlebot3_color_follower color_detector.launch.py
```

**Terminal 3 — Manual teleoperation:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 4 — Camera feed (optional):**
```bash
rqt_image_view /camera/image_raw
```

---

## 🔑 Key Implementation Details

**Color Detection** (`color_detector.py`)
- Converts camera frames from BGR to HSV color space for lighting-robust detection
- Applies tuned red HSV threshold mask to isolate target object
- Finds contours and selects largest valid object to filter noise
- Calculates centroid from bounding moments for precise alignment

**Reactive Control**
- Compares centroid x-position to image center to determine rotation direction
- Publishes angular velocity to `/cmd_vel` to align robot heading with object
- Switches to forward linear velocity once object is centered in frame
- Stops all motion when object is no longer detected

---

## 🔭 Future Work

- [ ] Multi-color target support with dynamic color selection
- [ ] Distance estimation from object bounding box size
- [ ] Fully autonomous room exploration with color search
- [ ] PID controller for smoother tracking behavior
- [ ] 3D target localization using depth camera

---

## 👩‍💻 Author

**Sara Esmaeili** — Robotics Software Engineer
GitHub: [@Sara-Esm](https://github.com/Sara-Esm)

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.
