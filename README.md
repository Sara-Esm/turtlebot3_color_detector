## 🧠 TurtleBot3 Color Detector in Custom House World (ROS 2 Humble + Gazebo)

A real-time **robot perception and simulation project** using a TurtleBot3 robot in a custom indoor Gazebo world.
Built with **ROS 2 Humble**, this system features a **color detection vision node**, manual teleoperation, and full 3D visualization with **RViz2**, all running natively on **WSL2 with WSLg (Ubuntu 22.04)**.

---

## 🗂️ Project Structure

```
turtlebot3_color_detector_ws/
├── src/
│   ├── turtlebot3_gazebo/              ← official simulation package (custom world inside)
│   │   └── worlds/turtlebot3_house.world
│   └── turtlebot3_color_detector/      ← custom vision & launch package
│       ├── color_detector/             ← red color detection node (OpenCV + ROS 2)
│       ├── launch/
│       │   ├── house_world.launch.py   ← Gazebo + RViz2 + CV
│       │   └── color_node.launch.py    ← vision node only
├── README.md                          
```

---

## ⚙️ Setup Instructions

### 📦 Install Required Packages

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* \
                 ros-humble-rqt-image-view \
                 ros-humble-cv-bridge \
                 python3-opencv
```

### 🧠 Set TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

### 🔨 Build the Workspace

```bash
cd ~/projects/turtlebot3_color_detector_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🧪 How to Run the Project

### 🏠 Launch Full Simulation: World + Robot + Camera + RViz

```bash
ros2 launch turtlebot3_color_detector house_world.launch.py
```

This will:

* Launch Gazebo 11 with a **custom indoor house world**
* Spawn the **TurtleBot3 burger** robot
* Activate the **real-time red object detector node**
* Start **RViz2** for full 3D monitoring

---

### 🎮 Drive the Robot Manually

In a new terminal:

```bash
source ~/projects/turtlebot3_color_detector_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use arrow keys to move the robot toward colored objects.

---

### 👁️ View Live Camera Feed

In another terminal:

```bash
rqt_image_view /camera/image_raw
```

---

### 🧪 Run Vision Node Separately

If you already have the simulation running:

```bash
ros2 launch turtlebot3_color_detector color_node.launch.py
```

---

## 🧠 How Red Color Detection Works

The Python node in `color_detector/red_detector.py`:

* Subscribes to `/camera/image_raw`
* Uses **OpenCV** + **cv\_bridge** to convert ROS images
* Applies **HSV filtering** to detect red objects
* Highlights the red region in the frame

---

## 🖼️ Custom World Info

You can find and edit the world file here:

```bash
src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_house.world
```

To customize:

```bash
gazebo src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_house.world
```

---

## 🔍 Future Improvements

* Autonomous red object following using centroid position
* Use [YOLOv8](https://github.com/ultralytics/ultralytics) or [TinyML](https://www.edgeimpulse.com/) models for advanced detection
* Simulate pick-and-place with robot arm based on object color
* Integrate with SLAM (Cartographer) and Navigation2

---

## ✅ System Compatibility

| Component  | Tested Version                 |
| ---------- | ------------------------------ |
| **OS**     | Ubuntu 22.04 (WSL2)            |
| **ROS 2**  | Humble Hawksbill               |
| **Gazebo** | Gazebo 11 (gzclient, gzserver) |
| **GPU**    | Intel UHD 620                  |
| **GUI**    | WSLg (no VcXsrv needed)        |

