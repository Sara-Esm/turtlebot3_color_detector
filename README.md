## 🧠 TurtleBot3 Color Detector in Custom House World (ROS 2 Humble + Gazebo)

This ROS 2 Humble project simulates a TurtleBot3 robot navigating in a custom **house world** using Gazebo 11 under **WSL2 with WSLg** on Ubuntu 22.04. The robot uses a **real-time computer vision node** to detect red objects through its camera feed and enables basic interactive control with teleop and visualization tools like **RViz2** and **rqt\_image\_view**.

---

### 🗂️ Project Structure

```
turtlebot3_color_detector_ws/
├── src/
│   ├── turtlebot3_gazebo/              ← simulation package (includes custom world)
│   │   ├── worlds/
│   │   │   └── turtlebot3_house.world  ← Custom indoor Gazebo environment
│   └── turtlebot3_color_detector/      ← custom package with CV and launch files
│       ├── color_detector/             ← Python node to detect red colour
│       ├── launch/
│       │   ├── house_world.launch.py   ← launches Gazebo + RViz + CV node
│       │   └── color_node.launch.py    ← launches only the CV node
```

---

### ⚙️ Prerequisites

Install the required ROS 2 and TurtleBot3 packages (if not done already):

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* \
                 ros-humble-rqt-image-view \
                 ros-humble-cv-bridge \
                 python3-opencv
```

Set TurtleBot3 model (you can put this in `~/.bashrc` too):

```bash
export TURTLEBOT3_MODEL=burger
```

---

### 🔧 Build the Workspace

```bash
cd ~/projects/turtlebot3_color_detector_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 🚀 Launch the Full Simulation

**Gazebo + RViz2 + Color Detector Node**

```bash
ros2 launch turtlebot3_color_detector house_world.launch.py
```

This launches:

* Your custom **house world** in Gazebo
* A **TurtleBot3 robot** inside the house
* The **color detector node** using the robot's camera
* **RViz2** for real-time 3D visualization

---

### 👀 View the Camera Feed

In a separate terminal:

```bash
rqt_image_view /camera/image_raw
```

This lets you visualize what the robot sees.

---

### 🎮 Manual Control

You can control the robot manually via keyboard teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use arrow keys to drive around and test how your red object appears in the camera.

---

### 🤖 Run Only the Color Detector Node

If Gazebo is already running:

```bash
ros2 launch turtlebot3_color_detector color_node.launch.py
```

---

### 📁 Custom World Location

Your custom house world is saved at:

```
src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_house.world
```

You can modify or expand the environment using `.sdf` files or Gazebo UI.

---

### 📸 About the Color Detection

The custom Python node in `color_detector/red_detector.py`:

* Subscribes to `/camera/image_raw`
* Converts image from ROS to OpenCV
* Detects red regions using HSV thresholds
* Optionally publishes detection info (e.g., centroid) — extendable to motion control

---

### ✅ System Compatibility

This project runs smoothly under:

* ✅ **ROS 2 Humble**
* ✅ **Gazebo 11**
* ✅ **WSL2 with WSLg** (no VcXsrv needed)
* ✅ **Intel UHD 620 GPU**
* ✅ **Ubuntu 22.04**

---

### 🔜 Future Upgrades (Ideas)

* Automatically follow red objects
* Add navigation goals using detected color blobs
* Extend detection to multiple colors or shapes
* Use YOLO or TensorFlow Lite for more advanced detection


