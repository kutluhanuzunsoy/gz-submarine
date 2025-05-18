# Quick Startup

> Prerequisites: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-rosdep python3-colcon-common-extensions python3-vcstool \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-teleop-twist-keyboard xterm
```

---

### 2. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

---

### 3. Clone & Build Workspace

```bash
git clone https://github.com/kutluhanuzunsoy/gz-submarine.git
cd gz-submarine

rosdep install --from-paths ros2_ws/src --ignore-src -r -y

cd ros2_ws
colcon build --symlink-install
```

---

### 4. Run the Simulation

```bash
source ros2_ws/install/setup.bash
ros2 launch submarine_bringup bringup.launch.py
```

---
