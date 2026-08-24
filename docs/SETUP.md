# Setup Guide — Project HuitzilinReflex

## Prerequisites
- Ubuntu 24.04 — native (Dell box; required for depth rendering) or WSL2 (flight/SITL only)

## 1. ROS 2 Jazzy
```bash
sudo apt install ros-jazzy-desktop ros-dev-tools
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 2. ArduPilot & SITL
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
# Restart shell after this
./waf configure --board sitl
./waf copter
```

## 3. Gazebo Harmonic
```bash
sudo apt install gz-harmonic
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## 4. ArduPilot-Gazebo Plugin
```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

## 5. ROS-Gazebo Bridge
```bash
sudo apt install ros-jazzy-ros-gz
```

The Python dependencies are **not** listed here on purpose — `rosdep` reads them
from the two `package.xml` files in step 6, which is the copy that stays correct
when a dependency is added.

## 6. Clone & Build the Project
```bash
git clone <your-repo-url> ~/huitzilin_ws
cd ~/huitzilin_ws
source /opt/ros/jazzy/setup.bash

# Not optional. Both packages import pymavlink, numpy, scipy and yaml, and
# huitzilin_perception publishes TF directly. Skipping this builds cleanly and
# then ImportErrors at runtime, which reads as a code bug rather than a
# missing package.
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## 7. Check the build
```bash
./scripts/run_tests.sh          # whole unit suite, both packages
./scripts/preflight_check.sh    # SITL environment, 4 checks
```

## Running the Simulation

Bring-up commands, the service calls, and the traps that bite (`sitl_frame.parm`,
`--out` port fan-out) live in `CLAUDE.md` — read its sharp edges before the first run.

The full documentation index is the Documentation table in `README.md`.

## Acceptance Criteria
- `ros2 launch huitzilin_sim week2_sitl.launch.py` + the three service calls fly a
  closed patrol loop with logged telemetry (`/huitzilin/odom` publishing throughout).
- Perception: `/oak/points` stable at 15 Hz sim; `run_regression.sh … test` exits 0.
