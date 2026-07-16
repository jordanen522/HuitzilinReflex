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

## 5. ROS-Gazebo Bridge + pymavlink
```bash
sudo apt install ros-jazzy-ros-gz
sudo pip install pymavlink --break-system-packages
```

## 6. Clone & Build the Project
```bash
git clone <your-repo-url> ~/huitzilin_ws
cd ~/huitzilin_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

Flight-stack bring-up (3 terminals — Gazebo, SITL, launch) and the
arm/takeoff/start_patrol service calls are in `CLAUDE.md` "Build & run". Two rules that
bite (full detail in `CLAUDE.md` sharp edges):

- **Always pass `--add-param-file=…/sitl_frame.parm`** to `sim_vehicle.py` — without it
  a fresh EEPROM has `FRAME_CLASS=0` and the drone arms but never lifts.
- **`--out` ports must match the YAMLs**: `bridge.yaml` listens on `:14552`,
  `patrol.yaml` on `:14553`; MAVProxy's own `:14550` serves `first_flight.py` / QGC.

Optional smoke test: `python3 scripts/first_flight.py` (heartbeat → armed → takeoff →
"Holding position", on `:14550`).

Perception stack (depth world, detector, bag capture — Dell box only):
`docs/week3_capture_runbook.md`.

## Acceptance Criteria
- `ros2 launch huitzilin_sim week2_sitl.launch.py` + the three service calls fly a
  closed patrol loop with logged telemetry (`/huitzilin/odom` publishing throughout).
- Perception: `/oak/points` stable at 15 Hz sim; `run_regression.sh … test` exits 0.
