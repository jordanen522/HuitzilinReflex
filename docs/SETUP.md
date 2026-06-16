# Setup Guide — Project HuitzilinReflex

## Prerequisites
- Windows 11 with WSL2 (Ubuntu 24.04)
- IntelliJ IDEA Ultimate with Remote Development connected to WSL

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

## 6. pymavlink
```bash
sudo pip install pymavlink --break-system-packages
```

## 7. Clone & Build the Project
```bash
git clone <your-repo-url> ~/huitzilin_ws
cd ~/huitzilin_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Terminal 1 — Gazebo (headless)
```bash
gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

### Terminal 2 — SITL
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --out udp:127.0.0.1:14551 \   # Week 1 script (first_flight.py) / QGC
  --out udp:127.0.0.1:14552     # Week 2 ROS 2 bridge — its own port avoids contention
```
Wait for "Received 1363 parameters", then arm:

- **Week 1 (manual):** in the MAVProxy console, `arm throttle force`.
- **Week 2 (scripted):** the `huitzilin_sim` bridge arms programmatically after waiting for EKF/GPS-ready. For unattended runs, relax SITL pre-arm once with `param set ARMING_CHECK 0` rather than a manual force-arm. (Closes the Week 1 "automate the force arm" open question.)

### Terminal 3 — pymavlink flight script
```bash
cd ~/huitzilin_ws
python3 scripts/first_flight.py
```
Expected output: Heartbeat received → Armed → Taking off → Holding position

### Terminal 4 — ROS 2 bridge / Week 2 stack
```bash
source /opt/ros/jazzy/setup.bash
source ~/huitzilin_ws/install/setup.bash   # repo root = colcon workspace
# Week 2 bridge (supersedes the Week 1 `mavlink_bridge` node), on its own port:
ros2 run huitzilin_sim mav_bridge --ros-args -p connection:=udp:127.0.0.1:14552
# …or bring up the whole Week 2 stack (bridge + patrol + telemetry logger):
ros2 launch huitzilin_sim week2_sitl.launch.py
```

## Acceptance Criteria
- `first_flight.py` prints "Holding position"
- QGroundControl shows "Flying" and "Guided"
- **Week 2:** `ros2 run huitzilin_sim mav_bridge` connects and publishes `/huitzilin/odom`; `ros2 topic pub /huitzilin/cmd_vel …` moves the drone; `ros2 launch huitzilin_sim week2_sitl.launch.py` flies a closed patrol loop with logged telemetry