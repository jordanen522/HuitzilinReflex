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
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out udp:127.0.0.1:14551
```
Wait for "Received 1363 parameters" then force arm in the MAVProxy console:

arm throttle force

### Terminal 3 — pymavlink flight script
```bash
cd ~/huitzilin_ws
python3 scripts/first_flight.py
```
Expected output: Heartbeat received → Armed → Taking off → Holding position

### Terminal 4 — ROS 2 MAVLink bridge (optional)
```bash
source /opt/ros/jazzy/setup.bash
source ~/huitzilin_ws/install/setup.bash
ros2 run mavlink_bridge mavlink_bridge
```

## Acceptance Criteria
- `first_flight.py` prints "Holding position"
- QGroundControl shows "Flying" and "Guided"
- `ros2 run mavlink_bridge mavlink_bridge` prints "MAVLink heartbeat received"