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
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14552 \   # Week 2 ROS 2 bridge   (matches bridge.yaml)
  --out udp:127.0.0.1:14553     # Week 2 patrol node    (matches patrol.yaml)
```
Wait for "Received 1363 parameters".

**Port mapping (must match the YAMLs):** MAVProxy fans the MAVLink stream out to
each consumer's own UDP port. `bridge.yaml` listens on `udpin:0.0.0.0:14552`,
`patrol.yaml` on `udpin:0.0.0.0:14553`. The `--out` ports above must match, or the
nodes get `TimeoutError: no heartbeat`. (MAVProxy's own default `--out …:14550`
still carries the Week 1 `first_flight.py` / QGC link.)

**Airframe (critical):** `--add-param-file=…/sitl_frame.parm` sets
`FRAME_CLASS=1` (Quad) / `FRAME_TYPE=1` (X) on every boot. Without it, a fresh
EEPROM boots with `FRAME_CLASS=0` — arming "succeeds" but the motors never spin
as a quad (throttle maxes out, zero lift, `PreArm: Motors: Check frame class and
type`). This was the actual takeoff blocker; do **not** paper over it with
`ARMING_CHECK 0` — that only hides the pre-arm message while the airframe stays
unmixed. If you ever set the frame by hand instead: `param set FRAME_CLASS 1`,
`param set FRAME_TYPE 1`, then `reboot` (the FC link drops for a few seconds and
reconnects — that's normal).

Then arm:

- **Week 1 (manual):** in the MAVProxy console, `arm throttle`.
- **Week 2 (scripted):** the `huitzilin_sim` bridge arms programmatically via the
  `/huitzilin/arm` service after waiting for EKF/GPS-ready.

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