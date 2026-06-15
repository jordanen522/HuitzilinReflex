# Project Journal — HuitzilinReflex

## Week 1 — 2026-06-15

### What was accomplished
- Completed all design docs: requirements, node graph, message contracts, state machine, frames, safety case
- Installed and verified: ArduPilot SITL, Gazebo Harmonic, ardupilot_gazebo plugin, ros_gz bridge, QGroundControl
- First scripted flight via pymavlink: arm → takeoff → hold position confirmed in QGC
- ROS 2 mavlink_bridge node built and verified receiving /cmd/evade commands

### What worked
- WSL2 + IntelliJ Remote Development workflow solid for coding
- SITL + Gazebo headless connection stable
- pymavlink connecting via udp:127.0.0.1:14551

### What didn't work / workarounds
- Gazebo GUI runs at ~24% real-time due to Intel Iris Xe GPU not supported in WSL2 — running headless as workaround
- ROS 2 node needed pymavlink installed to system Python (`sudo pip install pymavlink --break-system-packages`)
- arm throttle force needed manually in MAVProxy before script can arm

### Open questions for Week 2
- NED/ENU conversion handling in the bridge
- Command stream rate for position targets
- How to automate the force arm for scripted testing

### Versions
- ROS 2: Jazzy
- Gazebo: Harmonic 8.11.0
- ArduPilot: latest main
- Python: 3.12.3