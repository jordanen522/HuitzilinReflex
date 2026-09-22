# Hardware bring-up

Owns: turning the purchased BOM into a bench-tested aircraft and companion stack.
Everything here is **props off** until the last bench stage says otherwise. Nothing in this
repo has run on the real hardware yet, so no hardware number exists and none may be
quoted (`docs/RESULTS.md` covers simulation only).

Target: Raspberry Pi 5 on **Ubuntu 24.04 + ROS 2 Jazzy** (the same stack as the Dell),
MicoAir H743 V2 on ArduCopter 4.5+, OAK-D Lite, WS2812B strip and piezo siren.

```
 H743 ──USB──▶ mavlink-router ──UDP 14552──▶ mav_bridge ──▶ /huitzilin/odom, /state
  (ArduPilot)      (Pi)       ──UDP 14553──▶ patrol (position setpoints)
                              ──UDP 14554──▶ preflight_hw.sh / hw_param_readback.py
                              ──UDP 14550──▶ QGroundControl over Wi-Fi
 OAK-D ──USB3──▶ depthai-ros ──/oak/points──▶ detector ──▶ evasion
 Pi GPIO: SPI0 MOSI (GPIO10) ─▶ level shifter ─▶ WS2812B    GPIO17 ─▶ NPN ─▶ siren
```

The router is why the sim configs carry over: it plays the part MAVProxy's `--out` plays in
SITL, on the same ports, so `bridge.yaml` and `patrol.yaml` are unchanged on the aircraft.

## Blocker before any flight: a position source

The BOM has no GPS and no optical flow. ArduPilot needs a position estimate for GUIDED
position control, the circular fence, LOITER and RTL, so **the aircraft can be bench-tested
but not flown autonomously** until one is fitted: GPS (outdoor) or an optical-flow +
rangefinder module (indoor / netted enclosure). The bench stages below work without it
(`/huitzilin/state` publishes armed, mode and battery with no fix). Once a source is chosen,
its parameters go in `hw_frame.parm` with a test in `test_hw_config.py`, as the fence did.

## 1. Airframe (battery disconnected)

- Photograph the stock GEP F722 AIO wiring first: 4 motors (3 phases each), battery leads,
  HD-VTX/camera plug, ELRS receiver plug. It is the only reference once it is off.
- Swap to the MicoAir H743 on the same 25.5 mm standoffs, same motor corner mapping. Check
  every joint for shorts across the ESC pads. Worth outsourcing to an FPV shop if
  fine-pitch soldering is not comfortable.
- Flash ArduCopter (H7 target) over the H743's USB-C.
- Load `src/huitzilin_sim/params/hw_frame.parm` (frame, fence, failsafes, kill switch):
  `param load hw_frame.parm` in MAVProxy, or the parameter file loader in QGroundControl.
- Motor test in QGroundControl, props off: order and direction for quad-X. Fix a backward
  motor by swapping two phase wires, not in firmware.
- Accelerometer six-position calibration. Set the RC input to ELRS (CRSF) and bind the
  RadioMaster Pocket. The kill switch is `RC7_OPTION 31` on its own momentary switch.

## 2. Pi power

- Pololu D24V50F5 input from the 6S battery rail, never the FC's rail. Output to the Pi's
  5 V/GND GPIO pins. Measure 5.0-5.2 V on the regulator before connecting the Pi.
- Bench work without the battery: keep using the 27 W USB-C supply.

## 3. Pi software (once)

Ubuntu Server 24.04 (64-bit) for Raspberry Pi, then ROS 2 Jazzy:

```bash
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-ros-base ros-jazzy-depthai-ros \
  python3-colcon-common-extensions python3-rosdep \
  python3-libgpiod gpiod python3-spidev libraspberrypi-bin
sudo rosdep init && rosdep update
```

Workspace. The Gazebo bridge packages are simulation-only; skip them on the Pi:

```bash
mkdir -p ~/huitzilin_ws && cd ~/huitzilin_ws
git clone https://github.com/jordanen522/HuitzilinReflex.git .
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "ros_gz_image ros_gz_bridge ros_gz_interfaces"
colcon build --symlink-install && source install/setup.bash
```

Boot config: add these two lines to `/boot/firmware/config.txt`, then reboot. The first stops
the Pi throttling its USB ports on GPIO power (the OAK-D starves); the second enables SPI
for the LED strip.

```
usb_max_current_enable=1
dtparam=spi=on
```

Device access: the OAK-D udev rule, and your user in the groups that own the serial port,
SPI and GPIO devices (`ls -l /dev/ttyACM0 /dev/spidev0.0 /dev/gpiochip*` shows which):

```bash
sudo cp ~/huitzilin_ws/hardware/99-huitzilin.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER     # plus the spidev/gpiochip owner groups; log in again
```

mavlink-router, built from source (it is not packaged for Ubuntu):

```bash
sudo apt install -y meson ninja-build pkg-config gcc g++ systemd
git clone https://github.com/mavlink-router/mavlink-router.git ~/mavlink-router
cd ~/mavlink-router && git submodule update --init --recursive
meson setup build . && ninja -C build && sudo ninja -C build install
sudo mkdir -p /etc/mavlink-router
sudo cp ~/huitzilin_ws/hardware/mavlink-router.conf /etc/mavlink-router/main.conf
ls /dev/serial/by-id/      # put the ArduPilot id in main.conf, replacing CHANGE-ME
sudo systemctl enable --now mavlink-router
```

## 4. Preflight

```bash
cd ~/huitzilin_ws && ./scripts/preflight_hw.sh
```

A checklist, not a gate: it exits 0 and counts warnings, covering the workspace, the FC's
by-id path and router config, a heartbeat through the router, a parameter readback against
`hw_frame.parm`, the OAK-D on a 5000M USB-3 link, throttling and `config.txt`, disk,
SPI/GPIO access and the kill-switch channel (manual). Parameter readback on its own:

```bash
./scripts/hw_param_readback.py
```

## 5. Bench stages (props off)

Each stage is one command added to the last. `hardware.launch.py` pins `use_sim_time`
false and loads each node's sim yaml with its `hw_*` overlay on top.

**Stage 1 — FC link.** Bridge, supervisor, payload, telemetry log.

```bash
ros2 launch huitzilin_perception hardware.launch.py
ros2 topic echo /huitzilin/state --full-length
```

Expect `armed`, `mode`, `batt_v` and a small `fc_age_s`. `alt` and odom stay absent until
the FC has a position estimate. Unplug the FC USB: `fc_age_s` climbs and odom (if present)
stops within 1 s. That is the supervisor's LINK_LOSS input.

**Stage 2 — parameters.** `./scripts/hw_param_readback.py` reports every `hw_frame.parm`
value matching. Load the file on any mismatch rather than fixing values by hand.

**Stage 3 — RC and kill switch.** Flip the kill switch and watch channel 7 move in
QGroundControl's radio page. Pull the receiver's power: the RC failsafe must fire
(`FS_THR_ENABLE`).

**Stage 4 — payload.** Disarmed, so the supervisor ignores the alarm:

```bash
ros2 topic pub --once /payload/alarm std_msgs/msg/Bool '{data: true}'
```

The strip goes orange and the siren sounds for at least `min_on_s`, clearing by `stale_off_s`.
If the node logs `payload degraded`, the reason names the missing piece (the strip needs the
3.3 -> 5 V level shifter on its data line, which is not yet purchased).

**Stage 5 — camera and detector.**

```bash
ros2 launch huitzilin_perception hardware.launch.py with_camera:=true
ros2 topic hz /oak/points
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame
```

The depthai-ros launch-argument and frame names in `hardware.launch.py` are the driver's
documented ones and have not yet been checked on this Pi: confirm with
`ros2 launch depthai_ros_driver camera.launch.py --show-args` and fix the include if they
differ. Record the **delivered** point-cloud rate and resolution. The detector's
`min_track_updates` and the supervisor's `sensor_timeout_s` are timed against 15 Hz from sim,
and a sensor is reach, sector and rate (`CLAUDE.md`), so this measurement decides what the
real envelope is. Measure the camera mount against `docs/frames.md` and pass it as
`camera_x:=` / `camera_z:=`.

**Stage 6 — full stack armed, props off.**

```bash
ros2 launch huitzilin_perception hardware.launch.py with_camera:=true with_evasion:=true
```

Arm from the transmitter. The supervisor logs `DISARMED -> ARMING -> TAKEOFF`, and TAKEOFF
waits for an altitude it cannot reach on the bench. Then, one at a time:

- unplug the OAK-D USB: SENSOR_DROPOUT -> FAILSAFE within `sensor_timeout_s`;
- stop the launch: the FC's GCS failsafe (`FS_GCS_ENABLE`) fires once the bridge heartbeat
  stops (disconnect QGroundControl first; it heartbeats as the same GCS system id);
- flip the kill switch: motors stop immediately.

Without a position source, the LOITER and RTL the supervisor requests are refused by the FC
and it lands instead. That is expected on this bench and is why flight waits on the blocker
above.

The supervisor changes flight mode as soon as the FC reports armed. **For a manual RC
flight, launch with `with_supervisor:=false`.**

## 6. After the bench

Held-down static throttle-up with props on, then HITL, tethered hover, and flight inside a
netted enclosure with soft projectiles (`HuitzilinReflex_v2.md` §5). Before any outdoor
flight: FAA registration, Remote ID verified with a receiver app, and the site checked
against controlled airspace (`docs/SAFETY_CASE.md`).
