# Serial autodetection: ESP32 + RPLIDAR (ROS 2 Humble)

The utility `tools/serial_autodetect.py` auto-detects **exactly two** connected
USB-Serial devices without using port numbers, CP2102 IDs, or udev aliases.
It does not edit any existing ROS 2 launch files.

## Safety design

1. Discover two distinct `/dev/ttyUSB*` / `/dev/ttyACM*` ports. With extra
   serial peripherals, use `--ports PORT_A PORT_B` to restrict the candidates.
2. Passively read each port at 115200 baud. Confirm the ESP32 only after
   **two complete** telemetry frames in the same `V:... A:... RC:... TL:...
   TR:... EL:... ER:... B:... T:...` format parsed by
   `src/odom_pub/odom_pub/odom_publisher.py`.
3. **Only after one ESP32 is confirmed**, send `A5 50` (RPLIDAR GET_INFO)
   to the **single remaining candidate**. Require the official response
   descriptor `A5 5A 14 00 00 00 04` followed by 20 bytes.
4. If any step fails or more than one device matches: **exit without launching**.
   The program never guesses from tty indices, and never sends ESP32 velocity
   commands during discovery.
5. **Default: dry-run.** It prints the `ros2 launch mo_hinh real_odom.launch.py`
   command with the detected `esp_port:=...` and `lidar_port:=...` arguments.
   `--launch` explicitly executes that existing hardware launch.

**Physical safety caveat:** Even without sending motor-control packets,
opening/closing CP2102 ports may change DTR/RTS electrically, reset an ESP32,
or affect RPLIDAR A1 motor enable. Immobilize the robot, confirm hardware E-stop,
and ensure no other process already owns either port. No guarantee of zero
physical motor activity can be derived from the no-write condition alone.

## Use on Jetson

```bash
cd ~/ros2_ws
git pull --ff-only origin main
sudo apt install python3-serial

# Offline unit tests (no serial hardware used)
python3 -m unittest discover -s tools -p 'test_serial_autodetect.py' -v

# Identify devices and print suggested launch command; does not launch
python3 tools/serial_autodetect.py

# With other serial devices present, explicitly select exactly two candidates
python3 tools/serial_autodetect.py --ports /dev/ttyUSB0 /dev/ttyUSB1
```

Example successful output (ordering can reverse):

```text
ESP32:   /dev/ttyUSB1 (passive telemetry confirmed)
RPLIDAR: /dev/ttyUSB0 (GET_INFO confirmed, model=..., fw=...)
ROS 2 command:
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp esp_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0 esp_baudrate:=115200 lidar_baudrate:=115200 use_sim_time:=false
DRY RUN: no ROS 2 nodes started; no motor velocity commands sent.
```

**Only after checking the motor firmware watchdog and emergency stop:**

```bash
# Explicit hardware start; odom_pub CAN transmit motor velocity frames.
python3 tools/serial_autodetect.py --launch
```

This starts the **ESP odometry hardware pipeline**, not Nav2 and not the
Hoverboard pathway. The default `real_odom.launch.py` would otherwise use
`odom_source:=hoverboard`.

### Limitations

- Fail closed if the ESP does not stream two valid `V:` frames passively.
  `E,...` telemetry alone is **not** sufficient for the current odometry parser.
- Does not support more than two candidates without explicit `--ports`.
- Requires supported RPLIDAR GET_INFO protocol at selected baudrate.
- Existing `nav2_static_map_bringup.launch.py` has separate older detection logic
  with tty-index fallbacks; this utility does **not** modify it.
- Discovery alone does **not** validate motor safety, LiDAR scans, TF, or Nav2.

Protocol reference: [SLAMTEC RPLIDAR GET_INFO]
(https://wiki.slamtec.com/download/attachments/83066883/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf?api=v2).
