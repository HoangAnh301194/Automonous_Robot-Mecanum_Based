# ESP32 Bridge — Kettle IR (stage 1 / bench build)

**Status: experimental firmware only, not commissioned on the robot.**
One ESP32 remains responsible for both USB-to-hoverboard UART motor transport
and 38 kHz kettle IR transmission via GPIO18. No extra ESP32 required.

## Build selection

| PlatformIO env | Behavior |
|---|---|
| `esp32dev` (default) | Existing unmodified motor bridge/passthrough behavior |
| `esp32dev_kettle` | Experimental validated 8-byte motor/kettle parser + IR task |

Do not flash `esp32dev_kettle` while the wheels are loaded or the kettle can heat/pump.
First compile and validate with wheels lifted / propulsion safely isolated and the IR
LED pointed away from the real kettle.

```bash
cd firmware/esp32_bridge
python -m platformio run -e esp32dev_kettle
# Upload ONLY after isolated bench verification and confirming the serial port:
# python -m platformio run -e esp32dev_kettle -t upload
```

The core parser can be tested without ESP32 hardware:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Isrc \
  tools/test_command_parser.cpp -o /tmp/test_command_parser
/tmp/test_command_parser
```

## Serial command frames (PC -> ESP32)

All fields are **little-endian**, eight bytes total. UART/USB is a stream: reads
may return a partial frame or more than one frame. The ESP32 parser reassembles
frames and rejects invalid checksums. **Never write any kettle bytes directly to
the STM32 UART2 line.**

| Header | Word 1 | Word 2 | Checksum | Destination |
|---|---|---|---|---|
| `0xABCD` | signed steer | signed speed | XOR of three preceding 16-bit words | UART2 → STM32 |
| `0xABCE` | command ID | `0` (reserved) | XOR of three preceding 16-bit words | Internal GPIO18 IR task |

Confirmed by the project owner (not independently decoded by this firmware):
`1=POWER -> 0xD81`, `2=BOIL -> 0xD82`, `3=PUMP -> 0xDC3`, `4=MODE -> 0xD84`.
The library transmits Symphony 12-bit with **zero extra repeats** initially.
If the verified original remote requires repeats, bench-test and configure the
repeat count deliberately; POWER toggle requires special caution.

## Safety behavior (experimental env)

- E-Stop and RC override preserve precedence over PC motor commands.
- Motor watchdog (200 ms) is refreshed **only by valid motor packets**.
- The kettle command is rejected unless the most recent validated motor
  command is zero-steer and zero-speed, a valid motor frame was received
  within the watchdog interval, and at least 1.5 s have elapsed since a
  nonzero motor command. E-Stop or RC override rejects queued requests.
- A single pending IR command is queued; additional commands are not queued.
  Minimum 2 s between accepted kettle commands.
- IR sending is isolated in a low-priority FreeRTOS task on core 0; motor
  forwarding remains in the normal Arduino loop (normally core 1).
- The motor feedback stream from UART2 is kept intact.

### IMPORTANT LIMITATIONS

- These checks only confirm the **last commanded motor setpoint**, not
  zero actual velocity. A ROS-side measured wheel/odometry stationary check
  and a movement interlock are still required.
- IR transmission can finish after an E-Stop if already in progress.
  E-Stop does **not** switch off the kettle or cut mains power.
  Overtemperature/empty-water/overflow protections must be independent.
- The ESP32 intentionally sends **no ACK** for kettle frames in stage 1.
  Injecting unsolicited ACK bytes would corrupt the existing ROS hoverboard
  feedback parser. Add ACK only together with a compatible ROS parser.
- Kettle IR commands are **not yet exposed** via a ROS 2 service or Kiosk UI.
  The existing hoverboard driver owns the USB serial port; a separate process
  must NOT open the same tty for raw writes.
- This firmware has not been hardware-commissioned and must not be considered
  production safe merely because native parser tests pass.

## Acceptance checklist (before ROS 2 integration)

1. Build `esp32dev` and `esp32dev_kettle` successfully.
2. Run native parser tests; inject partial, malformed, interleaved motor/kettle
   frames; verify **no kettle byte reaches UART2**.
3. On bench with wheels isolated, measure motor packet timing, watchdog,
   E-Stop, RC override, motor feedback integrity and USB reconnect.
4. Use logic analyzer or a receiver to measure IR waveform, repetition and
   timings. Do not rely on phone camera LED flicker as proof of decoding.
5. With heater/pump safely disabled, verify IR separately before integrating
   Kiosk and Mission Control.
6. Only after ROS-side serial arbitration and ACK parsing are implemented,
   enable end-to-end operation, with physical emergency cut-off and safeguards.
