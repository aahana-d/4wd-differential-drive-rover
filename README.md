# Differential-Drive 4WD Robotic Rover — Firmware & Validation Toolkit

Closed-loop, encoder-fed, PID-controlled 4WD differential-drive rover with a packetized Bluetooth control link, communication-loss failsafe, hardware watchdog, and battery-voltage feed-forward compensation. This package contains the embedded firmware, a host-side telemetry logger with a SQL schema for validation, and native (hardware-free) unit tests.

All logic-bearing code in this package has been compiled and executed as part of writing it (native unit tests pass on real Unity; the SQL schema and views have been exercised against real SQLite; the host logger has been syntax-checked against the real sqlite3 C API surface). The Arduino/ESP32-dependent files (Encoder, MotorDriver, BluetoothTransport, Watchdog, Rover) cannot be compiled without the target hardware/toolchain present, so they are validated per the bench-test procedure below instead.

---

## 1. Rover hardware structure

**Chassis / drivetrain**
- 4 independent DC gear-motors, one per wheel (front-left, rear-left, front-right, rear-right), each fitted with a quadrature encoder on the motor shaft (behind the gearbox).
- Differential-drive kinematics: the two **left** wheels always share one commanded angular velocity, the two **right** wheels share the other. Steering comes purely from the left/right speed difference — there is no separate steering mechanism.
- Wheels are controlled *independently* at the PID level (4 separate PID loops, one per encoder), rather than physically ganging left/right pairs onto a single motor driver channel. This is what lets the firmware compensate for small manufacturing differences between the 4 motors instead of assuming both wheels on a side are identical.

**Electronics**
- **MCU:** ESP32 dev board (Arduino framework via PlatformIO). Chosen for built-in classic Bluetooth (SPP), enough PWM/interrupt-capable GPIOs for 4 motors + 4 encoders, and a hardware task watchdog.
- **Motor drivers:** 2x TB6612FNG dual H-bridge breakout boards (2 channels each = 4 channels total). Each channel: 1 PWM pin + 2 direction pins.
- **Encoders:** quadrature, 2 signal pins each (A/B), wired to interrupt-capable GPIOs. 1x decode (interrupt on the A channel's rising  edge, direction read from B) is used for simplicity; see `Encoder.h` for notes on upgrading to full 4x decode or the ESP32 PCNT peripheral.
- **Power:** 3S LiPo (11.1V nominal) → buck regulator → 5V logic rail for the ESP32/drivers' logic side; battery voltage is also fed through a resistor divider into an ADC pin so firmware can measure it directly.
- **Bluetooth:** ESP32's built-in classic Bluetooth SPP, advertised as `ROVER-4WD-01` (see `config.h`). Appears as a standard serial port to a paired host (e.g. `/dev/rfcomm0` on Linux), which is exactly what `basestation/telemetry_logger.cpp` connects to.

**Why voltage-drop causes drift, and how this design fixes it:**
As the LiPo discharges under motor load, its terminal voltage sags. A pure PID loop *will* eventually correct for this (the encoder shows the wheel is going slower than commanded, the integral term winds up, PWM duty increases), but only *after* accumulating error, which shows up as a visible, asymmetric lag/pull to one side every time you change throttle,
worse on whichever wheel's motor/driver happens to have slightly higher resistance. `BatteryMonitor` measures pack voltage directly and produces a feed-forward multiplier applied *on top of* each wheel's PID output, so the effective voltage delivered to the motor stays close to what it would be at a full battery — the PID then only has to correct for genuine mechanical differences between wheels, not for pack state, and does so faster.

---

## 2. Repository layout

```
rover_project/
├── README.md                     
├── common/                        <- platform-independent, shared by firmware AND basestation
│   ├── Protocol.h / Protocol.cpp     packet framing, byte-stuffing, CRC-8
│   └── Kinematics.h / Kinematics.cpp differential-drive inverse kinematics
│
├── firmware/                      <- PlatformIO project, runs on the ESP32
│   ├── platformio.ini                 esp32dev (real firmware) + native (host unit tests) envs
│   ├── lib/common -> ../../common     symlink so PlatformIO auto-compiles the shared code
│   ├── include/
│   │   ├── config.h                   ALL pins, physical constants, tuning gains, timing
│   │   ├── PIDController.h            generic PID, header-only, no Arduino dependency
│   │   ├── Encoder.h                  quadrature encoder reader (ISR-driven)
│   │   ├── MotorDriver.h              single TB6612FNG channel wrapper
│   │   ├── DriveController.h          owns all 4 wheels; kinematics + PID + voltage comp
│   │   ├── BatteryMonitor.h           ADC read, filtering, compensation factor
│   │   ├── BluetoothTransport.h       thin wrapper over BluetoothSerial
│   │   ├── CommandProcessor.h         packet validation, dispatch, comm-loss tracking
│   │   ├── Watchdog.h                 hardware task-watchdog wrapper
│   │   └── Rover.h                    top-level orchestrator / scheduler
│   ├── src/                           .cpp implementations matching the headers above, plus main.cpp
│   └── test/test_native/              host-run Unity tests (no hardware needed)
│       ├── test_protocol.cpp          framing/CRC/escaping/resync
│       ├── test_pid.cpp               PID convergence, saturation, reset, bad-dt handling
│       └── test_kinematics.cpp        differential-drive math
│
└── basestation/                   <- host-side companion tool (any Linux/macOS PC)
    ├── schema.sql                     SQLite schema + validation views
    ├── telemetry_logger.cpp           reads the SAME protocol off a serial/BT port, logs to SQLite
    └── CMakeLists.txt                 builds telemetry_logger against libsqlite3
```

**Why `common/` exists:**
The packet-framing code and the differential-drive math are pure algorithms with no hardware dependency. Sharing one implementation between the firmware and the host logger means the logger can never silently drift out of sync with what the firmware actually transmits, and both the protocol and the kinematics can be unit-tested on a laptop in milliseconds instead of requiring the physical rover.

---

## 3. Control & safety architecture

```
Bluetooth app on phone/PC
        │  SET_VELOCITY(linear, angular), STOP, HEARTBEAT, SET_PID_GAINS, ...
        ▼
CommandProcessor  ── validates length & range, tracks lastValidPacketMs ──► ACK / NACK
        │ setVelocityCommand(v, w)
        ▼
DriveController
  differentialDriveTargets(v, w) ──► leftTarget, rightTarget (rad/s)
        │
        ├─ Wheel FL: PID(target=left,  actual=encoderFL) ─┐
        ├─ Wheel RL: PID(target=left,  actual=encoderRL) ─┤ x compensationFactor ──► MotorDriver
        ├─ Wheel FR: PID(target=right, actual=encoderFR) ─┤   (from BatteryMonitor)
        └─ Wheel RR: PID(target=right, actual=encoderRR) ─┘

Rover::update() every loop() iteration:
  1. commandProcessor_.poll()            — drain/parse Bluetooth bytes
  2. comm-loss failsafe check            — if no valid packet in COMM_TIMEOUT_MS, drive_.stop()
  3. every CONTROL_LOOP_PERIOD_MS        — battery_.update(); drive_.update(dt, compFactor)
  4. every TELEMETRY_PERIOD_MS           — commandProcessor_.sendTelemetry()
  5. watchdog_.feed()                    — hardware watchdog, every iteration
```

**Two independent safety layers:**
- **Comm-loss failsafe** (`CommandProcessor::isCommLost`) — handles "the link dropped but the firmware is fine": stops motors within `COMM_TIMEOUT_MS` (500 ms default) of the last valid packet.
- **Hardware watchdog** (`Watchdog`) — handles "the firmware itself hung" (stuck ISR, blocking call that never returns): resets the MCU if `feed()` isn't called within `HW_WATCHDOG_TIMEOUT_S` (2 s default), which a comms failsafe alone cannot catch since it depends on the main loop still running.

**Protocol** (`common/Protocol.h`):
`START | LEN | CMD | PAYLOAD | CRC-8 | END`, byte-stuffed so payload bytes can never be confused with framing bytes, and self-resynchronizing on any `START` byte so a single dropped or corrupted byte can't permanently wedge the parser. Every command is length- and range-validated before being acted on; invalid frames get a `NACK` with a reason code, bad-CRC frames are silently dropped (the sender finds out via the comm-loss failsafe/heartbeat rather than a NACK it can't trust the CMD field of anyway).

---

## 4. Building

**Firmware (ESP32):**
```bash
cd firmware
pio run -e esp32dev            # build
pio run -e esp32dev -t upload  # build + flash
```

**Native unit tests (no hardware):**
```bash
cd firmware
pio test -e native
```

**Host telemetry logger:**
```bash
cd basestation
mkdir build && cd build
cmake .. && make
# pairs with the rover's Bluetooth SPP link, which shows up as e.g. /dev/rfcomm0
./telemetry_logger /dev/rfcomm0 rover_test.db "session notes here"
```

---

## 5. Testing & validation plan

### 5.1 Unit tests (native, run on a laptop)
`pio test -e native` runs three suites against the real Unity framework:
- **`test_protocol.cpp`** — encode/decode round-trip (including bytes that collide with framing bytes, exercising the escaping logic), CRC rejects corrupted payloads, truncated frames never complete, and the parser resynchronizes after garbage bytes instead of getting stuck.
- **`test_pid.cpp`** — the controller converges to setpoint on a stable synthetic plant, output is correctly saturated at extreme gains, `reset()` clears integral windup, and a zero/invalid `dt` is handled without producing garbage output.
- **`test_kinematics.cpp`** — straight-line commands produce equal left/right targets, pure rotation produces opposite-sign targets, combined motion matches hand-calculated values, and zero command yields zero targets.

### 5.2 Hardware-in-the-loop bench test (wheels off the ground)
1. Prop the rover on blocks so all 4 wheels spin freely with no load.
2. Send `SET_VELOCITY` for a slow forward speed via a BLE terminal app or a throwaway script; confirm all 4 wheels turn the correct direction and `Encoder::totalTicks()` (exposed via telemetry) increments as expected. If a wheel spins backwards, swap its motor leads or flip that wheel's sign in `Encoder::handleInterrupt()` — don't touch the kinematics math.
3. Step the target speed and use `basestation/telemetry_logger` to capture a session; query `v_wheel_rms_error` — a well-tuned wheel should settle within a couple hundred ms with low steady-state RMS error. Iterate `SET_PID_GAINS` per wheel from there.

### 5.3 Voltage-compensation A/B validation
1. Power the rover from a variable bench PSU instead of the LiPo so you can sweep voltage on demand and reproduce results.
2. Run one logging session with `SET_COMP_MODE(0)` (compensation off) while sweeping the PSU from ~12.6V down to ~9.5V at a fixed commanded speed.
3. Run a second session with `SET_COMP_MODE(1)` (on) across the same voltage sweep, same commanded speed.
4. Query `v_voltage_vs_error` for both sessions: compensation working correctly shows `avg_abs_error_front_wheels` staying roughly flat across the voltage buckets for `comp_enabled = 1`, versus growing steadily as voltage drops for `comp_enabled = 0`.

### 5.4 Communication-loss failsafe test
1. Start driving, then physically move the Bluetooth host out of range (or kill the app) mid-command.
2. Confirm motors stop within `COMM_TIMEOUT_MS` — visually, and by querying `v_telemetry_gaps` afterward (a gap should line up with when the link dropped; telemetry keeps flowing from the rover's perspective, so the *absence* of new SET_VELOCITY commands combined with unchanging `commandOut` telemetry values is the signal to look for, alongside the physical stop).
3. Re-establish the link and confirm the rover accepts new commands again without needing a power cycle.

### 5.5 Protocol robustness / negative testing
Write a short script that connects to the same serial port and intentionally sends: a frame with a flipped payload bit (bad CRC), a frame with a `LEN` field that doesn't match the actual payload it sent, and a frame using an unknown `CMD` byte. Confirm: the firmware never crashes, bad-CRC frames are silently dropped (no ACK/NACK, no motor response), and th unknown-command frame gets a `NACK` with `UNKNOWN_CMD`. `test_protocol.cpp` covers this logic in isolation already; this step confirms it holds up over the real radio link too.

### 5.6 Hardware watchdog test
Build a throwaway firmware variant that spins forever in a tight loop somewhere inside `Rover::update()` (simulating a lockup) and confirm the ESP32 resets itself within `HW_WATCHDOG_TIMEOUT_S`. Revert before flashing the real firmware — this is a one-off destructive test, not something to leave wired into the build.

---

## 6. Known follow-ups
- `config.h` flags a pin-conflict TODO between the encoder map and the battery ADC pin — finalize against your actual board rev before wiring.
- 1x quadrature decoding is used for simplicity; if wheel-speed resolution turns out too coarse at low speeds, upgrade `Encoder` to full 4x decode or the ESP32 PCNT peripheral without changing any other module.
- `telemetry_logger.cpp`'s serial handling is POSIX-only (Linux/macOS); a Windows build would need a different serial backend behind the same `openSerialPort()` seam.
