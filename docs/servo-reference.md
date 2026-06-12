# Servo Reference — Feetech STS3215 (SO-101 / FLAM)

> **Purpose.** Reference for writing/modifying the servo driver. Identifies the exact
> servo, captures the official spec-sheet numbers, and maps every control register the
> driver touches to its meaning and source. Cross-references the in-repo driver
> (`feetech_ros2_driver`) so register addresses and encoding can be verified against code.

---

## 1. Which servo is this?

**CORRECTION (2026-06-12, confirmed on the real arm): the flam follower is the 12 V
"30 KG·CM" STS3215 variant**, not the 7.4 V one this page originally assumed. The arm is
a Waveshare **SO-ARM101** follower kit ("sub robotic arm uses bus servos rated at
30 kg·cm @ 12 V"); the bus honestly reported 12.0–12.2 V on its stock 12 V PSU. The
7.4 V/19 kg·cm description below is the LEADER-arm/SO-ARM100-class variant — register
map and protocol are identical, but electrical/torque numbers in §2 are from the 7.4 V
sheet and do NOT apply to the follower's motors (12 V stall ~30 kg·cm; driver voltage
warn window now [9.0, 13.0]).

Original (7.4 V variant) description: a plastic-shell, metal-gear, magnetic-encoder,
dual-axis, TTL serial bus smart servo — the standard LeRobot SO-101 LEADER actuator.

### Evidence

| Source | Evidence |
| --- | --- |
| Official datasheet | Product name: *"7.4V 19KG.CM plastic shell metal tooth magnetic coding dual-axis TTL serial servo"*, model `STS3215`, edition A/0, 2020-04-10 (Shenzhen Feetech RC Model Co.). |
| LeRobot SO-101 docs | *"The follower arm uses 6× STS3215 motors with 1/345 gearing. The leader … uses three differently geared motors."* |
| Driver model table | `STS3215` is an enumerated model — `feetech_ros2_driver/feetech_driver/include/feetech_driver/common.hpp:124` → `servo_model(9, 3, "STS3215")`. |
| Repo configs | `so101_bringup/config/hardware/follower_joints.yaml`, `flam/src/flam/config/hardware/follower_joints.yaml` — 6 joints, IDs 1–6, calibrated for STS3215 (4096-tick range). |
| Hardware doc | `so101-ros-physical-ai/docs/hardware.md` — references the official LeRobot SO-101 motor-setup/calibration flow and a Waveshare controller board. |

### Model-number readback (for driver auto-detection)

The servo reports its model in two EEPROM bytes: `MODEL_L` (addr 3) and `MODEL_H` (addr 4),
read-only. The driver combines them as `model_number = (MODEL_H << 8) | MODEL_L`
(`common.hpp:77`, `from_sts`). For the STS3215 the firmware returns **`MODEL_L = 9`** (the STS
series tag) and **`MODEL_H = 3`** (the 3215 model), i.e. **model number 777** (`0x0309`).
Lookup table: `common.hpp:107-128`; `STS3215` entry at `:124`. Series classification
(`get_model_series`, `common.hpp:142`) keys on the `"STS"` name prefix → `ModelSeries::kSts`.

---

## 2. Official datasheet specifications

Source: *SHENZHEN FEETECH RC MODEL CO., LTD. — STS3215 Product Specification, Edition A/0,
2020-04-10* (the 7.4 V / 19 kg·cm variant). Numbers quoted exactly; two columns where the
sheet specs both 6 V and 7.4 V operation.

### Electrical (§5)

| Parameter | @ 6 V | @ 7.4 V |
| --- | --- | --- |
| Typical operating voltage | 6 V | 7.4 V |
| **Working / input voltage range** | **4 V – 7.4 V** | |
| No-load speed | 0.238 s/60° (**42 RPM**) | 0.192 s/60° (**52 RPM**) |
| Running current (no load) | 130 mA | 150 mA |
| **Stall torque** | 16.5 kg·cm (229.54 oz·in) | **19.5 kg·cm** (271.28 oz·in) |
| Stall current (locked) | 2.0 A | **2.5 A** |
| Idle current (stopped) | 6 mA | 6 mA |
| Rated torque | 4 kg·cm | 5 kg·cm |
| Rated current | 500 mA | 650 mA |
| Motor terminal resistance | 2.5 Ω | |
| Kt (torque constant) | 8 kg·cm/A | |

### Environmental (§1–2)

- Storage temp: **−30 °C … +80 °C**; operating temp: **−20 °C … +60 °C**
- Standard test env: 25 ±5 °C, 65 ±10 % RH
- Certifications: EMC ✔, ROHS ✔ (ASTM F963, EN71, FCC, REACH: no)

### Mechanical (§6)

| Parameter | Value |
| --- | --- |
| Dimensions | **45.2 × 24.7 × 35 mm** |
| Weight | **55 ±1 g** |
| Case / gear material | PA+GF shell, **copper (metal) gears** |
| Bearing | ball bearing |
| Angle sensor | **12-bit magnetic encoder**, 360°, unlimited life |
| Output horn / spline | **25T, OD 5.9 mm**, M3×6 horn screw |
| Gear ratio (base part) | **1:345** |
| Backlash | ≤ 0.5° |
| Connector / cable | 5264-3P, 15 cm, pin 1 = GND, pin 2 = Vcc, pin 3 = Signal/TTL |
| Motor | cored ("Core Motor") |
| Life | > 100,000 cycles; motor noise 45 ±5 dB, gear noise 60 ±5 dB; **not waterproof** |

### Control characteristics (§7)

| Parameter | Value |
| --- | --- |
| Command signal | Digital packet |
| Protocol | **Half-duplex asynchronous serial**, 8 bit / 1 stop / no parity |
| Baud rate | 38,400 bps … **1 Mbps** (factory default **1 Mbps**) |
| ID range | **0 – 253** (customizable) |
| Control algorithm | PID |
| Neutral / median position | **180° = 2048** |
| Rotation range | 360° = 0 … 4096 ticks |
| **Resolution** | **0.088°/tick** (360° / 4096) |
| Default rotation direction | clockwise = 0 → 4096 |
| Feedback | load, position, speed, input voltage, current, temperature |
| Max position update rate | 1 ms |
| Signal high / low | 2–5 V / 0.0–0.45 V |

### Operating modes (§7-2, register 33 `SMS_STS_MODE`)

| Mode | Name | Behavior |
| --- | --- | --- |
| 0 | **Angle / servo mode** (default) | 0–360° absolute position control |
| 1 | Speed closed-loop | speed held under load, no deceleration |
| 2 | Speed open-loop | speed drops as load increases |
| 3 | Step mode | step movement relative to current position |

### On-board electronic protection (§7-2)

- **Overload**: stalls > 80 % of stall torque for 2 s → protect; re-sending a position command
  clears the flag. Threshold % and duration are customizable (registers 34/35/36).
- **Over-current**: running current > 2 A for 2 s → output off; re-send position to clear.
  Value/duration customizable (registers 28/29/38).
- **Over-voltage**: > 7.4 V or < 4 V → protect; auto-released when voltage returns to range
  (registers 14/15).
- **Over-temperature**: torque output off above 70 °C (register 13).
- **Multi-loop / multi-turn**: at full precision can address ±7 turns absolute, but turn count
  is **not retained on power-off**.
- **Centering function**: write `128` to address `40` (torque-enable register) to re-center.

---

## 3. SO-101 gear-ratio map (variant per joint)

The SO-101 follower uses one gear ratio across all joints; the **leader** mixes three ratios so
each joint can hold its own weight yet back-drive easily. All are the same 7.4 V STS3215 body —
only the internal gearbox differs (this changes effective torque, speed, and back-drivability,
not the register map or protocol).

| Joint | ID | Follower ratio | Leader ratio |
| --- | --- | --- | --- |
| Shoulder pan (base) | 1 | 1:345 | 1:191 |
| Shoulder lift | 2 | 1:345 | 1:345 |
| Elbow flex | 3 | 1:345 | 1:191 |
| Wrist flex | 4 | 1:345 | 1:147 |
| Wrist roll | 5 | 1:345 | 1:147 |
| Gripper | 6 | 1:345 | 1:147 |

*Source: LeRobot SO-101 assembly docs. Common retail SKUs: 1:345 ≈ "19 kg" (ST3215-C001),
1:191 ≈ heavy-duty (C044), 1:147 ≈ standard-torque/faster (C046).*

> **FLAM note.** FLAM is a single-arm modified SO-101 (see project memory). Its
> `wrist_roll` neutral is rotated 180° versus stock SO-101, reflected in its homing offset.
> See `flam/src/flam/config/hardware/follower_joints.yaml`.

---

## 4. Control / memory register table

The driver's register map lives in
`feetech_ros2_driver/feetech_driver/include/feetech_driver/SMS_STS.h` and is intentionally
compatible with LeRobot's `STS_SMS_SERIES_ENCODINGS_TABLE`. Feetech's authoritative,
field-by-field memory table is the spreadsheet linked from the driver's user guide
(`feetech_ros2_driver/doc/user.md:16`):
<https://docs.google.com/spreadsheets/d/1GVs7W1VS1PqdhA1nW-abeyAHhTUxKUdR/edit?gid=364516031>.

All multi-byte values are little-endian word pairs (`to_sts`/`from_sts`, `common.hpp:69-77`):
low byte at address N, high byte at N+1.

### EEPROM — read-only (persisted)

| Addr | Name | Notes |
| --- | --- | --- |
| 0–1 | `FIRMWARE_VER_L/H` | Firmware major/minor. **Driver logs this at startup**; firmware < 3.10 has a SYNC-READ corruption bug (`feetech_ros2_driver.cpp`). |
| 3–4 | `MODEL_L/H` | Model number — STS3215 = `L=9, H=3` (→ 777). See §1. |

### EEPROM — read/write (persisted across power cycles)

| Addr | Name | Driver param / use |
| --- | --- | --- |
| 5 | `ID` | Servo bus ID (1–6 on these arms). Set once via LeRobot motor-setup. |
| 6 | `BAUD_RATE` | Serial baud (default 1 Mbps). |
| 7 | `RETURN_DELAY` | `return_delay_time` (YAML/URDF). |
| 8 | `RESPONSE_STATUS_LEVEL` | Status-packet return level. |
| 9–10 | `MIN_ANGLE_LIMIT_L/H` | `range_min` (raw ticks, after homing offset). |
| 11–12 | `MAX_ANGLE_LIMIT_L/H` | `range_max`. |
| 13 | `MAX_TEMPERATURE_LIMIT` | Over-temp cutoff (≈ 70 °C). |
| 14 | `MAX_INPUT_VOLT` | Over-voltage protect (≤ 7.4 V). |
| 15 | `MIN_INPUT_VOLT` | Under-voltage protect (≥ 4 V). |
| 16–17 | `MAX_TORQUE_L/H` | `max_torque_limit` (gripper default 500). |
| 18 | `PHASE` | Motor phase config. |
| 19 | `UNLOADING_CONDITION` | Fault conditions that unload torque. |
| 20 | `LED_ALARM_CONDITION` | Fault conditions that flash the LED. |
| 21 | `P_COEF` | `p_coefficient` (configs use 16). |
| 22 | `D_COEF` | `d_coefficient` (configs use 32). |
| 23 | `I_COEF` | `i_coefficient` (configs use 0). |
| 24–25 | `MINIMUM_STARTUP_FORCE_L/H` | Punch / minimum drive force. |
| 26 | `CW_DEAD` | Clockwise dead-band. |
| 27 | `CCW_DEAD` | Counter-clockwise dead-band. |
| 28–29 | `PROTECTION_CURRENT_L/H` | `protection_current` (gripper default 250). |
| 30 | `ANGULAR_RESOLUTION` | Resolution multiplier (multi-turn). |
| 31–32 | `OFS_L/H` | **Homing offset** (`homing_offset`). Sign-magnitude, sign bit 11. Firmware applies `Present_Position = Actual_Position − Homing_Offset`. |
| 33 | `MODE` | Operating mode 0–3 (see §2). 0 = position. |
| 34 | `PROTECTIVE_TORQUE` | Torque held after overload trips. |
| 35 | `PROTECTION_TIME` | Overload trip duration. |
| 36 | `OVERLOAD_TORQUE` | `overload_torque` % (gripper default 25). |
| 37 | `SPEED_CLOSED_LOOP_P_COEF` | Velocity-mode P gain. |
| 38 | `OVER_CURRENT_PROTECTION_TIME` | Over-current trip duration. |
| 39 | `VELOCITY_CLOSED_LOOP_I_COEF` | Velocity-mode I gain. |

### SRAM — read/write (volatile)

| Addr | Name | Use |
| --- | --- | --- |
| 40 | `TORQUE_ENABLE` | 0 = off, 1 = on; write **128** to re-center (§2). |
| 41 | `ACC` | `acceleration` (configs use 254). |
| 42–43 | `GOAL_POSITION_L/H` | Target position, ticks. Sign-magnitude, sign bit 15. |
| 44–45 | `GOAL_TIME_L/H` | Time to target (time-based moves). |
| 46–47 | `GOAL_SPEED_L/H` | Target speed. |
| 48–49 | `TORQUE_LIMIT_L/H` | Runtime torque limit. |
| 55 | `LOCK` | EEPROM write lock (1 = lock, 0 = unlock before EEPROM writes). |

### SRAM — read-only (feedback / status)

| Addr | Name | Use |
| --- | --- | --- |
| 56–57 | `PRESENT_POSITION_L/H` | Current position. Sign-magnitude, sign bit 15. |
| 58–59 | `PRESENT_SPEED_L/H` | Current speed. |
| 60–61 | `PRESENT_LOAD_L/H` | Current load. |
| 62 | `PRESENT_VOLTAGE` | Bus voltage (×0.1 V). |
| 63 | `PRESENT_TEMPERATURE` | °C. |
| 66 | `MOVING` | 1 = moving. |
| 69–70 | `PRESENT_CURRENT_L/H` | Current draw (×6.5 mA). |

### 4.1 Cross-reference with the official Feetech memory table

Verified against Feetech's authoritative STS memory table (the spreadsheet linked from
`doc/user.md`). Every address the driver defines matches the official table; the table adds
scaling, defaults, and two registers the driver does not map. **Use these units when writing
register values — the raw integer is rarely the engineering value.**

**Scaling / units (raw register → physical):**

| Register(s) | Raw unit | Example |
| --- | --- | --- |
| 14/15 `MAX/MIN_INPUT_VOLT`, 62 `PRESENT_VOLTAGE` | **0.1 V** | default max=80 → 8.0 V; min=40 → 4.0 V |
| 28/29 `PROTECTION_CURRENT`, 69/70 `PRESENT_CURRENT` | **6.5 mA** (max 511 → 3255 mA) | gripper `protection_current: 250` → ≈ **1.63 A** |
| 16/17 `MAX_TORQUE`, 48/49 `TORQUE_LIMIT` | **0.1 %** (1000 = 100 %) | gripper `max_torque_limit: 500` → **50 %** |
| 34 `PROTECTIVE_TORQUE`, 36 `OVERLOAD_TORQUE` | **1 %** | `overload_torque: 25` → **25 %**; defaults 20 / 80 |
| 35 `PROTECTION_TIME`, 38 `OVER_CURRENT_PROTECTION_TIME` | **10 ms** (max 254 → 2.54 s) | default 200 → 2.0 s |
| 41 `ACC` | **100 step/s²** | `acceleration: 254` → 25,400 step/s² |
| 46/47 `GOAL_SPEED` | **step/s** | 50 step/s ≈ 0.73 RPM |
| 7 `RETURN_DELAY` | **2 µs** (max 254 → 508 µs) | default 0 |
| 13 `MAX_TEMPERATURE_LIMIT`, 63 `PRESENT_TEMPERATURE` | **°C** | default cutoff 70 °C |
| 24/25 `MINIMUM_STARTUP_FORCE` | **0.1 %** | default 16 |
| All position/angle fields | **step** = 0.088° (4096 = 360°) | — |

**Confirmations from the official table:**

- **Model encoding.** Addresses 3/4 (`MODEL_L/H`) carry default values **9** and **3** — the
  STS series tag (9) and the 3215 model (3). This corroborates §1's model number 777.
- **Firmware default** is **3.6** (addr 0/1 defaults = 3, 6). Stock SO-101 servos therefore ship
  *below* the 3.10 threshold of the SYNC-READ corruption bug — so the driver's startup firmware
  log is not theoretical; expect to hit it on un-updated motors.
- **Homing offset range** (addr 31) is officially **−2047 … +2047 step** → exactly the
  sign-bit-11 magnitude limit the driver enforces (`encode_sign_magnitude`). Target position
  (addr 42) is **−32766 … +32766** → the sign-bit-15 limit.
- **Baud register** (addr 6) is an **index 0–7, not a raw rate** (default 0). Standard Feetech STS
  mapping: `0=1,000,000 · 1=500,000 · 2=250,000 · 3=128,000 · 4=115,200 · 5=76,800 · 6=57,600 ·
  7=38,400`. The driver's `serial_port.cpp` works in real baud rates, so the index↔rate mapping
  must be applied when reading/writing this register.
- **Torque switch** (addr 40) takes **0 = off, 1 = on, 128 = center-calibrate** (the official
  table labels value-range "enable/disable/calibrate"). Matches the §2 re-centering trick.

**Naming differences (same bytes, different label):**

| Addr | Driver name (`SMS_STS.h`) | Official table name |
| --- | --- | --- |
| 3 / 4 | `MODEL_L` / `MODEL_H` | "Servo main version" / "Servo sub version" |
| 16 | `MAX_TORQUE` | "Maximum torque" (0.1 % of stall) |
| 33 | `MODE` | "Operation mode" — table lists range 0–2; the **datasheet** documents a 4th mode (3 = step), so treat 0–3 as valid but verify on your firmware |
| 44 | `GOAL_TIME` | "Running time" — in PWM/mode-2 this is a 0.1 % duty, not a duration |
| 46 | `GOAL_SPEED` | "Running speed" (step/s) |

**Registers in firmware but NOT mapped by the driver** (worth adding if you need fault
handling):

| Addr | Official name | Why it matters |
| --- | --- | --- |
| 64 | Asynchronous write flag | Indicates a buffered (REG_WRITE) async op is pending. |
| **65** | **Servo status / error bits** | **Per-servo fault bitmask** (over-voltage / over-temp / over-load / over-current, etc.). The driver currently infers faults indirectly; reading addr 65 gives the actual error condition. Strong candidate for a driver enhancement. |

---

## 5. Wire protocol (for driver work)

Instruction bytes (`common.hpp:79-88`):

| Byte | Instruction | Use |
| --- | --- | --- |
| `0x01` | PING | Presence / discovery |
| `0x02` | READ | Read N bytes from an address |
| `0x03` | WRITE | Write immediately |
| `0x04` | REG_WRITE | Buffer a write |
| `0x05` | REG_ACTION | Execute buffered writes |
| `0x06` | RECOVERY/RESET | Factory reset |
| `0x82` | SYNC_READ | Read from many servos in one transaction |
| `0x83` | SYNC_WRITE | Write to many servos in one transaction |

- **Broadcast ID**: `0xFE` (`common.hpp:88`).
- **Packet frame**: `[0xFF, 0xFF, ID, Length, Instruction, …params…, Checksum]`; checksum =
  inverted byte-sum (`sum_bytes`, `common.hpp:93-97`).
- **Half-duplex TTL** on a single data line — TX and RX share the wire (datasheet §11, §7-2);
  the driver must manage bus turnaround. Default 1 Mbps, 8N1.
- **SYNC_READ caveat**: firmware < 3.10 corrupts SYNC_READ; the driver logs firmware versions
  at startup so this is diagnosable.

### Position / angle conversion (`common.hpp:24-41`)

```
resolution      = 4096 ticks / 360°       (kStsResolution)
midpoint        = 2048 ticks   = center   (kStsMidpoint)
to_angle(t)     = t * 360 / 4096           (degrees)
to_radians(t)   = t * 2π / 4096
from_angle(d)   = d * 4096 / 360           (ticks)
```

### Sign-magnitude encoding (`common.hpp:43-60`)

Signed fields are **sign-magnitude**, not two's complement — the sign lives in a specific bit
and the rest is magnitude. Getting this wrong is the classic driver bug, so the sign bits are
named explicitly (`SMS_STS.h:79-81`):

| Field | Sign bit | Max magnitude |
| --- | --- | --- |
| Homing offset (`OFS`) | **11** | 2047 |
| Position (goal / present) | **15** | 32767 |
| Velocity | **15** | 32767 |

`encode_sign_magnitude(value, sign_bit)` throws if `|value|` exceeds the magnitude width — a
useful guard when computing homing offsets.

---

## 6. How the repo configures these servos

- **Per-joint config** lives in YAML and is matched to ROS joints **by servo `id`**, not by name
  (`feetech_ros2_driver/doc/user.md:41`). Precedence: `joint_config_file` (YAML) **overrides**
  URDF/Xacro `<param>` defaults (`docs/hardware.md:132`). On init, the driver writes the
  resolved values to the servo's EEPROM.
- **Calibration** (IDs, baud, homing offsets, angle limits) is done once via the
  LeRobot SO-101 flow and persists in EEPROM; the repo's `lerobot_*.json` files are raw
  calibration output kept for reference, and the YAML files are override examples
  (`docs/hardware.md:124-169`).
- **Homing**: the driver always centers at 2048; the old `offset` param is deprecated in favor
  of `homing_offset`, which writes to EEPROM so centering happens in hardware
  (`feetech_ros2_driver/doc/user.md:3, 22`).
- **Gripper protection defaults** (set in `so101_ros2_control.xacro`, not produced by LeRobot
  calibration): `max_torque_limit: 500`, `protection_current: 250`, `overload_torque: 25`
  (`docs/hardware.md:149-153`). These map to registers 16/17, 28/29, and 36 respectively, and
  exist to keep a stalled gripper from cooking the motor.

### Example calibration (SO-101 follower — `so101_bringup/config/hardware/follower_joints.yaml`)

| Joint | ID | homing_offset | range_min | range_max |
| --- | --- | --- | --- | --- |
| shoulder_pan | 1 | 530 | 866 | 3231 |
| shoulder_lift | 2 | −369 | 896 | 3211 |
| elbow_flex | 3 | −98 | 866 | 3066 |
| wrist_flex | 4 | 725 | 1039 | 3121 |
| wrist_roll | 5 | 1704 | 123 | 3955 |
| gripper | 6 | 1573 | 2027 | 3483 |

All joints: `p=16, i=0, d=32, acceleration=254, return_delay_time=0`. Gripper adds the
protection trio above.

---

## 7. Key files

| File | What it is |
| --- | --- |
| `feetech_ros2_driver/feetech_driver/include/feetech_driver/SMS_STS.h` | Register address map (this doc's §4). |
| `…/feetech_driver/include/feetech_driver/common.hpp` | Encoding/decoding, model table, instruction bytes, conversions. |
| `…/feetech_driver/include/feetech_driver/communication_protocol.hpp` | Packet building / parsing. |
| `…/feetech_driver/src/serial_port.cpp` | Baud-rate handling, port config. |
| `feetech_ros2_driver/src/feetech_ros2_driver.cpp` | ros2_control `FeetechHardwareInterface` plugin. |
| `feetech_ros2_driver/doc/user.md` | URDF/YAML param reference + Feetech memory-table link. |
| `so101_description/urdf/ros2_control/so101_ros2_control.xacro` | Joint/servo wiring, gripper protection defaults. |
| `so101_bringup/config/hardware/{follower,leader}_joints.yaml` | Per-robot calibration overrides. |
| `so101-ros-physical-ai/docs/hardware.md` | Device naming, udev, permissions, calibration flow. |

---

## Sources

- [Feetech STS3215 official datasheet (translated PDF), Core Electronics](https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf) — Edition A/0, 2020-04-10, "7.4V 19KG.CM" variant.
- [LeRobot SO-101 assembly & motor-setup docs (Hugging Face)](https://huggingface.co/docs/lerobot/so101) — follower 1:345; leader mixed 1:191 / 1:345 / 1:147.
- [Feetech ST3215 C001 (7.4V, 1:345, 19 kg) — Seeed Studio](https://www.seeedstudio.com/STS3215-19kg-cm-7-4V-Serial-Servo-p-6338.html)
- [Feetech STS3215 instruction manual — manuals.plus](https://manuals.plus/ae/1005009214017320)
- [Independent test report (backlash/repeatability/torque) — Robonine](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/)
- In-repo driver code & configs (paths in §7), feetech memory table: <https://docs.google.com/spreadsheets/d/1GVs7W1VS1PqdhA1nW-abeyAHhTUxKUdR/edit?gid=364516031>
