# Feetech driver critique — wrist_roll range + correctness bugs

Analysis of `feetech_ros2_driver` (the STS3215 ros2_control hardware interface) and the
FLAM follower config. Two parts: (1) the wrist_roll range-of-motion question, (2) a bug
catalogue. Findings are grounded in `file:line` references and cross-checked by an independent
review pass.

Servo background: see [`servo-reference.md`](servo-reference.md).

---

## Part 1 — wrist_roll: why the range is small, and how to fix it

### TL;DR

The homing offset does **not** shrink travel by itself. But the value chosen for wrist_roll
(`homing_offset: 1804`) places the **raw magnetic-encoder seam ~21° away from rest** on the
positive side, and **single-turn mode cannot cross that seam**. Result: usable travel is badly
asymmetric — roughly **+21° / −157°** instead of the ±160° the URDF advertises. To get "≥180°
each side" you need **multi-turn mode**, which the driver does **not** currently support cleanly
(see bug **H1**).

### The numbers (FLAM follower)

| Source | Value |
| --- | --- |
| `flam/.../follower_joints.yaml:46-59` | `homing_offset: 1804`, `range_min: 0`, `range_max: 4095` |
| same, config comment | raw encoder tick at rest = **3852** (offset = 3852 − 2048) |
| `flam_ee_common.xacro:31`, joint `type="revolute"` | URDF limits `lower=-2.74385` (−157.2°), `upper=2.84121` (+162.8°) |

### Why it's limited — the encoder seam

The servo firmware reports `Present = raw − homing_offset (mod 4096)`, and the driver maps the
ROS angle θ linearly around center 2048:

```
raw(θ) = from_radians(θ) + 2048 + homing_offset   (mod 4096)
       = from_radians(θ) + 3852                    (mod 4096)
```

- θ = 0 → raw 3852 → Present 2048 → ROS reads 0.0 ✓ (matches config comment)
- The 12-bit encoder has a **hard seam where raw 4095 wraps to 0**. In **single-turn** position
  mode the servo **cannot drive across that seam** — it's the encoder's discontinuity.
- Distance from rest to the seam:
  - **positive**: `from_radians(θ) = +243` → θ = **+21.4°** (raw 3852 → 4095) ← the wall
  - negative: `from_radians(θ) = −3852` → θ = −338° (never reached; URDF stops at −157°)

So the nearest seam is just **+21.4°** above rest. Everything above that — including the URDF's
own +162.8° upper limit — is **physically unreachable in single-turn mode**, and commanding into
it makes the servo try to cross the seam and **jam**.

### This is already observed, not theoretical

The driver's own code comments it, on wrist_roll specifically:

> `feetech_ros2_driver.cpp:243-244` — *"the stale goal is reinterpreted in the new frame and the
> servo JUMPS on torque-on (seen on wrist_roll: ~49° lunge into a mechanical jam + overload
> cutoff)."*

That jam **is** the seam. The `range_min: 0 / range_max: 4095` setting makes it worse, not
better: with the seam *inside* `[0, 4095]`, the angle-limit registers don't fence the joint off
from the seam at all.

### Root cause

`homing_offset: 1804` was chosen to satisfy the **URDF-zero convention** (rest must read 0.0 —
pinned by `flam/tests/kinematics/test_wrist_roll_convention.py`), **not** to center the joint in
the encoder. Because the output horn is mounted such that rest = raw 3852 (near the top of the
encoder), the seam ends up right next to the working position. You cannot have *both* "rest reads
0" *and* "seam far from rest" in single-turn mode unless rest happens to sit near raw 2048.

### Fix options (best first, for "≥180° per side")

1. **Enable multi-turn mode** — set **both** `range_min` and `range_max` to `0`. The STS3215 then
   does absolute positioning across **±7 turns** (datasheet §7-13), no seam. This matches the
   "it can spin many times" intuition and is the only option giving >±180°.
   **Blocked by bug H1**: the hot-loop position read isn't sign-magnitude-decoded, so multi-turn
   positions (negative / >4095) would be misread as ~+47 rad and the controller would lunge.
   **Fix H1 first**, then this is a config change. Also confirm the goal-write path handles
   multi-turn targets (it encodes sign-magnitude with bit 15, magnitude ≤ 32767 ≈ ±7.99 rev — OK,
   but add a clamp; see M2).
2. **Re-index the output horn** so rest sits near **raw ≈ 2048** (rotate the spline by the
   appropriate number of 25T teeth). Then single-turn gives a symmetric **±180°** with the seam
   at the far side. Hardware change; keeps the driver single-turn-simple. Note this changes the
   homing offset and must be re-pinned in `test_wrist_roll_convention.py` + `joint_conventions.md`.
3. **Stay single-turn, make it safe** (band-aid): tighten the URDF `upper` for wrist_roll to
   ≲ +21° so the controller never commands into the seam/jam, and set `range_min`/`range_max` to a
   window that **excludes** the seam. You keep only ~+21°/−157°, but you stop the overload cutoffs.

**Recommendation:** option **1** (multi-turn) is what actually gets you the range you want; it
requires fixing **H1** (a small, independently-correct fix). Option **2** if you'd rather keep the
driver in simple single-turn mode and accept exactly ±180°.

---

## Part 2 — Bug catalogue

Severity: **High** = wrong behaviour / safety; **Med** = real but narrower; **Low** = cosmetic /
robustness. Cross-checked by an independent review.

### H1 — Hot-loop position read is not sign-magnitude decoded  *(the multi-turn blocker)*
`feetech_ros2_driver.cpp:353-355`

`read()` computes `to_radians(from_sts(word) − kStsMidpoint)` with **no**
`decode_sign_magnitude(..., SMS_STS_SIGN_BIT_POSITION)` — even though **velocity** right below it
(`:358-360`) *does* decode, and the helper `read_position()` (`communication_protocol.cpp:93-97`)
*does* decode. Present position is sign-magnitude on bit 15.

- **Single-turn:** harmless — firmware keeps Present in 0–4095, bit 15 always clear, so the raw
  combine equals the decoded value. (This is why it's shipped and "works".)
- **Multi-turn:** broken — a Present like `0x8001` (true −1) reads as `0x8001 − 2048 = 30721`
  ticks ≈ **+47 rad**. A position controller closing on that state commands a violent correction →
  arm lunge.

**Fix:** decode before centering:
```cpp
state_hw_positions_[index] = feetech_driver::to_radians(
    feetech_driver::decode_sign_magnitude(
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[0], .high = readings[1]}),
        SMS_STS_SIGN_BIT_POSITION) - feetech_driver::kStsMidpoint);
```
Round-trip check: write sends `encode_sm(from_radians(θ)+2048)`; read becomes
`to_radians(decode_sm(raw)−2048)` — exact inverses for all in-range values; single-turn behaviour
is unchanged. **This is the prerequisite for the multi-turn wrist_roll fix.**

### M2 — `encode_sign_magnitude` can throw out of the realtime `write()` loop
`common.hpp:45-52`, reached via `write()` → `sync_write_position` (`communication_protocol.hpp:95`)

`encode_sign_magnitude` throws `std::out_of_range` if `|value| > 32767`. `write()` builds the tick
as `from_radians(hw_positions_[i]) + kStsMidpoint` with **no clamp**. A large/garbage setpoint
(or the corrupted state from H1) → magnitude > 32767 → **uncaught exception out of `write()`**,
which ros2_control does not expect → control thread tears down, arm left in its current torque
state. **Fix:** clamp the commanded tick to the valid range before encoding (and ideally return an
`Expected` instead of throwing).

### M3 — `NaN → int` cast on the command path (UB)
`feetech_ros2_driver.cpp:302, 406`; `common.hpp:39-41`

`hw_positions_` is initialized to `quiet_NaN()`. `write()` unconditionally does
`from_radians(hw_positions_[i])` → `static_cast<int>(NaN)` — **undefined behaviour** (typically
`INT_MIN`/0). `on_activate` (`:464`) seeds `hw_positions_ = state_hw_positions_`, which covers the
normal path, but a `write()` before activation, or a controller that emits NaN, yields a garbage
goal (then M2). **Fix:** skip joints where `std::isnan(hw_positions_[i])` in `write()`.

### M5 — EEPROM rewritten on every `on_init` (write-wear)
`feetech_ros2_driver.cpp:159-236`

`configure_joints_` unconditionally writes `homing_offset` (EEPROM addr 31) and the other EEPROM
params (angle limits, torque limits, PID…) on **every** init / launch. STS EEPROM has limited
write endurance. The homing-offset block even reads back the current value (`:208-219`) but only
to *warn* — it still writes when `current == target`. **Fix:** guard each EEPROM write on
`readback != target` (you already have the readback for homing_offset), or gate all EEPROM writes
behind an explicit `--calibrate` flag instead of running them every boot.

### M4 — misleading `sync_write` length comment
`communication_protocol.hpp:50`

Value `(N+1)*ids.size() + 4` is **correct** (per servo: 1 id + N param bytes; +4 overhead), but
the comment "`#parameters * (#parameters + 1 (id)) + 4`" is nonsense and will mislead the next
editor. Comment-only fix.

### L7 — `check_head` off-by-one
`communication_protocol.cpp:45-46` — `for (i = 0; i <= max_number_of_tries; i++)` runs 11 times
for `max_number_of_tries = 10`; the error says "after 10 tries". Harmless (one extra byte read).

### L8 — broken `fmt::format` diagnostic strings
`communication_protocol.cpp:69-70, 72-73` — format strings have **zero `{}`** but pass 3 / 1 args
(`"buffer[0] != id != BROADCAST_ID", buffer[0], id, kBroadcastId`). Args are silently dropped, so
the error message is useless exactly when you need it. **Fix:** add `{}` placeholders.

### L10 — one bad servo masks the whole bus (by design, noted)
`communication_protocol.hpp:196-215` + `feetech_ros2_driver.cpp:318-342` — `sync_read` aborts on
the first checksum mismatch, so `read()` holds last-state for **all** joints and can trip
`needs_torque_recovery_`/reconnect. This is the documented self-healing policy (hold-last-state,
never latch the component — `:320-322`), so it's intentional and safe; a chronically-bad single
servo would, however, perpetually mask the other five. Possible robustness improvement: per-servo
validity.

### Verified-fine (checked, not bugs)
- Comm failures return `OK` not `ERROR` (`read():341`, `write():424`) — intentional, avoids
  latching the ros2_control component (`:320-322`). Correct.
- Goal-sync-before-torque-on (`:240-257`, `recover_torque_:441-455`) uses the **decoded**
  `read_position` and re-encodes via `write_position` — correctly prevents the activation lunge.
  (The state-interface read path H1 is the *only* place that skips the decode.)
- `flashInputBuffer()` before each `sync_read` (`:163`) — correct desync prevention.
- Checksum computed before the `0xFF,0xFF` header bytes are set (`write_buffer`, `sync_read`,
  `sync_write`) — correct, header excluded from checksum.
- Velocity sign-magnitude decode (`:358-360`) — correct (and is exactly what H1 is missing).
- Firmware version byte order (`:59-60`) and model-number decode (777 = STS3215) — consistent.

---

## Suggested fix order

1. **H1** — one-line decode fix; unblocks multi-turn and removes a latent lunge. Do first.
2. **M2 + M3** — clamp + NaN-guard the command path so the realtime loop can't throw / emit
   garbage goals. Cheap safety.
3. **wrist_roll range** — once H1 lands, switch wrist_roll to multi-turn (`range_min/max = 0`),
   or re-index the horn for symmetric single-turn ±180°. Re-pin the convention test either way.
4. **M5** — skip-write-if-unchanged for EEPROM longevity.
5. **M4 / L7 / L8** — cosmetic cleanups.
