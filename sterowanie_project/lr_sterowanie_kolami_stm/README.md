# Wheel-control firmware (STM32G474)

Firmware for the small rover's motor board. One STM32G474RET3 drives **4 DC motors**
through 4 H-bridges, reads 4 quadrature encoders, and talks to the PC (ROS2) over
**CAN-FD** through the MAB CANdle.

> **Status:** the code compiles and all host unit tests pass.
> **Nothing has been tested on real hardware yet.** Several values are still
> placeholders (see [Configuration](#7-configuration-app_configh)).

---

## Contents

1. [What it does, in one minute](#1-what-it-does-in-one-minute)
2. [Where it sits in the rover](#2-where-it-sits-in-the-rover)
3. [Hardware used](#3-hardware-used)
4. [The CAN-FD protocol](#4-the-can-fd-protocol)
5. [How the firmware works inside](#5-how-the-firmware-works-inside)
6. [Safety behaviour](#6-safety-behaviour)
7. [Configuration (`app_config.h`)](#7-configuration-app_configh)
8. [Building, flashing and testing](#8-building-flashing-and-testing)
9. [First power-on checklist](#9-first-power-on-checklist)
10. [Debugging tips](#10-debugging-tips)
11. [Code map](#11-code-map)
12. [Known limitations](#12-known-limitations)

---

## 1. What it does, in one minute

1. The PC sends **one command frame** (ID `0x100`) with "drive forward at X m/s,
   turn at Z rad/s" and a **mode** (open loop / PID / stop).
2. The board works out how fast each of the 4 wheels must turn (left and right
   wheels differ when turning).
3. Every **100 ms** it reads the encoders, runs the speed control and sets the
   motor PWM.
4. Every **100 ms** it sends **one feedback frame per wheel** (IDs `0x101`–`0x104`)
   with wheel position and speed, for ROS2 `joint_states`.
5. If commands stop arriving for **2.5 s**, all motors stop and the H-bridges are
   switched off. Driving resumes by itself when commands come back.

The board **never** sends anything to the PDS (power distribution). It only drives motors.

---

## 2. Where it sits in the rover

```
 PC / Jetson (ROS2) ──USB── CANdle ══CAN-FD══╦══ this board (STM32 + 4 H-bridges) ── 4 DC motors
                                             ╚══ MAB PDS (power, not controlled by this board)
```

---

## 3. Hardware used

All of this is set in the CubeMX project (`lr_sterowanie_kolami_stm.ioc`).
**Do not change pins in code**; change them in CubeMX.

| Function | Peripheral / pins | Notes |
|---|---|---|
| CAN-FD | FDCAN1, RX = PB8, TX = PB9 | 1 Mbit/s nominal, 2 Mbit/s data, BRS on |
| Motor PWM | TIM1 CH1–CH4 = PC0–PC3 | 20 kHz |
| Motor direction | `M1_DIR`–`M4_DIR` = PC6–PC9 | GPIO |
| H-bridge enable | `M1_EN`–`M4_EN` = PC10–PC13 | GPIO, low at boot |
| Encoder motor 1 | TIM2 (PA15 / PB3) | 32-bit counter |
| Encoder motor 2 | TIM3 (PA6 / PA4) | 16-bit counter |
| Encoder motor 3 | TIM4 (PA11 / PA12) | 16-bit counter |
| Encoder motor 4 | TIM5 (PA0 / PA1) | 32-bit counter |
| Control tick | TIM6, every 10 ms | |
| Watchdog | IWDG, about 125 ms | resets the MCU if the firmware hangs |
| Programming / debug | SWD (PA13 / PA14) | |

### Motors and wheels

Motor N drives wheel N (confirmed):

| Motor | Wheel | Position | Feedback ID |
|---|---|---|---|
| 1 | 1 | right-front | `0x101` |
| 2 | 2 | right-rear | `0x102` |
| 3 | 3 | left-front | `0x103` |
| 4 | 4 | left-rear | `0x104` |

The right-side motors are mounted mirrored, so to move the rover **forward** they
spin the **opposite** way to the left motors. The firmware handles this for you
(right wheels × −1, left wheels × +1).

---

## 4. The CAN-FD protocol

- 11-bit standard IDs, **little-endian** (lowest byte first), 8 data bytes.
- Frames are CAN-FD with bit-rate switching. Classic CAN frames with the right ID
  are also accepted.
- Every ID except the command ID is rejected by the hardware filter.

### 4.1 Command: PC → board, ID `0x100`

Four signed 16-bit numbers. Divide each by 1000 to get the real value.

| Bytes | Field | Unit | Meaning |
|---|---|---|---|
| 0–1 | `lin_x` | m/s | forward speed (negative = backwards) |
| 2–3 | `lin_y` | m/s | **ignored** (fixed wheels cannot drive sideways) |
| 4–5 | `lin_z` | – | **mode**: `0` open loop, `1000` PID, `2000` stop |
| 6–7 | `ang_z` | rad/s | turn rate (positive = turn left / counter-clockwise) |

Range: −32.768 … +32.767 for each field.

**Modes**

| Raw value | Mode | What the motors do |
|---|---|---|
| 0 (±99) | Open loop | PWM is proportional to the target speed. No encoder needed. Good for first tests. |
| 1000 (±99) | PID | A PI controller uses the encoders to hold the target speed. |
| 2000 (±99) | Stop (X_STOP) | All motors at 0 % PWM. |
| anything else | – | **Frame rejected.** It does not count as a valid command. |

A frame is only **valid** if it has ID `0x100`, exactly 8 bytes, and a known mode.
Only valid frames keep the motors running (see [timeout](#6-safety-behaviour)).

**Example:** forward 0.5 m/s, PID mode, no turning:

```
lin_x = 0.5  → 500  → F4 01
lin_y = 0    → 0    → 00 00
mode  = PID  → 1000 → E8 03
ang_z = 0    → 0    → 00 00

ID 0x100, data: F4 01 00 00 E8 03 00 00
```

With a Linux SocketCAN adapter that supports CAN-FD (the `1` after `##` turns on BRS):

```bash
cansend can0 100##1F4010000E8030000
```

### 4.2 Feedback: board → PC, IDs `0x101`–`0x104`

Sent every 100 ms for every wheel, **always**, even when the motors are stopped.

| Bytes | Type | Field | Unit / scaling |
|---|---|---|---|
| 0–1 | int16 | steering | always `0` (these wheels do not steer) |
| 2–5 | int32 | position | wheel angle since power-on, rad × 10000 |
| 6–7 | int16 | velocity | wheel speed, rad/s × 1000 |

Things to know on the PC side:

- **Position starts at 0 every time the board powers on or resets.**
- Position is wheel angle, not distance. Distance = position × wheel radius.
- Position goes up to ±214 748 rad, then **wraps around** (like an odometer). It
  never jumps back to zero on its own.
- Velocity is capped at ±32.767 rad/s.
- Speed and position are in the **wheel's own direction**. When the rover drives
  forward, right wheels report **negative** speed and left wheels **positive**.

**Example:** wheel 3, position 1.5 rad, speed −2.5 rad/s:

```
ID 0x103, data: 00 00 98 3A 00 00 3C F6
```

Decoding in Python:

```python
import struct
steering, position, velocity = struct.unpack('<hih', data)
position_rad = position / 10000.0
velocity_rad_s = velocity / 1000.0
```

---

## 5. How the firmware works inside

### 5.1 Startup

1. CubeMX code sets up clocks and peripherals. The enable pins start **low**
   (bridges off). The watchdog starts running.
2. `app_init()`:
   - sets all PWM to 0 and keeps the H-bridges **off**, then starts PWM,
   - starts the encoders,
   - configures CAN-FD (filters, interrupts) and starts it,
   - starts the 10 ms tick timer.
3. The motors stay off until the **first valid command** arrives.

### 5.2 Main loop (`app_loop()`, runs continuously)

Each pass, in this order:

1. **Read commands.** Takes received frames from the buffer, checks them, and
   computes each wheel's target speed.
2. **Check CAN health.** If the bus went "bus-off", stops the motors and restarts CAN.
3. **Check the timeout.** Switches the H-bridges on or off.
4. **Every 100 ms: control step.** For each motor: read encoder → compute speed
   → run the selected mode → set PWM.
5. **Send feedback** frames (only 3 fit in the CAN hardware at once, so the 4th
   goes on the next pass).
6. **Feed the watchdog**, but only if everything is healthy.

### 5.3 Interrupts (kept very short)

| Interrupt | What it does |
|---|---|
| CAN receive | Copies every waiting frame into a buffer. No decoding. |
| CAN error | Raises a "bus-off" flag. |
| TIM6 (10 ms) | Increments a tick counter. |

All the real work happens in the main loop. This keeps the interrupts fast so no
CAN frame is lost.

### 5.4 From command to motor

```
lin_x, ang_z  ──►  right side: lin_x + ang_z × width/2      ──►  × direction flip
                   left side:  lin_x − ang_z × width/2           (right −1, left +1)
                                                                       │
                                                         target speed per wheel [m/s]
                                                                       │
          mode 0: PWM = target × 330 (max 1000 = 100 %)   ◄────────────┤
          mode 1: PWM = PI(target − measured speed)       ◄────────────┤
          mode 2: PWM = 0                                  ◄────────────┘
                                                                       │
                                     sign → DIR pin, size → PWM duty on TIM1
```

The controller works on a scale of −1000 … +1000, where 1000 means 100 % PWM.

**Open loop example:** target 1 m/s → 330 → 33 % duty. Anything above about
3.03 m/s is capped at 100 %.

**PI controller:**
- Proportional plus integral part (the derivative gain is 0).
- The integral stops growing while the output is at its limit, so it doesn't "wind up".
- If the target is about zero (below 0.01 m/s), the controller resets and outputs 0.

### 5.5 Encoders

- Every 100 ms the firmware reads how many counts each encoder moved.
- The counters wrap around (16-bit ones every 65 536 counts). The maths handles
  this, so wrap-around never causes a jump.
- Counts per wheel turn = counts per motor turn (28) × gear ratio (19) = 532
  (both are placeholders).
- Position is stored in a 64-bit counter, so it never overflows inside the board.

---

## 6. Safety behaviour

| Situation | What happens |
|---|---|
| **Power-on / reset** | PWM 0, H-bridges off. Nothing moves until a valid command arrives. |
| **No valid command for 2.5 s** | All PWM → 0, H-bridges off, PI reset. Next valid command → drives again. |
| **Invalid frame** (wrong length, unknown mode) | Ignored and counted. It does **not** keep the motors alive. |
| **CAN bus-off** (bus fault) | Motors stop at once. CAN restarts after 100 ms. Driving resumes on the next valid command. |
| **CAN won't recover** (10 restarts with no frame received) | Board stops feeding the watchdog → MCU resets → safe start again. |
| **Firmware hangs or tick timer stops** | Watchdog resets the MCU within about 125 ms. |
| **Open-loop command too large** | PWM is capped at 100 %. |

**What this means for the PC:** send the command frame **regularly**, for
example 10–20 times per second, even when the speed doesn't change. If you send
it less often than every 2.5 s, the rover stops.

The E-STOP is a hardware power cut. It is not handled in software.

---

## 7. Configuration (`app_config.h`)

**All** settings live in one file:
[`Core/Inc/app_config.h`](Core/Inc/app_config.h). No other file has numbers you should change.

Two markers show what still needs work:

- `PLACEHOLDER - REPLACE`: a temporary value (mostly copied from the old
  Rover 3.0 code). Replace it with the real value.
- `UNKNOWN - REQUIRES CONFIRMATION`: depends on the board schematic. Check it on
  the real hardware.

### Must check before driving

| Setting | Current | What it is |
|---|---|---|
| `MOTOR_EN_ACTIVE_LEVEL` | `1` | Pin level that turns an H-bridge **on**. If your bridge is on when EN is low, set `0` **and** change PC10–PC13 to start High in CubeMX. |
| `MOTOR_DIR_POSITIVE_LEVEL` | `0` | DIR pin level for positive speed. |
| `ENCODER_SIGN` | `{1,1,1,1}` | Per motor: flip to `-1` if the encoder counts the wrong way. |

### Mechanics (affect speed and position values)

| Setting | Current | What it is |
|---|---|---|
| `GEAR_RATIO` | 19 | motor turns per wheel turn |
| `ENC_COUNTS_PER_MOTOR_REV` | 28 | encoder counts per motor turn, as counted by the timer |
| `WHEEL_DIAMETER_M` | 0.26 | wheel diameter in metres |
| `ROVER_WIDTH_M` | 0.713 | distance between left and right wheels in metres |

### Control

| Setting | Current | What it is |
|---|---|---|
| `CMD_TIMEOUT_MS` | 2500 | stop if no valid command for this long |
| `OPEN_LOOP_GAIN` | 330 | open loop: PWM units per m/s |
| `PI_KP`, `PI_KI`, `PI_KD` | 32.629, 995.281, 0 | PI gains (tuned for an old BLDC motor; **must be re-tuned**) |
| `PI_INTEGRAL_LIMIT` | 1 | integral clamp |
| `PI_OUTPUT_LIMIT` | 1000 | max controller output (1000 = 100 % PWM) |
| `PI_ZERO_SETPOINT_EPS` | 0.01 | targets below this count as "stop" |

### CAN

| Setting | Current | What it is |
|---|---|---|
| `CAN_ID_DRIVE_CMD` | `0x100` | command ID (agree with the ROS2 side) |
| `CAN_ID_DRIVE_FB_BASE` | `0x100` | feedback ID = this + wheel number |
| `CAN_RX_ACCEPT_IDS` | `{ CAN_ID_DRIVE_CMD }` | IDs let through the hardware filter (up to 8) |
| `CAN_FB_STEERING_RAW` | 0 | value sent in the steering field |
| `CAN_TDC_OFFSET` | DataPrescaler × DataTimeSeg1 | CAN-FD delay compensation (check with a scope) |
| `CAN_BUSOFF_RETRY_DELAY_MS` / `_MAX_RETRIES` | 100 / 10 | bus-off recovery |

### Wheel mapping

| Setting | Current | What it is |
|---|---|---|
| `MOTOR_WHEEL_NO` | `{1,2,3,4}` | which wheel each motor drives. If cables get swapped, change only this. |
| `WHEEL_IS_RIGHT_SIDE` | `{1,1,0,0}` | per wheel number: right side? |
| `WHEEL_DIR_FLIP` | `{-1,-1,1,1}` | per wheel number: direction multiplier |

Bit rates, pins and timer settings are **not** in this file. They come from CubeMX.

---

## 8. Building, flashing and testing

### Firmware (STM32CubeIDE)

1. **File → Import → Existing Projects into Workspace**, and select
   `sterowanie_project/lr_sterowanie_kolami_stm`.
2. **Project → Build Project**.
3. Connect an ST-Link to SWD and use **Run** or **Debug**.

If you regenerate code from the `.ioc` in CubeMX: your changes survive **only**
inside `/* USER CODE BEGIN … END */` blocks and in the separate app files. Never
edit generated code outside those blocks.

### Unit tests (on your PC, no board needed)

The control logic (protocol, kinematics, PI, encoder maths, timeout, buffer) has
no hardware dependencies, so it can be tested on a normal PC:

```bash
cd sterowanie_project/lr_sterowanie_kolami_stm/tests && make
```

You need `gcc` and `make`. The output ends with `N checks, 0 failures`.
`make clean` removes the build files.

---

## 9. First power-on checklist

Do these in order. Keep the **wheels off the ground** until step 6.

1. **BOOT0 pin.** PB8 is both BOOT0 and CAN RX. The CAN transceiver holds RX high,
   so the chip may start its built-in bootloader instead of this firmware. Check
   the option bytes (nSWBOOT0 = 0, nBOOT0 = 1) or confirm the hardware prevents it.
2. **Motor power off.** Flash the board. With a scope or multimeter, check that
   EN pins are inactive and PWM pins are at 0.
3. **CAN.** Connect the bus. You should see `0x101`–`0x104` every 100 ms.
   (At least one other node must be on the bus to acknowledge the frames.)
4. **Motor power on, open loop.** Send mode 0 with a small `lin_x` (for example
   0.2 m/s) several times per second. Every wheel should turn in the **rover's
   forward** direction. If one turns the wrong way, check the motor wiring or
   `MOTOR_DIR_POSITIVE_LEVEL`.
5. **Encoders.** While driving in open loop, each wheel's feedback speed must have
   the **same sign as its target**: negative for right wheels, positive for left
   wheels when driving forward. If one is wrong, flip that motor's `ENCODER_SIGN`.
   **Do not use PID mode until this is right**: a wrong sign makes the motor run away.
6. **Timeout.** Stop sending commands. The motors must stop after about 2.5 s.
7. **PID.** Switch to mode 1 and tune `PI_KP` / `PI_KI` on the real motors.
8. Fill in the real mechanical values (gear ratio, counts, wheel size, width).

---

## 10. Debugging tips

There is no serial log yet (`dbg_log()` is an empty stub until the USB-C debug
path is known). Use **Live Expressions** in the CubeIDE debugger instead:

| Variable | Shows |
|---|---|
| `wheels[i].target_m_s` | target speed of motor i (m/s) |
| `wheels[i].meas_rad_s` | measured wheel speed (rad/s) |
| `wheels[i].enc_total` | encoder counts since boot |
| `drive_mode` | current mode |
| `cmd_timeout.state` | 0 = waiting for first command, 1 = active, 2 = timed out |
| `outputs_enabled` | H-bridges on? |
| `stats.rx_invalid` | rejected command frames |
| `stats.lin_y_ignored` | commands with a non-zero `lin_y` |
| `stats.feedback_dropped` | feedback cycles that couldn't be fully sent |
| `rx_ring.overflows` | frames lost because the receive buffer was full |
| `busoff_count` | how many times the CAN bus went bus-off |

**Common problems**

| Symptom | Likely cause |
|---|---|
| Motors never start | No valid command: wrong ID, not 8 bytes, or bad mode value. Check `stats.rx_invalid`. |
| Motors stop every few seconds | Commands are sent less often than every 2.5 s. |
| Wheel runs away in PID mode | `ENCODER_SIGN` or DIR wrong for that motor. |
| Board keeps resetting | Watchdog: CAN can't recover from bus-off (wiring, termination, bit rate). |
| No feedback on the bus | No other node is acknowledging, or wrong bit rate on the other side. |
| Speed readings are "steppy" | Normal at low speed: one encoder count per 100 ms ≈ 0.12 rad/s with the current placeholders. |

---

## 11. Code map

Everything is under `Core/`. **Pure** modules have no hardware access and are
unit-tested. **HAL** modules touch the hardware.

| File | Kind | Job |
|---|---|---|
| `Inc/app_config.h` | config | all settings |
| `Src/app.c` | HAL | main loop: ties everything together |
| `Src/drv_can.c` | HAL | CAN-FD setup, receive interrupt, send, bus-off recovery |
| `Src/drv_motor.c` | HAL | PWM, DIR and EN pins |
| `Src/drv_encoder.c` | HAL | reads encoder timers |
| `Src/dbg_log.c` | stub | debug log (does nothing yet) |
| `Src/can_proto.c` | pure | decodes the command, builds feedback frames |
| `Src/kinematics.c` | pure | rover speed → wheel speed |
| `Src/drive_ctrl.c` | pure | picks open loop / PI / stop |
| `Src/pi_ctrl.c` | pure | PI controller |
| `Src/encoder_math.c` | pure | encoder wrap-around, counts → rad, rad/s |
| `Src/cmd_timeout.c` | pure | command timeout state machine |
| `Src/ring_buf.c` | pure | buffer between the CAN interrupt and the main loop |
| `Src/main.c` | CubeMX | calls `app_init()` and `app_loop()` (3 added lines) |
| `../tests/` | tests | host unit tests and Makefile |

---

## 12. Known limitations

- **Not tested on hardware.**
- Mechanical values, PI gains and CAN IDs are placeholders.
- H-bridge EN/DIR polarity and encoder direction are not confirmed.
- No status/error frame is sent to the PC; errors are only visible in the debugger.
- No speed ramp: speed changes are applied immediately.
- No encoder fault detection (for example, an unplugged encoder in PID mode).
- CubeMX has AutoRetransmission **off**: a feedback frame that loses bus
  arbitration is not resent. The next one comes 100 ms later.
- The control loop runs at 10 Hz (100 ms), as in the old code. That is slow for
  fine speed control; it can be changed later with `CONTROL_TICKS_PER_STEP`.
