# Rover wheel-controller firmware (STM32G474RET3 + CAN-FD): analysis and implementation plan

Status: **analysis and planning only. No repository file has been modified.**
Date: 2026-10-07. Scope: everything under `/home/revel34/sterowanie_kola_stm32` plus the architecture PDF `Mały Łazik - architektura.drawio.pdf` and the hyperlinks embedded in it.

Marking convention: anything I could not establish from the supplied files is marked **UNKNOWN - REQUIRES CONFIRMATION**. Anything I propose that is not taken from the legacy code or the PDF is marked **PROPOSED**.

---

## 0. Read this first

### Key findings

1. **The repository holds no material for the new board.** `sterowanie_project/` is empty. There is no CubeMX `.ioc` for the G474, no schematic or netlist, no ROS2 package, no CANdle code and no protocol documentation. The only content is `legacy files/` with four STM32CubeIDE projects.
2. **All legacy projects target STM32F446RE (Nucleo-F446RE), HAL F4, bxCAN, classic CAN 2.0A, 500 kbit/s, 11-bit IDs.** None can be reused as-is on FDCAN.
3. **No legacy project does "4 DC motors + encoders + PID".** What exists:
   - `DRIVE_CAN`: one swerve wheel module (BLDC Maxon + stepper steering + SSI absolute encoder). It is the **only** legacy velocity controller (a PI).
   - `GIZ_CAN_...`: four DC H-bridges over CAN, **open loop, no encoders**.
   - `MANI_CAN`: six DC channels, open loop.
   - `POWER_CAN`: 12 switched power circuits (the PDF replaces this with the MAB PDS).
4. **Legacy mechanical constants belong to Rover 3.0, not the small rover.** PDF page 3 says: "Projekty na STM z łazika 3.0 ale z CAN 2.0". The PDF itself is titled "Mały Łazik". Wheel diameter 0.26 m, gear ratio 19, rover width and length (0.713 and 0.789 m) must not be assumed for the new rover.
5. **The PDF says the CAN-FD messages are custom and must be implemented on both the PC side and the STM side** ("będzie potrzebne zaimplementowanie customowych wiadomości zarówno w projekcie na PC jak i na STM"). The message set is therefore co-designed with the ROS2 side. Everything in section C is a **PROPOSAL** that keeps legacy semantics and layouts. The IDs have to be agreed with whoever owns the ROS2 `hardware_interface`.
6. **CAN-FD bus parameters could not be established.** The MAB CANdle pages and the CANdle-SDK repo are blocked, 404, or do not state them. The only documented facts are:
   - the PDS has a CAN BAUDRATE property (0x21) with enum values 1M / 2M / 5M / 8M;
   - the default PDS CAN ID is 100 (0x64).

   Nominal bitrate, data bitrate, BRS, and whether CANdle forwards arbitrary custom frames are all **UNKNOWN - REQUIRES CONFIRMATION**.
7. **Pin-level hardware is unknown.** The board is an EasyEDA project (linked from the PDF, login required). I have not seen it.

### Decisions I need from you before any code

| # | Decision | Why it matters |
|---|----------|----------------|
| D1 | Does ROS2 send **per-wheel target velocities** (natural for `ros2_control` / `hardware_interface`), or a **cmd_vel-style frame** like the legacy `0x100` with kinematics on the STM? | Legacy put kinematics on the STM (swerve). The new board has 4 DC motors and (per the PDF) no steering. Variant R (per-wheel) is my recommendation, Variant L (legacy-like) is the alternative. Both are specified in C2. |
| D2 | Wheel numbering and sign convention for the new rover | Legacy numbering is established (section B7). Whether it carries over is not. |
| D3 | CANdle bus: nominal bitrate, data bitrate, BRS on/off, 11-bit vs 29-bit IDs, can CANdle send/receive arbitrary IDs? | Mandatory FDCAN configuration. |
| D4 | Command-timeout duration and recovery policy | Legacy has three different values (2.5 s / 1.5 s / 1.0 s, one disabled). I will not pick one for you. |
| D5 | Hardware export (EasyEDA netlist/PDF or CubeMX `.ioc`) | Pin map, H-bridge type, encoder interface, fault pins. |
| D6 | Motor/encoder/wheel constants of the **small rover**, and the gain-tuning plan | Legacy PI gains were tuned for a BLDC and will not transfer to DC motors. |

---

## A. Repository analysis

### A1. Structure found

```
sterowanie_kola_stm32/
├── legacy files/
│   ├── DRIVE_CAN/                              CubeIDE project "CAN_send_test", F446RET6
│   ├── GIZ_CAN_DZIALA_WSZYSTKO_LED_CZERWONY/   same project name, F446RET6
│   ├── MANI_CAN/                               same project name, F446RET6
│   └── POWER_CAN/                              same project name, F446RET6
└── sterowanie_project/                         EMPTY
```

Each legacy project has `Core/Src`, `Core/Inc`, a `.ioc`, HAL F4 V1.28.3 drivers, and build output under `Debug/`. `Debug/`, `.metadata*` and `Drivers/` are noise and were ignored.

### A2. Classification of what exists

| Item | Role | Authority |
|------|------|-----------|
| Architecture PDF (4 pages) | System architecture, power path, board block diagram, STM software task list | **Authoritative** (hardware/system) |
| `DRIVE_CAN` (`main.c`, `ALL_STRUCT.[ch]`, `BLDC_MOTOR.[ch]`, `ABS_ENCODER.[ch]`, `.ioc`) | Wheel command/feedback protocol, velocity PI, timeout, unit conventions, wheel numbering | **Reference** for semantics. Hardware is Rover 3.0 swerve. |
| `GIZ_CAN_...` (`main.c`, `.ioc`) | Four DC H-bridges via CAN (PWM + DIR), 0x200 to 0x205, 0x210 | Reference for the DC-motor driver style (open loop only) |
| `MANI_CAN` | Six-channel open-loop DC, ID 0x300/0x301 | Reference (ID allocation, timeout) |
| `POWER_CAN` | Switchable circuits, ID 0x50/0x051 | **Not relevant.** The PDF replaces it with MAB PDS. |
| Anything for ROS2, CANdle, `hardware_interface`, new-board CubeMX | – | **Not present** |

### A3. What the PDF defines

Page 1: system chain, power path, board.
- Control chain: Laptop/phone ↔ RF 2.4/5 GHz ↔ WiFi Bullet ↔ Ethernet ↔ **PC/Jetson ↔ ROS2 ↔ CANdle Lib ↔ USB ↔ CANdle (MAB driver) ↔ CAN-FD ↔ PDS and the motor PCB**. The CAN-FD bus is shared by CANdle, the MAB PDS and the PCB.
- Power path: Battery → E-STOP power switch → PDS → DC/DC → Jetson and WiFi Bullet. PDS power also feeds the PCB (red line). The **E-STOP is a hardware cut in the power path, not a CAN message.**

Page 2: PDS documentation (IC_ENABLE / PS_ENABLE / BR_ENABLE switches, current and voltage measurement, motor bus with brake resistor). The firmware does not control the PDS (hard rule in your brief, and nothing in the PDF requires it).

Page 3: ROS2 and STM software tasks.
- ROS2 packages: URDF description, ported CANdle library, ROS2 Control, `hardware_interface` ("tłumaczenie wiadomości na CAN"), a custom package sending CAN messages to the PCB, and a bringup/launch package.
- "Projekt na STM, Software", four items:
  1. CAN-FD communication needs custom messages on both PC and STM.
  2. Encoder readings, speed conversion, PID implementation.
  3. Motor control.
  4. Sending feedback about speed, position etc. as "joint_states" for ROS2.

Page 4: PCB block diagram. 24 V in; DC/DC "probably 5 V" for STM, encoders etc.; CAN-FD transceiver ↔ STM32 (CAN TX / CAN RX); USB-C "debugging, programming etc." connected to the STM32; "IC for Encoders if needed?" with "some protocol if needed" to the STM32; DC Motors Controllers block with **H-Bridge 1 to 4** driving the motors; motor encoder readings fed back. Several "Safety Protection" blocks. Note: "Płytki zrobione do tej pory w EASYEda."

### A4. External references embedded in the PDF and whether I could read them

| Reference | Result |
|-----------|--------|
| `mabrobotics.github.io/.../PDS/overview.html` | Read. Setup only, no frame details. |
| `.../PDS/communication.html` | Read. Gave the facts in section 0 (finding 6). |
| `.../CANdle/intro.html` | Read. Role description only. Says CANdle is likely too slow for 500 Hz+ loops. No bitrate. |
| `github.com/mabrobotics/CANdle-SDK/tree/main/candlelib` | **Blocked by robots.txt** |
| `Legendary-Rover-3-0/Legendary_3.0_Manipulator` | **404** (private or moved) |
| `Legendary-Rover-Team/Kody-na-STM32-Microros-oraz-CAN` | **404** |
| Nextcloud ROS2 courses; EasyEDA team project | Login required. Not reachable. |

Consequence: the ROS2-side message definitions and the board schematic have not been seen.

---

## B. Legacy protocol analysis

All values below are read directly from the code. Each project's `.ioc` (CAN1 BS1 = 14 TQ, BS2 = 6 TQ, prescaler 4 on 42 MHz APB1, TQ = 95.24 ns) gives 500 kbit/s with a 71.4 % sample point.

### B1. Transport

| Item | Legacy value |
|------|--------------|
| Protocol | Classic CAN 2.0A, standard 11-bit IDs, data frames, DLC 8 on all TX frames except POWER status (DLC 2) |
| Bitrate | 500 kbit/s (all four projects) |
| Endianness | Little-endian (`memcpy`, `int16_t*` casts on Cortex-M4; explicit LSB-first shifts in GIZ 0x203/0x204) |
| RX | Filter bank, FIFO0, `CAN_IT_RX_FIFO0_MSG_PENDING` callback loops until FIFO empty |
| TX | Polled from a timer ISR, first free mailbox (DRIVE aborts all mailboxes if none is free) |
| Auto bus-off / retransmit | DRIVE and MANI: AutoBusOff off, AutoRetransmission on. GIZ: AutoBusOff on, AutoRetransmission off. POWER: AutoBusOff off, AutoRetransmission on. |

### B2. Message table (all IDs found)

| ID | Dir (as seen by STM) | Project | Name / purpose | DLC | Payload |
|----|----------------------|---------|----------------|-----|---------|
| 0x100 | RX | DRIVE | Wheel command (cmd_vel-like) | 8 | 4 × int16 LE, each ÷ 1000: `lin_x`, `lin_y`, `lin_z`, `ang_z` (see B3) |
| 0x101..0x104 (= 0x100 + wheel no.) | TX | DRIVE | Wheel feedback (joint state) | 8 | see B4 |
| 0x200 | RX | GIZ | RGB LED | not checked | R, G, B (uint8) = bytes 0..2 |
| 0x201 | RX | GIZ | 4 × DC H-bridge, open loop | not checked | `int8` × 4, range −100..100, bytes 0..3 → `motor_1..4` |
| 0x202 | RX | GIZ | 4 × RC servo | not checked | `uint8` 0..180 (clamped), bytes 0..3 |
| 0x203 | RX | GIZ | TMC stepper command | not checked (8 used) | id(1), steps int32 LE(4), dir(1), speed reload uint16 LE(2) |
| 0x204 | RX | GIZ | UART bus-servo | not checked | id(1), angle uint16 LE(2), clamped to 170 |
| 0x205 | RX | GIZ | "future" | – | unused |
| 0x210 | TX | GIZ | Heartbeat | 8 | all 0xFF filler, every ~107 ms |
| 0x300 | RX | MANI | 6 × DC joint, open loop | 8 | `int8` × 6 (−100..100), remapped: `PwmJoint1 = d[4]`, `2 = d[5]`, `3 = d[3]`, `4 = d[2]`, `5 = d[1]`, `6 = d[0]` |
| 0x301 | TX | MANI | Heartbeat | 8 | 0xFF filler, ~107 ms |
| 0x50 | RX | POWER | 12-circuit switch command | ≥ 3 | 2 bits per circuit; byte 3 = 0xAA means global OFF |
| 0x051 | TX | POWER | Circuit status | 2 | 1 bit per circuit, every 100 ms |

Notes:
- In `MANI_CAN` a comment mentions "0x210" for feedback while the code sets `0x301`. The code is taken as authoritative.
- MAB PDS default CAN ID is 0x64 (documentation). That does not collide with the legacy IDs above.
- MAB MD-controller default IDs: **UNKNOWN - REQUIRES CONFIRMATION**.

### B3. DRIVE command 0x100, decode semantics

Payload (int16 LE ÷ 1000.0): `lin_x`, `lin_y`, `lin_z`, `ang_z`.

`lin_z` is **not a velocity**. It is a mode selector (`ReadExtraParametres`):

| lin_z (±0.1 window) | Mode | Behaviour |
|---------------------|------|-----------|
| 0 (default for anything unrecognised) | `DRIVE_WITHOUT_PID` | open loop: `motor_signal = setpoint_velocity × 330` (PWM counts, **not clamped**) |
| 1 | `DRIVE_WITH_PID` | closed-loop PI (B6) |
| 2 | `X_STOP` | `motor_signal = 0`, wheels steer to a "X" angle |

Velocity commands are interpreted per wheel with sign flags from the per-wheel table `tab[]` (B7), and swerve kinematics: forward, crab, diagonal, spin-in-place, arc (uses rover width 0.713 m and length 0.789 m). `VelocityLimit(v, 100)` makes the velocity limit effectively 100 m/s (non-functional as a safety limit). The setpoint unit is m/s (comments).

All wheels receive the **same** frame. Each wheel derives its own target from its own `tab[]`.

### B4. DRIVE feedback 0x100 + wheel number

| Bytes | Type | Content | Scaling |
|-------|------|---------|---------|
| 0..1 | int16 LE | steering angle, normalised to −180..+180° then converted to rad | × 10000 (rad) |
| 2..5 | int32 LE | wheel position = accumulated rotation, rad | × 10000 |
| 6..7 | int16 LE | wheel velocity, rad/s (rev/s × 2π) | × 1000 |

Period: 100 ms (every 10th tick of a 10 ms timer). Ranges implied by the types: velocity ±32.767 rad/s; position ±214 748 rad. Legacy code zeroes the accumulated position when |position| ≥ 200 000 rad (a discontinuity the PC side would see).

Position source: encoder/Hall counts accumulated since boot; no persistent zero reference for the wheel (steering has an absolute encoder with a stored `zero_angle`, not relevant for DC wheels).

### B5. Timing in DRIVE

| Function | Period |
|----------|--------|
| Base tick (TIM10, 84 MHz/840/1000) | 10 ms |
| PID + velocity estimate + PWM update + feedback TX | 100 ms (10 Hz) |
| Steering/absolute encoder + stepper | 50 ms |
| Command timeout counter | counted in 10 ms ticks |

Everything runs in the TIM10 ISR. The CAN RX callback also runs the kinematics, including `atan2f`/`sqrt`, in interrupt context.

### B6. Velocity control (DRIVE)

| Item | Legacy value |
|------|--------------|
| Structure | **PI** (derivative term is computed but `D = 0.0`) |
| Kp | 32.629 (the commented-out variants show 12.629 as an alternative) |
| Ki | 995.281 (comment shows 695 as an alternative) |
| Kd | 0.0 |
| Error | `setpoint_velocity − measured LinearSpeed`, both in **m/s** |
| Control period | `MOTOR_SETTING_TIME` = 100 ms; Ki/Kd integrate with explicit dt, so the gains are in continuous-time form |
| Output | PWM counts, saturated to **±1000** (TIM8 ARR = 999 at 12 kHz, so 1000 = 100 %) |
| Integral limit | clamped to **±1** (i.e. I-term max ±995 counts) |
| Anti-windup | integral accumulates only while the previous output is inside (−1000, 1000); when saturated it is frozen, and zeroed if measured speed is exactly 0 |
| Zero-speed handling | integral reset if `|setpoint| < 0.01` and `|speed| < 0.1` m/s; output forced to 0 if `|setpoint| < 0.01` |
| Sign handling | `|signal|` goes to PWM; direction pin `BLDC_FR` = RESET when signal > 0, SET otherwise |
| Brake | `BLDC_BK` pin SET (brake) when signal == 0, RESET otherwise |
| Ramp | A ramp structure exists, but `GenerateVelTab` is commented out and `SpeedRamp` is never called. **No ramp is active.** |
| Tuning origin | Maxon BLDC with gearbox ratio 19 (Rover 3.0) |

### B7. Encoder and mechanical constants in legacy

| Constant | Value | Where | Notes |
|----------|-------|-------|-------|
| `GEAR_RATIO` | 19 | ALL_STRUCT.h | Rover 3.0 BLDC gearbox |
| `PG_RESOLUTION` | 42 | ALL_STRUCT.h | used only by the unused `ReadVelocityFromPGPin` |
| `ENCODER_RESOLUTION` | 4096 | ALL_STRUCT.h | unused (the incremental-encoder velocity function is commented out) |
| `WHEEL_DIAMETER` | 0.26 m | ALL_STRUCT.h | travel = π·D |
| `ROVER_WIDTH`, `ROVER_LENGTH` | 0.713 m, 0.789 m | ALL_STRUCT.h | used in kinematics |
| Hall/encoder counts per motor rev actually used | **21** for positive delta, **28** for negative delta, 28 in the position integral | BLDC_MOTOR.c | asymmetric. UNKNOWN - REQUIRES CONFIRMATION whether calibration or defect. |
| Counter | TIM3, TI12 encoder mode, 16-bit, no filter | tim.c | |
| Glitch filter | `|rev/s| > 4` → velocity forced to 0 | BLDC_MOTOR.c | |

Wheel numbering is established by comments in `DRIVE_CAN/main.c` (the per-wheel `tab[]` arrays) and by the arc-mode `switch` labels:

| Wheel no. (`tab[4]`, also joint_states name `tab[5]`) | Position | Legacy sign flags `tab[1]`, `tab[2]`, `tab[3]` (x, y, z_ang) |
|---|---|---|
| 1 | right-front (PP) | −1, −1, −1 |
| 2 | right-rear (PT) | −1, −1, +1 |
| 3 | left-front (LP) | +1, +1, +1 |
| 4 | left-rear (LT) | +1, +1, −1 |

The compiled build in the folder is wheel 4 (left-rear): `tab = {-125, 1, 1, -1, 4, 4, ...}`. Whether this numbering carries over to the new board: **UNKNOWN - REQUIRES CONFIRMATION (D2)**.

### B8. Failsafe in legacy

| Project | Timeout | Action |
|---------|---------|--------|
| DRIVE | 250 ticks × 10 ms = **2.5 s** | zero all commanded velocities (once), PID then outputs 0, brake pin set. Resumes automatically on the next 0x100 frame. |
| MANI | 1500 ms | all PWM joints → 0, auto-resume |
| GIZ | 1000 ms | **disabled** ("na prośbę użytkownika, do testów") |
| POWER | none | – |

`emergency_stop()` exists in DRIVE but is never called. No heartbeat message from the PC is defined; the command stream itself is the heartbeat.

### B9. Legacy defects I will not carry over

1. DRIVE sets "safe PWM/brake" in `USER CODE Init` **before** `MX_GPIO_Init`/`MX_TIM8_Init`. At that point `htim8.Instance` is still NULL, so the intended safe-start does nothing reliable.
2. No `HAL_CAN_ErrorCallback`, bus-off handler or recovery (CAN notifications are enabled but no callback is implemented). With AutoBusOff off, a bus-off is never recovered.
3. Kinematics (trigonometry) in the CAN RX ISR; PID and CAN TX in the timer ISR.
4. `BLDC_EN` (PA12) is configured but never driven.
5. Open-loop mode has no output clamp; `VelocityLimit(..., 100)` is not a real limit.
6. Position wrap/reset at 200 000 rad is silent.
7. GIZ DC outputs: PWM = 100 Hz (TIM prescaler 83, ARR 9999 on 84 MHz) and `.ioc` pin labels do not match the code (index 0 is driven on pins labelled H4D / HBridge4_PWM, etc.). `motor_4` uses PC5 / TIM4_CH1, which is not in the `.ioc`.
8. `ReadVelocityFromPGPin_Hall` computes a second pulse delta and then forces it to 0; the PG counter is never incremented anywhere.

---

## C. New CAN-FD mapping (PROPOSED)

Principle: keep legacy semantics, units, scaling, endianness and wheel feedback layout; change only the transport to FDCAN. IDs and layouts below are **proposals** that need sign-off from the ROS2 side (the PDF requires both sides to implement them).

### C1. Frame-level parameters

| Parameter | Value |
|-----------|-------|
| Peripheral | STM32G474 FDCAN (instance chosen by the schematic: **UNKNOWN - REQUIRES CONFIRMATION**), external CAN-FD transceiver |
| Frame format | CAN-FD (FDF = 1) for application frames. Classic CAN is not used. |
| Nominal bitrate | **UNKNOWN - REQUIRES CONFIRMATION** |
| Data bitrate | **UNKNOWN - REQUIRES CONFIRMATION** (PDS doc lists 1M / 2M / 5M / 8M as selectable values; unclear which phase they refer to) |
| BRS | **UNKNOWN - REQUIRES CONFIRMATION** |
| ID type | 11-bit standard (legacy). **UNKNOWN - REQUIRES CONFIRMATION** whether the CANdle bus needs 29-bit. |
| DLC | Legacy-sized payloads, DLC = 8 for command and feedback. FD allows more (≤ 64 B) but legacy layouts are 8 B, so I start there. |
| Endianness | Little-endian |
| Sample point / SJW / TDC | derived from the bitrates above; transmitter-delay compensation will be needed at high data rates. **UNKNOWN - REQUIRES CONFIRMATION** |
| Termination | PDS control board has built-in termination; the PCB's termination: **UNKNOWN - REQUIRES CONFIRMATION** |

### C2. Messages

**RX: command.** Two alternatives, decision D1.

*Variant R (recommended): per-wheel velocity targets.*
Rationale: ROS2 Control and `hardware_interface` natively produce per-joint velocity commands and expect per-joint states; kinematics then stays in ROS2 (where the URDF lives). The STM does what the PDF's task list says: encoders → speed → PID → motors → feedback. This does depart from legacy, where the STM did kinematics, so I will not do it without your approval.

| Field | Proposal |
|-------|----------|
| ID | **UNKNOWN - REQUIRES CONFIRMATION** (suggest 0x100, as in legacy, so existing PC tooling keeps working) |
| Payload (DLC 8) | 4 × int16 LE: target wheel angular velocity of wheels 1..4, rad/s × 1000 (same unit and scale as the legacy feedback velocity) |
| Mode selector | The legacy `lin_z` mode (0 open loop / 1 PID / 2 stop) has no slot here. **UNKNOWN - REQUIRES CONFIRMATION** whether it must be kept; if so it takes DLC > 8 (FD allows it) or a second message. |

*Variant L (legacy-faithful): cmd_vel-style frame with kinematics on the STM.*
Payload as in B3: 4 × int16 LE ÷ 1000 (`lin_x`, `lin_y`, mode, `ang_z`). The STM needs 4 per-wheel sign tables and the kinematics for 4 wheels. Kinematics constants of the small rover are **UNKNOWN - REQUIRES CONFIRMATION**, and without steering the swerve modes (crab, diagonal, arc) may not apply.

**TX: wheel feedback.** Preserve the legacy layout, one frame per wheel:

| Field | Value |
|-------|-------|
| ID | 0x101..0x104 = 0x100 + wheel no. (legacy convention); **UNKNOWN - REQUIRES CONFIRMATION** for the new bus |
| DLC | 8 |
| Bytes 0..1 | int16: steering angle × 10000 rad. Not applicable to fixed wheels, so reserved = 0. **UNKNOWN - REQUIRES CONFIRMATION** whether the PC decoder can ignore it. |
| Bytes 2..5 | int32: wheel position, rad × 10000 |
| Bytes 6..7 | int16: wheel velocity, rad/s × 1000 |
| Period | Legacy 100 ms. Required rate for ROS2 `joint_states`: **UNKNOWN - REQUIRES CONFIRMATION**. I will make it a compile-time constant. |

A packed single-frame alternative (all four wheels in one 64-byte FD frame) is possible but is a protocol redesign, so it is **not** my default.

**TX: status / fault frame.** No legacy equivalent. The PDF asks for "relevant status and errors". A small frame (state, per-wheel fault bitmask, timeout flag, CAN error counters) is PROPOSED. ID and layout: **UNKNOWN - REQUIRES CONFIRMATION**.

**PID parameters / configuration over CAN.** None exists in legacy (gains are compile-time constants). v1 keeps them as compile-time constants. Runtime tuning messages are optional later work.

ID-collision list for allocation: 0x050, 0x051, 0x064 (PDS default), 0x100..0x104, 0x200..0x205, 0x210, 0x300, 0x301, plus MD80 default IDs (UNKNOWN).

### C3. Filtering, FIFO, interrupts, errors

| Topic | Proposal |
|-------|----------|
| Global filter | Reject non-matching standard and extended frames; reject remote frames |
| Standard filters | One exact-ID (or dual-ID) filter per accepted RX ID → RX FIFO 0 |
| FIFO | RX FIFO 0 for commands. FIFO 1 reserved for future configuration messages. |
| TX | TX FIFO mode (strict order) with the status frame at a different ID priority |
| RX interrupt | Rx FIFO 0 new-message interrupt. The ISR only copies the frame into a small ring buffer and sets a flag. No decoding, no trigonometry. |
| Error handling | Enable bus-off and protocol-error interrupts. On bus-off: outputs → safe state first, then request recovery (FDCAN stays in INIT after bus-off until software restarts it). Count and report in the status frame. |
| Bus-off recovery | Restart FDCAN after the mandatory idle period, rate-limited; motors stay in safe state until a valid command is received again |

---

## D. STM32G474RET3 peripheral allocation

I did not invent pins. Everything pin-level needs the schematic (D5).

| Function | Candidate resource | Status |
|----------|--------------------|--------|
| FDCAN | one of FDCAN1/2/3 + external transceiver (TX/RX pins per PDF page 4) | instance and pins **UNKNOWN - REQUIRES CONFIRMATION** |
| FDCAN kernel clock | HSE / PLLQ / PCLK1 | **UNKNOWN - REQUIRES CONFIRMATION** (crystal not known) |
| Encoders ×4 | four encoder-capable timers (the G474 has several: TIM1, 2, 3, 4, 5, 8). Prefer the 32-bit TIM2/TIM5 for the two slowest-count channels. | pins **UNKNOWN - REQUIRES CONFIRMATION**. The PDF shows an "IC for encoders if needed" with a "protocol": quadrature on timers vs an external IC is **UNKNOWN - REQUIRES CONFIRMATION**. |
| PWM ×4 | advanced timers TIM1/TIM8 or general-purpose timers; one channel per H-bridge | **PROPOSED**: one timer per pair so frequency is common; pins **UNKNOWN - REQUIRES CONFIRMATION** |
| Motor direction / enable | GPIO per H-bridge (PWM+DIR as in GIZ, or IN1/IN2). H-bridge IC and interface type **UNKNOWN - REQUIRES CONFIRMATION** | |
| Fault inputs (H-bridge fault/OC) | **UNKNOWN - REQUIRES CONFIRMATION**. None will be assumed to exist. | |
| Emergency/safety input | The PDF shows E-STOP in the power path only. A signal to the MCU: **UNKNOWN - REQUIRES CONFIRMATION**. | |
| Control-loop timer | basic timer (TIM6 or TIM7), update interrupt, fixed period | **PROPOSED** |
| Time base | SysTick (1 ms) for timeouts; DWT/TIM for profiling | **PROPOSED** |
| USB-C | The PDF says "debugging, programming etc." Whether this is native USB (DFU/CDC) or SWD/ST-Link: **UNKNOWN - REQUIRES CONFIRMATION** | kept separate from the runtime protocol |
| PWM frequency | **UNKNOWN - REQUIRES CONFIRMATION** (depends on the H-bridge; legacy used 12 kHz for the BLDC driver and 100 Hz for GIZ DC) | |

---

## E. Encoder calculation design

Symbols: `CPR` encoder cycles per revolution at the motor shaft, `X` counting factor (x1/x2/x4), `G` gearbox ratio (motor : wheel), `r` wheel radius, `Δt` sample period, `c` raw counter.

| Quantity | Formula |
|----------|---------|
| Counts per wheel rev | `N = CPR · X · G` |
| Count delta (wrap-safe) | `Δc = (int16_t)(c_now − c_prev)` for 16-bit timers, `(int32_t)` for 32-bit. Signed subtraction makes wrap-around correct without special cases. |
| Wheel angle (position) | `θ = 2π · c_total / N`, `c_total` accumulated in `int64_t` |
| Wheel angular velocity | `ω = 2π · Δc / (N · Δt)` [rad/s] |
| Linear wheel velocity | `v = ω · r` [m/s] |

Legacy equivalent: `rev/s = Δ / (counts · GEAR_RATIO · Δt)`, `v = rev/s · π · D`, position in rad (`Δ/(counts·GEAR)·2π`). The same convention is kept.

| Item | Status |
|------|--------|
| Encoder type, CPR, counting mode, `G`, wheel diameter of the small rover | **UNKNOWN - REQUIRES CONFIRMATION** (legacy values 19 and 0.26 m are Rover 3.0) |
| Sign convention | Legacy: positive delta = forward, `signal > 0` → DIR low. New: per-channel `encoder_sign` and `motor_sign` constants. **UNKNOWN - REQUIRES CONFIRMATION** |
| Zero / reference | Legacy: position starts at 0 at boot (incremental). Kept. |
| Position range | Feedback is int32 × 10000 rad, so ±214 748 rad. Firmware accumulates in 64 bits and defines wrap behaviour explicitly instead of the legacy silent reset at 200 000 rad. **UNKNOWN - REQUIRES CONFIRMATION** with the PC side. |
| Velocity quantisation | One count at the control period gives `ω_min = 2π/(N·Δt)`. It cannot be quantified until `N` is known. A low-pass filter on `ω` is PROPOSED if needed. |

---

## F. Four-wheel control architecture

One data structure per wheel, one control function used four times, table-driven hardware mapping.

```c
typedef struct {            /* configuration: const, per channel */
    timer/channel for encoder; timer/channel + DIR/EN for PWM;
    int8_t encoder_sign, motor_sign; ...
} wheel_hw_t;

typedef struct {            /* runtime state */
    float    target_w;      /* target wheel velocity (from command)        */
    float    meas_w;        /* measured wheel velocity                      */
    int64_t  enc_total;     /* encoder position, counts                     */
    pid_t    pid;           /* integrator, prev error, last output          */
    float    duty;          /* controller output, normalised -1..+1         */
    uint32_t last_cmd_ms;   /* communication timeout state                  */
    uint16_t fault;         /* fault bitmask                                */
    wheel_state_t state;    /* SAFE / ACTIVE / FAULT                        */
} wheel_t;

static wheel_t wheel[4];
void wheel_control_step(wheel_t *w, const wheel_hw_t *hw, float dt);  /* called 4x */
```

Strict separation:
- target velocity (`target_w`) ← command layer only
- measured velocity (`meas_w`) ← encoder/estimator only
- PID state (`pid`) ← controller only
- controller output (`duty`) ← controller only
- driver output (PWM compare, DIR, EN) ← motor driver only, derived from `duty` and the safety state

Wheel numbering: 1..4 as in legacy (RF, RR, LF, LR) as an **index** only; the mapping to the physical wheel is **UNKNOWN - REQUIRES CONFIRMATION (D2)**. Internal array index = wheel number − 1.

---

## G. PID / control-loop design

Keep the legacy controller semantics (PI, Kd = 0), the explicit-dt integral, the integral clamp, the conditional-integration anti-windup, the zero-setpoint handling and the output saturation (all listed in B6). No other change without justification.

| Item | Proposal |
|------|----------|
| Control law | `u = Kp·e + Ki·∫e dt + Kd·de/dt`, `Kd = 0` by default (derivative code kept) |
| Control frequency | compile-time constant. **UNKNOWN - REQUIRES CONFIRMATION** for the final value. Legacy was 10 Hz. PROPOSED default 100 Hz with the encoder window equal to the control period. Justification: the gains are continuous-time (explicit dt), so the law ports unchanged, and a 100 ms loop is slow for a drive wheel. This requires re-tuning anyway. |
| Kp, Ki, Kd | legacy 32.629 / 995.281 / 0 are **BLDC Rover-3.0 values with error in m/s and output in ±1000 PWM counts**. They are reference only. New gains for the DC motors: **UNKNOWN - REQUIRES CONFIRMATION** (must be tuned on the real motors). |
| Units | Legacy internal unit is m/s. If D1 = Variant R, the natural internal unit is rad/s (same as command and feedback). Conversion `Kp_rad = Kp_ms · r`, `Ki_rad = Ki_ms · r`. I will document this explicitly and not change units silently. |
| Integral limit | legacy ±1 (m). Rescaled with the unit; **UNKNOWN - REQUIRES CONFIRMATION** for the new motors. |
| Output limit | legacy ±1000 counts of ARR. New: normalised ±1.0 duty, mapped to ARR. **UNKNOWN - REQUIRES CONFIRMATION** for any lower limit (supply is 24 V per PDF; motor rating unknown). |
| Anti-windup | as legacy: integrate only when the previous output is not saturated, plus the zero-speed reset |
| Zero-speed | as legacy: if `|target| < ε` the output is forced to 0 and the integrator reset; the brake/disable action depends on the H-bridge (UNKNOWN) |
| Open-loop mode | Legacy mode 0 (`signal = setpoint · 330`, no clamp). If kept (D1), it gets a clamp. This is a deliberate fix. |
| Ramp | None in legacy (code is dead). **No ramp** in v1. A slew limiter may be added if you want one. |

---

## H. CAN-FD receive and transmit flow

```
 CANdle ──CAN-FD──▶ FDCAN RX FIFO0
                         │  (HW acceptance filter: only configured IDs)
                         ▼
                 FDCAN RX ISR  (short)
                   copy frame → ring buffer (SPSC), set flag, note HAL tick
                         │
                         ▼
          main loop: can_proto_decode()  (pure function)
            · check ID, DLC, payload length
            · convert int16 → physical units
            · range-check  → reject / clamp (see I)
            · write target_w[i], last_cmd_ms (atomic, interrupts masked briefly)
                         │
   TIMx ISR (fixed period) ▼
     for each wheel: read encoder → estimate ω → PID → duty → motor driver
     safety check: timeout, faults → force SAFE
                         │
   main loop / scheduler   ▼
     telemetry: pack 4 × feedback frames (+ status) → FDCAN TX FIFO, BRS per C1
```

- Interrupts: FDCAN RX (copy only), FDCAN error/bus-off (set safe state, flag), control-loop timer (deterministic period). Nothing else.
- Hardware timer: control tick; encoder counting is done by timer hardware, no software edge counting.
- Main loop: protocol decode, status/telemetry scheduling, bus-off recovery state machine, debug output.
- RTOS: **not required** for this workload. A bare-metal superloop plus one timer ISR is enough and keeps latency deterministic.
- No dynamic memory allocation anywhere.

---

## I. Safety / watchdog behaviour

| Topic | Behaviour |
|-------|-----------|
| Startup | All motor outputs low/disabled from the first instruction after reset (also a **hardware** requirement: pull-down on PWM/DIR/EN lines, **UNKNOWN - REQUIRES CONFIRMATION** in the schematic). State = SAFE. No output until the first valid command arrives. |
| Command timeout | Duration: **UNKNOWN - REQUIRES CONFIRMATION (D4)**. Legacy has 2.5 s (DRIVE), 1.5 s (MANI), 1.0 s (GIZ, disabled). I will not choose one; it will be a named constant. |
| What is "lost" | No *valid* command frame (correct ID, DLC, payload in range) within the timeout. Frames with a wrong ID or DLC do not reset the watchdog (legacy GIZ and MANI reset it on any frame that passed the filter, without checking DLC). |
| On timeout | All 4 wheels: `target_w = 0`, integrator reset, `duty = 0`, outputs disabled/braked per H-bridge capability, state = SAFE, timeout flag in status frame. Legacy behaviour was the same except it only zeroed the setpoint. |
| Recovery | Legacy auto-resumes on the next frame. PROPOSED: the same (auto-resume), because it is the legacy behaviour and you have not asked otherwise. A stricter policy (require a zero command first, or an explicit enable) is a one-line option. **UNKNOWN - REQUIRES CONFIRMATION**. |
| Invalid CAN ID | dropped by hardware filter; if it reaches software, ignored and counted |
| Malformed payload (wrong DLC) | frame rejected, counter incremented, watchdog **not** reset |
| Out-of-range command | clamp to the configured limit and flag. Rejection vs clamping: **UNKNOWN - REQUIRES CONFIRMATION**. Limits: **UNKNOWN - REQUIRES CONFIRMATION**. |
| CAN-FD error | counted, reported in status; error-passive does not stop motors by itself (the timeout will) |
| Bus-off | immediate SAFE, then rate-limited recovery (C3), status flag |
| Encoder failure / implausible reading | Plausibility test: measured |ω| above a configurable limit, or no counts for N periods while duty is high (stall/disconnect). Reaction: that wheel → FAULT, duty = 0. Limits **UNKNOWN - REQUIRES CONFIRMATION**. Closed-loop with a dead encoder could run away, so a fallback to open loop is not allowed. |
| Controller saturation | anti-windup (G); if saturated for longer than a configurable time with no motion, raise a stall flag |
| Motor-driver fault | handled only if a fault pin exists in hardware. **None assumed.** |
| E-STOP | hardware power cut (PDF). The firmware only needs to survive power loss/brown-out cleanly and start in SAFE. No CAN e-stop message is invented. |
| Independent watchdog | PROPOSED: enable the IWDG so a firmware hang also leads to a reset into SAFE (outputs low by hardware pull-downs). |

---

## J. Proposed firmware file / module structure

The CubeMX-generated tree goes in `sterowanie_project/Core/`. Application code goes beside it. Pure-logic modules have no HAL dependency, so they can be unit-tested on a PC.

```
sterowanie_project/
├── sterowanie.ioc                (created only after D5)
├── Core/ ...                     (generated)
└── App/
    ├── config.h                  all constants; UNKNOWN items are #error until set
    ├── app.c/.h                  init sequence, state machine, main loop
    ├── can_fd.c/.h               FDCAN init, filters, RX ISR → ring buffer, TX, bus-off
    ├── can_proto.c/.h            IDs, pack/unpack, range checks   (pure)
    ├── encoder.c/.h              timer encoder wrapper, wrap-safe deltas
    ├── velocity.c/.h             counts → rad, rad/s, m/s         (pure)
    ├── pid.c/.h                  PI(D) with anti-windup           (pure)
    ├── motor.c/.h                PWM/DIR/EN output per channel, safe-off
    ├── wheel.c/.h                wheel_t, wheel_control_step(), 4-channel table
    └── safety.c/.h               timeouts, fault latching, state transitions
Tests/                            host-side unit tests for pure modules
```

Telemetry packing lives in `can_proto.c` and its scheduling in `app.c`. No extra layers.

---

## K. Initialization sequence

1. Reset: GPIO for all motor lines set to the safe level first (hardware pull-downs assumed, see I).
2. `HAL_Init`, clock configuration (source **UNKNOWN - REQUIRES CONFIRMATION**: HSE crystal?).
3. GPIO init (all motor outputs low/disabled, encoder inputs).
4. Encoder timers configured and started, counters zeroed.
5. PWM timers configured with duty = 0; **start PWM only after step 8**, outputs still disabled.
6. FDCAN configured (bitrates from C1, TDC if needed, message RAM, RX element size for 8 B payloads).
7. FDCAN filters (C3) and interrupts configured, then FDCAN started.
8. Controller state zeroed (`target_w = 0`, PID reset), state = SAFE, watchdog started.
9. Control-loop timer started.
10. First valid command → ACTIVE. No motion before that.

---

## L. Test plan

Each item can be checked without the full rover.

| # | Test | Method | Pass criterion |
|---|------|--------|----------------|
| 1 | CAN-FD reception | FDCAN internal loopback first, then a CAN-FD adapter or CANdle sending known frames | RX counter and decoded values match the sent frames; wrong ID/DLC rejected |
| 2 | Command decoding | host-side unit tests of `can_proto` with legacy-derived vectors (e.g. raw int16 1000 → 1.000) | exact conversions, range checks |
| 3 | Encoder counting | drive the encoder inputs from a signal generator or a second timer producing quadrature; both directions; wrap across 16-bit/32-bit limits | count = expected, sign correct, no glitch at wrap |
| 4 | Velocity calculation | unit tests with synthetic Δc; bench test at known frequency | `ω` within quantisation of the expected value |
| 5 | PID response | host-side PID against a simulated first-order motor; then on the bench with the real motor, step responses | no windup after saturation; zero-speed behaviour as in G; plot overshoot/settling |
| 6 | PWM / motor output | scope/logic analyser on PWM and DIR with the motor supply **off**, then with the H-bridge on a current-limited supply | duty ↔ command, direction ↔ sign, outputs low at reset |
| 7 | CAN-FD feedback | capture feedback frames with a CAN-FD sniffer; compare with encoder stimulus | layout, scaling, period as in C2 |
| 8 | Communication timeout | stop the command stream | outputs go safe within the timeout; recovery behaves as agreed (I) |
| 9 | Error handling | malformed frames, out-of-range values, bus short / disconnect (bus-off), encoder unplugged, fault injection via a debug flag | each row in section I is observed |

Debug interface: USB-C is kept separate from the CAN-FD runtime protocol. If it exposes a serial port (UNKNOWN), it is used only for logging and a debug flag; no application command goes through it. Counters (RX frames, rejects, bus-off count, loop overruns) are visible in the status frame and in the debugger.

Tooling needed and not confirmed: a CAN-FD-capable adapter or a working CANdle path (**UNKNOWN - REQUIRES CONFIRMATION**).

---

## M. Unresolved values (UNKNOWN - REQUIRES CONFIRMATION)

Transport
1. Nominal bitrate, data bitrate, BRS, sample points, TDC
2. 11-bit vs 29-bit IDs on the CANdle bus
3. Whether CANdle forwards arbitrary (non-MD, non-PDS) frames
4. Final IDs for command, feedback, status; MD80 default IDs
5. Command variant R vs L (D1); mode-selector handling
6. Feedback rate for `joint_states`; handling of the unused steering field
7. Status/fault frame layout
8. Out-of-range handling and limits

Hardware
9. FDCAN instance, all pin assignments (encoders, PWM, DIR/EN, CAN, USB)
10. H-bridge IC, interface (PWM+DIR vs IN1/IN2), fault pins, PWM frequency
11. Encoder interface (quadrature direct vs external IC and its protocol)
12. Clock source / crystal
13. USB-C function (SWD vs native USB)
14. Pull-downs on motor lines at reset; any MCU-visible safety/E-STOP input; CAN termination on the PCB

Mechanical and control
15. Encoder CPR, counting mode, gearbox ratio, wheel diameter of the small rover
16. Wheel numbering/position mapping and sign conventions for the new rover (D2)
17. Whether the 21 vs 28 count asymmetry in legacy is intentional
18. PI gains, integral limit, output limit for the DC motors, control frequency
19. Whether a ramp is wanted
20. Kinematics constants (width/length) if Variant L

Safety
21. Command-timeout duration; recovery policy (auto vs gated)
22. Encoder plausibility thresholds, stall thresholds

Test
23. CAN-FD test adapter availability

---

## N. Assumptions (all stated, none silent)

1. The PDF block diagram (page 4) is the hardware reference; where it says "if needed" or "probably", I treat the item as undecided.
2. Legacy wheel numbering 1..4 (RF, RR, LF, LR) is the starting convention until you say otherwise.
3. Legacy scaling (int16 ÷ 1000 for command/velocity, × 10000 for angles/positions, little-endian) is preserved.
4. Motor power and CAN come over the shared architecture in the PDF; the STM never talks to the PDS.
5. No RTOS is needed; one control-loop timer plus a main loop suffices.
6. Auto-resume after timeout (legacy behaviour) is the default policy until you decide otherwise.
7. The MAB PDS documentation values (1M/2M/5M/8M, default ID 0x64) are real but not sufficient to fix the bus parameters.
8. A `Mały Łazik` motor has a quadrature or similar incremental encoder; no absolute wheel encoder is assumed.

---

## Sources

- Architecture PDF "Mały Łazik - architektura" (supplied)
- Legacy projects under `legacy files/` (DRIVE_CAN, GIZ_CAN_DZIALA_WSZYSTKO_LED_CZERWONY, MANI_CAN, POWER_CAN): `main.c`, `ALL_STRUCT.[ch]`, `BLDC_MOTOR.c`, `ABS_ENCODER.c`, `can.c`, `tim.c`, `.ioc`
- [MAB PDS overview](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/PDS/overview.html)
- [MAB PDS FDCAN communication](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/PDS/communication.html)
- [MAB CANdle intro](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle/intro.html)
