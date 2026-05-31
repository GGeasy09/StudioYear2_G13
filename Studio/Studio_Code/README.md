# Base System Firmware — 1-DOF Circular Robot (STM32G474)

> 🌐 ภาษาไทย: [README.th.md](README.th.md)

Embedded firmware for a **single-degree-of-freedom rotary (circular) robot**. The MCU runs a
cascade position/velocity controller with feedforward and a Kalman state estimator, drives the
motor through a real-time control loop, and exposes the robot to a PC application over **Modbus RTU**.
It can be operated two ways:

- **Local control** via the electrical-cabinet **joystick / buttons / mode switch** (`Trust_Joystick`).
- **Remote control** via the **Base System** PC app — web UI → WebSocket → Modbus over USB serial
  (`Trust_Basesystem`). See `../Base_system/README.md` for the host side and the full register map.

Supported operations: **Homing**, **Jog**, **Point-to-point**, **Auto pick-&-place sequence**,
and **Performance / Precision tests**, with **emergency** and **soft-stop** safety handling.

---

## 1. Hardware target

| Item | Detail |
|------|--------|
| MCU | STM32G474 (Cortex-M4F, single-precision FPU, CMSIS-DSP) |
| Motor drive | **TIM1 CH1** PWM (`PWM()` in `system_state.c`) |
| Microsecond time base | **TIM2** (free-running, trajectory timing) |
| Encoder | **TIM3** in encoder mode (quadrature position) |
| Control loop ISR | **TIM20** period-elapsed callback @ **2 kHz** (`Robot_Period_Control_Loop`) |
| Modbus timing | **TIM16** (T1.5 / T3.5 character gaps) |
| Serial link | **USART2**, **230400 baud, 8 data, Even parity, 1 stop**, slave addr **21 (0x15)** |
| CAN | **FDCAN** (configured) |
| Homing | Proximity reference sensor state machine |
| Discrete I/O | Joystick buttons, mode switch, pilot lamps, power/auto relays, gripper relays & reed sensors |

Mechanical convention: **one hole = 5°** (`DEG_PER_HOLE`). Positive = CCW, negative = CW.

---

## 2. Build & flash

1. Open **STM32CubeIDE** and import this project (contains `.cproject` / `.project`).
2. Hardware configuration is in **`Base_System.ioc`** — open with CubeMX to change pins/peripherals, then regenerate.
3. Select the **Debug** configuration and build (`make -j all`).
4. Flash / debug over **ST-Link** (`Base_System Debug.launch`).
5. For PC control, run the Base System host app and select the ST-Link Virtual COM port.

---

## 3. Project layout

Only application modules are listed; HAL/CMSIS and CubeMX-generated peripheral files
(`dma.c`, `fdcan.c`, `gpio.c`, `tim.c`, `usart.c`, `stm32g4xx_*`, `sys*.c`) are standard.

| File | Role |
|------|------|
| `Core/Src/main.c` | Entry, init, super-loop, the main **state machine** (`Robot_State_Process`) and the **2 kHz control loop** (`Robot_Period_Control_Loop`). Owns global objects (`sys_state`, `Controller`, `Kalman`, `traj_mgr`, joystick/gripper/sequence/test state). |
| `Core/Src/system_state.c` · `Inc/system_state.h` | Encoder read/compute, **`PWM()`** motor output, **`SYSTEM_STATE_Homing()`**, position bookkeeping, deg↔rad helpers. Defines `SYSTEM_STATE`, `Encoder`, `Proximity`. |
| `Core/Src/elec_cabient.c` · `Inc/elec_cabient.h` | Electrical-cabinet I/O: **`SYSTEM_STATE_Joystick_Update()`** (mode switch + buttons + soft-stop), pilot lamp/ramp relays (`PilotRamp_SetPower`, `PilotRamp_SetAuto`, `PilotLamp_SetWhite`), gripper control (relays + reed limit sensors). |
| `Core/Src/trajectory.c` · `Inc/trajectory.h` | Motion planners — **Trapezoidal**, **S-curve (7-segment)**, **Min-jerk**, **Min-jerk velocity-limited** — plus the high-level **`TrajManager`** API (`_Init`, `_Plan`, `_Step`, `_DynTime`). |
| `Core/Src/cascade.c` · `Inc/cascade.h` | **Cascade controller**: outer position PID → inner velocity PID, with motor **feedforward** and disturbance terms. `CASCADE_Controller_Init`, `CASCADE_Cascade_Start`, `CASCADE_Compute`. |
| `Core/Src/kalman.c` · `Inc/kalman.h` | **4-state Kalman estimator** (position, velocity, disturbance, accel) using CMSIS-DSP matrices. `KALMAN_Multi_Model_Init`, `_Compute`, `KALMAN_Calc_Acceleration`. |
| `Core/Src/BaseSystem.c` · `Inc/BaseSystem.h` | **Modbus RTU slave**: frame RX/parse/CRC, 50-register holding map, command decode (`BaseCmd`), heartbeat. Command enums (`CMD_MODE_*`, `CMD_TEST_*`, `CMD_GRIPPER_*`). |
| `Core/Inc/motor_params.h` | Motor model constants: torque/back-EMF constants, inertia, damping, resistance, inductance, FF filter time constant. |

---

## 4. Control architecture

Runs every 0.5 ms inside `Robot_Period_Control_Loop` (TIM20 @ 2 kHz):

```
 Encoder (TIM3) ──► Kalman estimator ──► state: pos / velo / disturbance
                                              │
 TrajManager_Step ──► reference (pos, velo) ──┤
                                              ▼
                          Cascade: outer PID (position) ──► inner PID (velocity)
                                              + feedforward (motor model)
                                              ▼
                                   voltage ──► PWM() ──► TIM1 motor drive
```

When no trajectory is active (`traj_mgr.running == 0`) the controller holds the last commanded
position (`sys_state.cur_pos`). A small static-friction offset is added at the PWM stage.

Default gains (top of `main.c`):

| Loop | Kp | Ki | Kd |
|------|----|----|----|
| Inner (velocity) | 0.52259 | 10.2259 | 0.0 |
| Outer (position) | 2.0 | 0.0 | 1.0 |

---

## 5. Operating modes & state machine

`Robot_State_Process()` runs in the super-loop (non-real-time). Control authority is selected by
`sys_state.trust`: **`Trust_Joystick`** (local cabinet) or **`Trust_Basesystem`** (PC/Modbus).

Base System modes follow the Modbus `0x01` one-hot values:

| Mode | Value | Behavior |
|------|------:|----------|
| Idle | 0 | No command |
| **Home** | 1 | Run proximity homing, then move to home reference |
| **Jog** | 2 | One signed step (degrees) from current position per request |
| **Auto** | 4 | Point-to-point or pick-&-place **sequence** (`seq`: go-pick → grip → go-place → release, per pair) |
| **Set home** | 8 | Snapshot current position as Base System origin (joystick-only) |
| **Test** | 16 | **Performance** (S-curve out/back) or **Precision** (repeated min-jerk init↔final round-trips) |

Safety overrides (checked first, every cycle):

- **Emergency** (PA4): stops the ISR, cuts PWM, clears all activity, latches until released; on release re-seeds position from the encoder and restarts homing.
- **Soft stop** (joystick or `0x25`): completes/cancels motion and returns to *waiting command*.

---

## 6. Joystick / local control (`Trust_Joystick`)

The electrical cabinet provides a **mode switch**, six colored buttons, two white step buttons, and
a soft-stop line. Inputs are read by `SYSTEM_STATE_Joystick_Update()`; all actions are **edge-triggered**
(fire once on a press, i.e. raw goes `1 → 0`).

**Physical layout** (front of the cabinet panel, as wired):

```
   ( White-1 )                       [ ⌷ ] mode switch
   ( White-2 )
                                     ╔═══════╗
                                     ║  DB9  ║  serial (Modbus/USART2)
   ( Black )                         ╚═══════╝

 ( Blue )    ( Yellow )              [□□] terminal block
   ( Red )
       │
   soft-stop / e-stop cable
```

The two white buttons are **stacked vertically** on the panel but are named **White-left (PC1)**
and **White-right (PB1)** in firmware. Note the hardware link: pressing the "right" white button
can also trigger the "left" line (see comment in `elec_cabient.h`).

**Button hardware map** (`elec_cabient.h`):

| Button | Pin | Button | Pin |
|--------|-----|--------|-----|
| Mode switch | PA15 | White-left (upper) | PC1 |
| Black | PA9 | White-right (lower) | PB1 |
| Red | PA8 | Blue | PB2 |
| Yellow | PB9 | Soft-stop | PB7 |

### 6.1 White buttons (jog by holes) — both modes
- **White-left** → **+** direction (CCW); **White-right** → **−** direction (CW).
- Step size depends on the mode switch: **mode 0 = 5 holes** per press (`5 × DEG_PER_HOLE`),
  **other modes = 1 hole** per press. Each press dispatches a min-jerk move.

### 6.2 Mode 0 — direct gripper control
| Button | Action |
|--------|--------|
| Black | Gripper **Open** |
| Blue | Gripper **Close** |
| Yellow | Gripper **Up** |
| Red | Gripper **Down** |

### 6.3 Mode 1 — home / pick / place
| Button | Action |
|--------|--------|
| Black | **Set home** = current position |
| Blue | **Go to home** position |
| Red | Start gripper **Pick** sequence |
| Yellow | Start gripper **Place** sequence |

The pick/place gripper state machine then runs each tick until the action completes (or times out).

---

## 7. Key tunable parameters

| Parameter | Location | Default | Meaning |
|-----------|----------|---------|---------|
| Control period | `TrajManager_Step` | `0.0005f` | 2 kHz loop period (must match TIM20) |
| `DEG_PER_HOLE` | `main.c` | `5.0f` | Degrees per index/hole |
| `GRIPPER_TIMEOUT_MS` | `main.c` | `5000` | Max wait per pick/place |
| Dynamic move speed | `trajectory.c` | `110 °/s` | Nominal cruise used by `TrajManager_DynTime` |
| Move time clamp | `trajectory.c` | `0.9 … 3.5 s` | Min/max auto min-jerk duration |
| Position-reached tol. | `main.c` (seq phases) | `0.001745f` rad (≈0.1°) | Tolerance to advance a sequence step |
| PID gains | `main.c` | see §4 | Cascade inner/outer tuning |

> **Note:** the 0.1° sequence tolerance is tight. If the controller settles with larger steady-state
> error, sequence steps can stall; loosen it (e.g. `0.00873f` ≈ 0.5°) if needed.

---

## 8. Modbus interface (firmware side)

The robot is a Modbus RTU **slave** (addr 21). The PC writes commands to holding registers and
block-reads status/feedback:

- **Writes (PC→robot):** `0x01` mode, `0x05` jog degrees, `0x06–0x11` test params, `0x12–0x22`
  pick-place sequence, `0x23–0x24` point-to-point, `0x25` soft stop, gripper `0x02–0x04`.
- **Reads (robot→PC):** `0x00` heartbeat, `0x26` reed sensors, `0x27` current task,
  `0x28/0x29/0x30` position/velocity/acceleration (**×10** scaled), `0x31` emergency.

Full register map, bit meanings, scaling and two's-complement rules: **`../Base_system/README.md`**.
Firmware decode lives in `BaseSystem.c` (`BaseCmd` struct, `CMD_*` enums in `BaseSystem.h`).

---

## 9. Notes / recent changes

- `TrajManager` and its `Init/Plan/Step/DynTime` API are defined in `trajectory.{c,h}` and used throughout `main.c`.
- All planner `*_Compute` functions explicitly set `out.Complete` in every path (a missing initializer previously caused trajectories to finish after a single step).
- White-button direction and the mode-0 5-hole step are as described in §6.
