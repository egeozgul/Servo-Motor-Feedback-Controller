# Servo Motor Feedback Controller

<p align="center">
  <img src="media/pcb__.png" alt="Smart Encoder Motor Driver" width="80%"/>
</p>

An open-source, closed-loop DC motor driver with embedded PID control and trapezoidal trajectory planning. Built around the STM32C0 microcontroller, the board integrates a full H-bridge, quadrature encoder interface, and a real-time PID feedback loop on a compact 30 mm × 40 mm PCB. It accepts simple high-level commands over I2C or UART and autonomously handles all low-level motor control.

---

## PCB Assembly Video

[![Watch the assembly video](https://img.youtube.com/vi/LhhGqf6qH90/maxresdefault.jpg)](https://youtu.be/LhhGqf6qH90)

---

## Key Features

| Feature | Details |
|---|---|
| Supply voltage | 5 V to 45 V |
| Continuous current | Up to 4.1 A |
| Microcontroller | STM32C011F6U6TR (ARM Cortex-M0+, 4 MHz) |
| Encoder interface | Incremental quadrature, 64-bit counter |
| PWM resolution | 16-bit |
| Communication | UART (115200 baud) and I2C (100 kHz, default address 0x20) |
| Daisy-chain | Up to 112 units on one I2C bus |
| PCB size | 30 mm × 40 mm |
| License | MIT |

---

## How It Works

The host sends a single target position command (`MOVETO 3000`). The onboard trapezoidal trajectory planner computes a smooth motion profile — acceleration, cruise, deceleration — and the PID controller tracks it in real time. No streaming of setpoints from the host is required.

---

## Trapezoidal Trajectory — Measured Results

Five consecutive cycles (0 → 3000 → 0 encoder counts), no load, 12 V supply:

<img src="https://github.com/user-attachments/assets/1d923db1-4959-4971-a14b-bc9a89ad5b6b" alt="5-cycle trajectory" width="100%"/>

Step response (0 → 4000 encoder counts), proportional-only control (Kp = 1000), 12 V supply:

<img src="https://github.com/user-attachments/assets/67a65020-c57f-4084-943b-88aa9725c101" alt="Step response" width="100%"/>

---

## ASCII Command Interface

Commands are sent as plain text terminated with `\n` over UART or I2C.

| Command | Description | Example |
|---|---|---|
| `MOVETO <n>` | Move to position using trajectory planner | `MOVETO 3000` |
| `SETPOS <n>` | Move to position using PID only (no trajectory) | `SETPOS 1000` |
| `GETPOS` | Query current encoder position | `GETPOS` |
| `GETSPD` | Query current velocity | `GETSPD` |
| `SET_P <v>` | Set proportional gain | `SET_P 1000` |
| `SET_I <v>` | Set integral gain | `SET_I 0` |
| `SET_D <v>` | Set derivative gain | `SET_D 0` |
| `GET_PID` | Query current PID gains | `GET_PID` |
| `SETMAX_VEL <v>` | Set max trajectory velocity (counts/s) | `SETMAX_VEL 20` |
| `SETMAX_ACC <v>` | Set max trajectory acceleration (counts/s²) | `SETMAX_ACC 0.05` |
| `LOG <ms>` | Stream telemetry every `ms` milliseconds | `LOG 250` |
| `LOG 0` | Stop telemetry | `LOG 0` |
| `GET_ADC` | Read motor current (mA) | `GET_ADC` |
| `BRAKE` | Active brake | `BRAKE` |
| `FLOAT` | Coast (H-bridge floating) | `FLOAT` |
| `STATUS` | Query drive mode | `STATUS` |

Telemetry output format: `pos <n>,amps <i>,tps <v>`

---

## Quickstart

1. Order the PCB using the Gerber files in `gerber_files/`
2. Assemble following the [assembly video](https://youtu.be/LhhGqf6qH90)
3. Flash `firmware_Image.elf` using STM32CubeProgrammer via the TC2030 SWD footprint
4. Connect your DC motor and encoder to the J4 header
5. Power the board (5–45 V) via the J1 screw terminal
6. Connect via UART (115200 baud) or I2C (0x20) and send commands

---

## Flashing the Firmware

Connect an ST-Link V2 to the TC2030 SWD footprint (SWDIO, SWCLK, GND, 3.3 V) and use STM32CubeProgrammer to flash `firmware_Image.elf`, or build from source using STM32CubeIDE with the project in `SourceCode/`.

---

## Repository Contents

| Path | Description |
|---|---|
| `SourceCode/Src/main.c` | Complete STM32 firmware (C) |
| `gerber_files/` | PCB manufacturing files (Gerber + drill) |
| `PCB_Design/schematic.png` | Circuit schematic |
| `PCB_Design/schematic.ipc2581c` | PCB design (IPC-2581C, vendor-neutral) |
| `pcb_.png` | PCB render |
| `firmware_Image.elf` | Pre-compiled firmware image |
| `assembly.mp4` | PCB assembly walkthrough (also on YouTube) |

---

## PCB

<p align="center">
  <img src="https://github.com/user-attachments/assets/449203f8-8fd5-4fc8-97ef-0804dd21e011" alt="PCB Top View" width="80%"/>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/fbc92872-6157-4653-9876-7cdbfeab308c" alt="PCB Assembled" width="80%"/>
</p>

---

## License

MIT License — free to use, modify, and distribute for any purpose.
