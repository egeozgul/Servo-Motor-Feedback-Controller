# Servo Motor Feedback Controller — User Guide

A complete reference for controlling the Smart Encoder Motor Driver over UART or I2C using the ASCII command interface.

---

## Drive Modes

The driver operates in one of four modes at any time:

| Mode | Description |
|---|---|
| **FLOAT** | H-bridge disabled — motor spins freely |
| **BRAKE** | Both H-bridge channels active — motor held against external force |
| **PID** | Closed-loop position control using `SETPOS` |
| **TRAPEZOID** | Smooth closed-loop motion using `MOVETO` with trajectory planning |

---

## Command Reference

All commands are plain ASCII strings terminated with `\n`, sent over UART (115200 baud, 8N1) or I2C (100 kHz, default address `0x20`). Position values are in encoder counts; velocity in counts/s; acceleration in counts/s².

### Motion Control

| Command | Description | Example |
|---|---|---|
| `SETPOS <n>` | Move to position `n` using PID control (immediate step) | `SETPOS 1000` |
| `MOVETO <n>` | Move to position `n` using trapezoidal trajectory planner | `MOVETO 5000` |
| `SETSPD <0-100>` | Set motor speed as PWM percentage (open-loop) | `SETSPD 75` |
| `BRAKE` | Engage active braking | `BRAKE` |
| `FLOAT` | Disable drive — motor coasts freely | `FLOAT` |

### Position & Velocity Queries

| Command | Description | Example |
|---|---|---|
| `GETPOS` | Query current encoder position (counts) | `GETPOS` |
| `GETSPD` | Query current velocity (counts/s) | `GETSPD` |
| `STATUS` | Query current drive mode | `STATUS` |

### PID Tuning

PID gain values are divided by 10 internally before being applied to the control output. For example, `SET_P 1000` results in an effective proportional gain of 100.

| Command | Description | Example |
|---|---|---|
| `SET_P <v>` | Set proportional gain $K_p$ | `SET_P 1000` |
| `SET_I <v>` | Set integral gain $K_i$ | `SET_I 0` |
| `SET_D <v>` | Set derivative gain $K_d$ | `SET_D 0` |
| `GET_PID` | Query current gains — returns `Kp <p>,Ki <i>,Kd <d>` | `GET_PID` |

### Trajectory Planner Settings

These settings apply when using `MOVETO`. They persist until changed or the board is power-cycled.

| Command | Default | Description | Example |
|---|---|---|---|
| `SETMAX_VEL <v>` | 20.0 | Maximum velocity (counts/s) | `SETMAX_VEL 20` |
| `SETMAX_ACC <v>` | 0.05 | Maximum acceleration (counts/s²) | `SETMAX_ACC 0.05` |

### Telemetry & Diagnostics

| Command | Description | Example |
|---|---|---|
| `LOG <ms>` | Stream telemetry every `ms` milliseconds (min. 20 ms). Format: `pos <n>,amps <i>,tps <v>` | `LOG 250` |
| `LOG 0` | Stop telemetry streaming | `LOG 0` |
| `GET_ADC` | Read motor current in mA via IPROPI sense resistor | `GET_ADC` |
| `CONFIRM <0/1>` | Enable (1) or disable (0) verbose command acknowledgements | `CONFIRM 1` |

---

## Telemetry Format

When logging is active, each line output has the format:

```
pos <n>,amps <i>,tps <v>
```

| Field | Description |
|---|---|
| `pos` | Encoder position in counts |
| `amps` | Motor current in mA |
| `tps` | Velocity in encoder counts per second |

---

## Usage Examples

### 1 — Basic position control (PID)
```
SET_P 1000       # Effective Kp = 100
SET_I 0
SET_D 0
LOG 250          # Stream telemetry every 250 ms
SETPOS 1000      # Step to position 1000
```

### 2 — Smooth motion with trajectory planner
```
SETMAX_VEL 20    # Max velocity 20 counts/s
SETMAX_ACC 0.05  # Max acceleration 0.05 counts/s²
MOVETO 3000      # Move smoothly to position 3000
```

### 3 — Round trip
```
MOVETO 3000      # Move to 3000
MOVETO 0         # Return to home
```

### 4 — PID tuning workflow
```
CONFIRM 1        # Enable acknowledgements
SET_P 500        # Start with low Kp
GET_PID          # Verify: Kp 500.0000000,Ki 0.0000000,Kd 0.0000000
LOG 250          # Start telemetry
SETPOS 2000      # Issue step command and observe response
LOG 0            # Stop telemetry when done
```

### 5 — Emergency stop
```
BRAKE            # Immediate active braking
```

---

## Connector Pinout

**J4 — Motor and encoder (6-pin, 2.54 mm pitch)**

| Pin | Signal | Description |
|---|---|---|
| 1 | M1 | Motor terminal 1 |
| 2 | M2 | Motor terminal 2 |
| 3 | GND | Ground |
| 4 | 3.3 V | Encoder power supply |
| 5 | S2 | Encoder channel B |
| 6 | S1 | Encoder channel A |

**J3 — UART and secondary power (5-pin, 2.54 mm pitch)**

| Pin | Signal |
|---|---|
| 1 | 3.3 V |
| 2 | GND |
| 3 | RX |
| 4 | TX |
| 5 | Motor power supply |

**J2 / J5 — I2C Qwiic connectors (JST SH 4-pin, 1.0 mm pitch)**

Standard Qwiic pinout: GND, 3.3 V, SDA, SCL. Connect J5 output to J2 input of the next board to daisy-chain.

---

## Daisy-Chaining Multiple Units

Up to 112 drivers can share a single I2C bus using standard 7-bit addressing.

1. Assign each board a unique I2C address by modifying `hi2c1.Init.OwnAddress1` in `main.c` and reflashing before deployment (default address: `0x20`)
2. Connect J5 (output) of one board to J2 (input) of the next
3. Address each board individually in host I2C transactions

---

## Safety Notes

- Always verify power supply polarity before connecting
- Do not exceed 45 V supply voltage under any transient condition
- Do not connect or disconnect the motor while the board is energized
- Position is relative — absolute position is not retained across power cycles. Implement a homing routine if absolute positioning is required

---

## Default Settings

| Parameter | Default value |
|---|---|
| I2C address | `0x20` (7-bit) |
| UART baud rate | 115200 (8N1) |
| Max velocity | 20.0 counts/s |
| Max acceleration | 0.05 counts/s² |
| Kp / Ki / Kd | 0.0001 / 0.0 / 0.0 |
| Firmware scaling factor | ÷10 applied to all PID gains |
