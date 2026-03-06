/**
 ******************************************************************************
 * @file    main.c
 * @brief   DC motor controller for STM32C0xx.
 *
 *          Features:
 *            - Trapezoidal / triangular motion profiling
 *            - PID position control
 *            - Quadrature encoder tracking (TIM3 + overflow ISR)
 *            - H-bridge PWM drive (TIM1 CH3/CH4)
 *            - UART + I2C command interface (both DMA-driven, circular)
 *            - ADC current sensing with simple moving average filter
 *
 *          Command interface (ASCII over UART or I2C, newline-terminated):
 *            SETPOS <n>      — move to absolute encoder position (PID)
 *            MOVETO <n>      — move to position using trapezoidal profile
 *            FLOAT           — coast (zero PWM both channels)
 *            BRAKE           — active brake (50% PWM both channels)
 *            GETPOS          — read current encoder position
 *            GETSPD          — read encoder ticks-per-ms
 *            GET_PID         — print Kp, Ki, Kd
 *            SET_P/I/D <f>   — set PID gains
 *            GET_ADC         — read motor current in mA
 *            LOG <ms>        — enable periodic logging at <ms> interval
 *            CONFIRM <0|1>   — enable/disable verbose confirmations
 *            SETMAX_VEL <f>  — set trajectory max velocity
 *            SETMAX_ACC <f>  — set trajectory max acceleration
 *            RESET           — system reset (disabled by default)
 ******************************************************************************
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 * Licensed under terms found in the LICENSE file in the root directory.
 ******************************************************************************
 */

#include "main.h"
#include "helper.h"

/* ---------------------------------------------------------------------------
 * HAL peripheral handles
 * ---------------------------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_i2c1_rx;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;          /* PWM output for H-bridge */
TIM_HandleTypeDef  htim3;          /* Quadrature encoder input */
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_rx;
DMA_HandleTypeDef  hdma_usart2_tx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void handleCommand(uint8_t *buf, int bufSize, uint16_t cmdIdx);
static int  computeEncoderCounterPeriod(void);
static int64_t get_full_encoder_count(void);

/* ===========================================================================
 * Constants
 * ===========================================================================*/

/* Peripheral selector values */
#define UART_PERIPHERAL     0
#define I2C_PERIPHERAL      1

/* Motor drive modes */
#define MOTOR_FLOAT         0   /* Coast — both PWM channels at 0 */
#define MOTOR_BRAKE         1   /* Brake — both PWM channels at 50% */
#define MOTOR_PID           2   /* Hold position with PID */
#define MOTOR_TRAPEZOID     3   /* Move to position with motion profile + PID */

/* Buffer sizes */
#define I2C_BUF_SIZE        64
#define DMA_BUF_SIZE        2048
#define ADC_BUF_SIZE        20   /* Depth of ADC moving-average window */
#define MAX_CMD_LEN         25   /* Max characters in a single command */

/* PID */
#define INTEGRAL_MAX        100  /* Anti-windup clamp on the integral term */

/* ADC current sensing: I(mA) = (ADC * Vref / ADC_MAX) / R_sense */
#define ADC_VREF            3.3f
#define ADC_MAX_COUNT       4095.0f
#define IPROPI_RESISTOR     1.5f    /* kΩ sense resistor on IPROPI pin */

/* Encoder — TIM3 is 16-bit, so it overflows every 65536 counts.
 * The overflow ISR adjusts a 64-bit accumulator by this amount. */
#define ENCODER_OVERFLOW    0x10000

/* I2C transmit-ready flag timeout */
#define I2C_TXIS_TIMEOUT_MS 500

/* ===========================================================================
 * DMA receive buffers
 * ===========================================================================*/

/* UART RX circular DMA buffer — commands arrive here byte-by-byte */
static uint8_t rxBuffer[DMA_BUF_SIZE] __attribute__((section(".bss")));

/* I2C slave RX DMA buffer — commands from an I2C master */
static uint8_t i2cRxBuffer[I2C_BUF_SIZE];

/* I2C slave TX cyclic buffer — data queued here is sent on master read */
static uint8_t i2cTxBuffer[I2C_BUF_SIZE];

/* ===========================================================================
 * I2C transmit cyclic buffer
 *
 * A simple power-of-two cyclic (ring) buffer used to queue outgoing bytes
 * for the I2C slave transmit path.  The buffer is written by the main-loop
 * print helpers and drained inside the I2C IRQ handler.
 *
 * Head  — next write position.
 * Tail  — next read  position.
 * Full flag distinguishes the full state (head == tail) from empty.
 * ===========================================================================*/
static int txHead   = 0;
static int txTail   = 0;
static int txIsFull = 0;

/** @brief Returns 1 if the I2C TX cyclic buffer is full, 0 otherwise. */
static int buffer_is_full(void)  { return txIsFull; }

/** @brief Returns 1 if the I2C TX cyclic buffer is empty, 0 otherwise. */
static int buffer_is_empty(void) { return (txHead == txTail && !txIsFull); }

/**
 * @brief  Insert one character into the I2C TX cyclic buffer.
 * @note   If the buffer is full the oldest byte is silently overwritten
 *         (tail advances to make room).
 */
static void buffer_insert(char c)
{
    txIsFull = buffer_is_full();
    i2cTxBuffer[txHead] = c;
    txHead = (txHead + 1) % I2C_BUF_SIZE;
    if (txIsFull)
        txTail = (txTail + 1) % I2C_BUF_SIZE;  /* Overwrite oldest byte */
    if (txHead == txTail)
        txIsFull = 1;
}

/**
 * @brief  Remove one character from the I2C TX cyclic buffer.
 * @param  c  Output pointer — receives the removed character.
 * @retval 1  on success, 0 if the buffer was empty.
 */
static int buffer_remove(char *c)
{
    if (buffer_is_empty())
        return 0;
    *c = (char)i2cTxBuffer[txTail];
    txTail = (txTail + 1) % I2C_BUF_SIZE;
    txIsFull = 0;
    return 1;
}

/* ===========================================================================
 * ADC simple moving average (SMA) buffer
 *
 * Stores the last ADC_BUF_SIZE raw ADC readings so the current measurement
 * can be smoothed before conversion to milliamps.
 * ===========================================================================*/
static uint32_t adcBuf[ADC_BUF_SIZE];
static int      adcHead  = 0;
static int      adcTail  = 0;
static int      adcCount = 0;

/**
 * @brief  Push a new ADC sample into the circular SMA buffer.
 *         Oldest sample is discarded automatically when the buffer is full.
 */
static void adcbuf_insert(uint32_t val)
{
    adcBuf[adcHead] = val;
    adcHead = (adcHead + 1) % ADC_BUF_SIZE;
    if (adcCount < ADC_BUF_SIZE)
        adcCount++;
    else
        adcTail = (adcTail + 1) % ADC_BUF_SIZE;
}

/**
 * @brief  Compute the simple moving average of all samples in the ADC buffer.
 * @retval Average raw ADC count as a float, or 0 if the buffer is empty.
 */
static float adcbuf_sma(void)
{
    uint64_t sum = 0;
    for (int i = 0; i < adcCount; i++)
        sum += adcBuf[(adcTail + i) % ADC_BUF_SIZE];
    return adcCount > 0 ? (float)sum / adcCount : 0.0f;
}

/* ===========================================================================
 * Active output peripheral
 *
 * Selects whether print helpers route output to UART or the I2C TX buffer.
 * ===========================================================================*/
static int activePeripheral = UART_PERIPHERAL;

/* ===========================================================================
 * Print utility functions
 *
 * Lightweight replacements for printf / sprintf that work without the C
 * standard library.  All output is routed through activePeripheral so the
 * same call sites work for both UART and I2C.
 * ===========================================================================*/

/** @brief Transmit a single character via the active peripheral. */
static void printChar(char ch)
{
    if (activePeripheral == UART_PERIPHERAL)
        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    else
        buffer_insert(ch);
}

/** @brief Transmit a null-terminated string via the active peripheral. */
static void printString(const char *s)
{
    while (*s)
        printChar(*s++);
}

/** @brief Transmit a raw byte buffer of known length via the active peripheral. */
static void printBuffer(const uint8_t *data, uint16_t len)
{
    if (activePeripheral == UART_PERIPHERAL)
        HAL_UART_Transmit(&huart2, data, len, HAL_MAX_DELAY);
    else
        for (uint16_t i = 0; i < len; i++)
            buffer_insert((char)data[i]);
}

/**
 * @brief  Transmit an unsigned 64-bit integer as a decimal ASCII string.
 *         Builds the string right-to-left in a local buffer to avoid division
 *         by large powers of 10.
 */
static void printUInt64(uint64_t val)
{
    char buf[21];   /* Max 20 digits + null */
    int  i = 20;
    buf[i] = '\0';
    do {
        buf[--i] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    printString(&buf[i]);
}

/**
 * @brief  Transmit a signed 64-bit integer as a decimal ASCII string.
 *         Handles INT64_MIN explicitly because negating it overflows.
 */
static void printInt64(int64_t val)
{
    char buf[21];
    int  i        = 20;
    int  negative = (val < 0);

    buf[i] = '\0';

    if (val == INT64_MIN) {
        /* -INT64_MIN overflows; adjust manually */
        buf[--i] = '8';
        val = INT64_MAX;
    } else if (negative) {
        val = -val;
    }

    do {
        buf[--i] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);

    if (negative)
        buf[--i] = '-';

    printString(&buf[i]);
}

/**
 * @brief  Transmit a float as "<integer>.<decimals>" ASCII text.
 * @param  val       Value to print.
 * @param  decimals  Number of digits after the decimal point.
 */
static void printFloat(float val, int decimals)
{
    if (val < 0.0f) {
        printChar('-');
        val = -val;
    }
    printInt64((int64_t)val);
    printChar('.');
    float frac = val - (float)(int64_t)val;
    for (int i = 0; i < decimals; i++) {
        frac *= 10.0f;
        int d = (int)frac;
        printChar('0' + d);
        frac -= d;
    }
}

/* ===========================================================================
 * Minimal string helpers (no stdlib dependency)
 * ===========================================================================*/

/**
 * @brief  strcmp replacement that does not depend on <string.h>.
 * @retval 0 if strings are equal, non-zero otherwise (same semantics as strcmp).
 */
static int strcmp_embedded(const char *a, const char *b)
{
    while (*a && *b && *a == *b) { a++; b++; }
    return (unsigned char)(*a) - (unsigned char)(*b);
}

/**
 * @brief  atoi replacement — converts a decimal ASCII string to int.
 *         Skips leading whitespace and handles an optional leading sign.
 */
static int atoi_embedded(const char *s)
{
    int result = 0, sign = 1;
    while (*s == ' ' || *s == '\t' || *s == '\n' ||
           *s == '\r' || *s == '\f' || *s == '\v')
        s++;
    if      (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9')
        result = result * 10 + (*s++ - '0');
    return result * sign;
}

/* ===========================================================================
 * PWM / H-bridge helpers
 * ===========================================================================*/

/**
 * @brief  Update the duty cycle on a single TIM1 PWM channel.
 * @param  channel  TIM_CHANNEL_3 (forward) or TIM_CHANNEL_4 (reverse).
 * @param  value    Pulse count (0 = off, htim1.Init.Period = 100% duty).
 * @note   Reconfigures the channel at runtime so both channels can be set
 *         independently without stopping the timer.
 */
static void UpdatePWMDutyCycle(uint32_t channel, uint32_t value)
{
    TIM_OC_InitTypeDef oc = {
        .OCMode       = TIM_OCMODE_PWM1,
        .Pulse        = value,
        .OCPolarity   = TIM_OCPOLARITY_HIGH,
        .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
        .OCFastMode   = TIM_OCFAST_DISABLE,
        .OCIdleState  = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };
    HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &oc, channel);
    HAL_TIM_PWM_Start_optimized(&htim1, channel);
}

/* ===========================================================================
 * Encoder position tracking
 *
 * TIM3 runs in quadrature encoder mode with a 16-bit counter (0–65535).
 * When the counter overflows or underflows, TIM3_IRQHandler adjusts a 64-bit
 * accumulator so callers always see a monotonic signed position value.
 * ===========================================================================*/
static volatile int64_t fullEncoderCount = 0;

/**
 * @brief  TIM3 update ISR — extends the 16-bit hardware counter to 64 bits.
 *         Called on counter overflow (forward) or underflow (reverse).
 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE))
    {
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
            fullEncoderCount -= ENCODER_OVERFLOW;   /* Underflow (reverse) */
        else
            fullEncoderCount += ENCODER_OVERFLOW;   /* Overflow  (forward) */
    }
}

/**
 * @brief  Return the current absolute encoder position as a signed 64-bit count.
 *         Combines the 64-bit overflow accumulator with the live 16-bit
 *         hardware counter.  The counter is initialised to 32768 (mid-range)
 *         so that it can count both directions without immediately over/underflowing;
 *         subtracting ENCODER_OVERFLOW/2 (65536) re-centres the reading.
 */
static int64_t get_full_encoder_count(void)
{
    return fullEncoderCount + (int64_t)__HAL_TIM_GET_COUNTER(&htim3) - ENCODER_OVERFLOW;
}

/* Speed measurement state — updated each main-loop iteration */
static int64_t  encLastPos  = 0;
static int64_t  encCurPos   = 0;
static uint32_t encLastTick = 0;
static uint32_t encCurTick  = 0;

/**
 * @brief  Compute encoder ticks per millisecond (signed).
 *         Returns 0 if no counts have occurred to avoid division by zero.
 */
static int computeEncoderCounterPeriod(void)
{
    int32_t nCounts = (int32_t)(encCurPos  - encLastPos);
    int32_t nTicks  = (int32_t)(encCurTick - encLastTick);
    if (nCounts == 0) return 0;
    return nTicks / nCounts;
}

/* ===========================================================================
 * ADC current sensing
 * ===========================================================================*/

/**
 * @brief  Trigger a single ADC conversion and return the raw 12-bit result.
 *         Blocks until conversion completes (polling mode, 100 ms timeout).
 */
static uint32_t readADCValue(void)
{
    uint32_t val = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

/**
 * @brief  Convert a raw (averaged) ADC count to motor current in milliamps.
 *
 *         The motor driver outputs a current-proportional voltage on IPROPI:
 *           V_ipropi = ADC_count * (Vref / ADC_max)
 *           I_motor  = V_ipropi / R_sense
 *
 * @param  rawAvg  Averaged raw ADC count (e.g. from adcbuf_sma()).
 * @retval Motor current in mA.
 */
static float adcToMilliamps(float rawAvg)
{
    float Vipropi = rawAvg * (ADC_VREF / ADC_MAX_COUNT);
    return Vipropi / IPROPI_RESISTOR;
}

/* ===========================================================================
 * Trapezoidal / triangular motion profile
 *
 * Given a start and target encoder position, pre-computes the timing
 * parameters for a constant-acceleration profile:
 *
 *   Trapezoidal (long move):  accel → constant velocity → decel
 *   Triangular  (short move): accel → decel  (peak velocity never reached)
 *
 * updateTrajectory() is called every control cycle with the elapsed time dt
 * and writes the next setpoint position into trajSetpointPos, which the PID
 * controller then tracks.
 * ===========================================================================*/
static float   V_max           = 20.0f;   /* Max velocity (counts/s) */
static float   A_max           = 0.05f;   /* Max acceleration (counts/s²) */

static int64_t trajStartPos    = 0;
static int64_t trajTargetPos   = 0;
static int64_t trajSetpointPos = 0;
static int64_t trajTotalDist   = 0;
static int64_t trajDAccel      = 0;       /* Distance covered during acceleration phase */
static int64_t trajDirection   = 1;       /* +1 forward, -1 reverse */
static float   trajTAccel      = 0.0f;   /* Duration of accel/decel phase (s) */
static float   trajTConst      = 0.0f;   /* Duration of constant-velocity phase (s) */
static float   trajTTotal      = 0.0f;   /* Total trajectory duration (s) */
static float   trajCurrentTime = 0.0f;   /* Elapsed time since trajectory start (s) */
static int8_t  trajTrapezoid   = 0;       /* 1 = trapezoidal profile, 0 = triangular */

/**
 * @brief  Pre-compute motion profile parameters for a move from start to target.
 *
 *         Selects trapezoidal or triangular profile depending on whether the
 *         move is long enough to reach V_max.  Prints T_accel, T_const and
 *         D_accel to the active peripheral for debugging.
 *
 * @param  start   Encoder count at the beginning of the move.
 * @param  target  Desired final encoder count.
 */
static void initializeTrajectory(int64_t start, int64_t target)
{
    trajStartPos    = start;
    trajTargetPos   = target;
    trajSetpointPos = start;
    trajTotalDist   = target - start;
    if (trajTotalDist < 0) trajTotalDist = -trajTotalDist;

    /* Distance required to accelerate from 0 to V_max then decelerate back */
    trajDAccel = (int64_t)((V_max * V_max) / (2.0f * A_max));

    float adjusted_D_accel = -1.0f;

    if (trajTotalDist < 2 * trajDAccel) {
        /* Short move — triangular profile, peak velocity < V_max */
        trajTrapezoid    = 0;
        adjusted_D_accel = (float)(trajTotalDist / 2);
        trajTAccel       = custom_sqrtf(2.0f * adjusted_D_accel / A_max);
        trajDAccel       = (int64_t)adjusted_D_accel;
        trajTConst       = 0.0f;
    } else {
        /* Long move — full trapezoidal profile */
        trajTrapezoid = 1;
        trajTAccel    = V_max / A_max;
        trajTConst    = (float)(trajTotalDist - 2 * trajDAccel) / V_max;
    }

    trajTTotal      = 2.0f * trajTAccel + trajTConst;
    trajCurrentTime = 0.0f;
    trajDirection   = (target > start) ? 1 : -1;

    /* Debug output */
    printFloat(trajTAccel, 4); printChar(' ');
    printFloat(trajTConst, 4); printChar(' ');
    printFloat(adjusted_D_accel, 4); printChar(' ');
}

/**
 * @brief  Advance the motion profile by dt seconds and update trajSetpointPos.
 *
 *         Must be called every control cycle.  Once the trajectory is complete
 *         the setpoint is clamped to the target position and stays there.
 *
 * @param  dt  Time elapsed since the last call (seconds).
 */
static void updateTrajectory(float dt)
{
    trajCurrentTime += dt;

    if (trajCurrentTime <= trajTAccel) {
        /* Acceleration phase: s = 0.5 * a * t² */
        trajSetpointPos = trajStartPos
            + trajDirection * (int64_t)(0.5f * A_max * trajCurrentTime * trajCurrentTime);

    } else if (trajTrapezoid && trajCurrentTime <= trajTAccel + trajTConst) {
        /* Constant velocity phase: s = D_accel + v * (t - t_accel) */
        trajSetpointPos = trajStartPos
            + trajDirection * trajDAccel
            + trajDirection * (int64_t)(V_max * (trajCurrentTime - trajTAccel));

    } else if (trajCurrentTime <= trajTTotal) {
        /* Deceleration phase: mirror of acceleration using time-into-decel */
        float td     = trajCurrentTime - (trajTAccel + trajTConst);
        float dDecel = V_max * td - 0.5f * A_max * td * td;
        trajSetpointPos = (int64_t)(trajStartPos
            + trajDirection * trajDAccel
            + trajDirection * (trajTConst * V_max)
            + trajDirection * dDecel);

    } else {
        /* Trajectory complete — hold at target */
        trajSetpointPos = trajTargetPos;
    }
}

/* ===========================================================================
 * PID position controller
 *
 * A discrete PID controller that drives the motor to a target encoder position.
 * Output is split across two H-bridge PWM channels for bidirectional control:
 *   output > 0  → CH3 driven, CH4 = 0  (forward)
 *   output < 0  → CH3 = 0,   CH4 driven (reverse)
 * ===========================================================================*/
static float   Kp = 0.0001f;
static float   Ki = 0.0f;
static float   Kd = 0.0f;

static int64_t pidPrevError = 0;    /* Error from the previous cycle (for derivative) */
static int64_t pidIntegral  = 0;    /* Accumulated error (anti-windup clamped) */
static int64_t pidSetpoint  = 0;    /* Target position used in MOTOR_PID mode */

/**
 * @brief  Execute one PID control cycle and update PWM outputs.
 *
 *         Reads the current encoder position, computes the PID terms, and
 *         applies the result to the H-bridge.  The integral is clamped to
 *         ±INTEGRAL_MAX to prevent wind-up during large position errors.
 *
 * @param  setpoint  Desired encoder position for this cycle.
 */
static void update_PID(int64_t setpoint)
{
    int64_t curPos     = get_full_encoder_count();
    int64_t error      = setpoint - curPos;

    /* Integral with anti-windup clamp */
    pidIntegral += error;
    if (pidIntegral >  INTEGRAL_MAX) pidIntegral =  INTEGRAL_MAX;
    if (pidIntegral < -INTEGRAL_MAX) pidIntegral = -INTEGRAL_MAX;

    int64_t derivative = error - pidPrevError;
    pidPrevError       = error;

    /* PID output — divided by 10 to scale into PWM range */
    double  output = (Kp * (double)error
                    + Ki * (double)pidIntegral
                    + Kd * (double)derivative) / 10.0;

    /* Take absolute value for the duty cycle; direction set by channel selection */
    int64_t pwm = (int64_t)(output < 0.0 ? -output : output);
    if (pwm > (int64_t)htim1.Init.Period)
        pwm = (int64_t)htim1.Init.Period;

    if (output > 0.0) {
        UpdatePWMDutyCycle(TIM_CHANNEL_3, (uint32_t)pwm);  /* Forward */
        UpdatePWMDutyCycle(TIM_CHANNEL_4, 0);
    } else {
        UpdatePWMDutyCycle(TIM_CHANNEL_3, 0);
        UpdatePWMDutyCycle(TIM_CHANNEL_4, (uint32_t)pwm);  /* Reverse */
    }
}

/* ===========================================================================
 * Command processing
 *
 * Commands arrive as ASCII strings terminated by '\n' and are written into
 * a circular DMA buffer by the hardware.  ProcessCommand() scans the buffer
 * for newlines and dispatches to handleCommand() which parses the keyword
 * and optional numeric value.
 * ===========================================================================*/
static int      driveMode = MOTOR_FLOAT;
static uint32_t logPeriod = 0;     /* Logging interval in ms; 0 = disabled */
static uint32_t confirmOn = 0;     /* 1 = print verbose confirmations */

/**
 * @brief  Scan a DMA receive buffer for complete newline-terminated commands
 *         and dispatch each one to handleCommand().
 *
 *         Uses lastIdx as a persistent cursor so it can be called repeatedly
 *         in the main loop without reprocessing old data.
 *
 * @param  buf      Pointer to the circular DMA receive buffer.
 * @param  bufSize  Total size of the buffer in bytes.
 * @param  hdmarx   DMA handle for this peripheral (used to read remaining count).
 * @param  lastIdx  Persistent index of the last byte already processed.
 */
static void ProcessCommand(uint8_t *buf, int bufSize,
                            DMA_HandleTypeDef *hdmarx, uint16_t *lastIdx)
{
    while (*lastIdx != (uint16_t)(bufSize - __HAL_DMA_GET_COUNTER(hdmarx)))
    {
        if (buf[*lastIdx] == '\n')
            handleCommand(buf, bufSize, *lastIdx);
        *lastIdx = (*lastIdx + 1) % (uint16_t)bufSize;
    }
}

/**
 * @brief  Parse and execute a single command ending at cmdIdx in the buffer.
 *
 *         Reconstructs the last MAX_CMD_LEN bytes from the circular buffer,
 *         then calls extractKeywordAndValue() (from helper.h) to split the
 *         command string from its numeric argument.
 *
 * @param  buf      The circular DMA receive buffer.
 * @param  bufSize  Total buffer size.
 * @param  cmdIdx   Index of the terminating '\n' character.
 */
static void handleCommand(uint8_t *buf, int bufSize, uint16_t cmdIdx)
{
    /* Reconstruct the last MAX_CMD_LEN characters ending at cmdIdx */
    char raw[MAX_CMD_LEN];
    for (int i = 0; i < MAX_CMD_LEN; i++) {
        int idx = (int)cmdIdx - i;
        if (idx < 0) idx += bufSize;
        raw[MAX_CMD_LEN - 1 - i] = (char)buf[idx];
    }

    char cmdStr[MAX_CMD_LEN] = {0};
    char valStr[MAX_CMD_LEN] = {0};
    extractKeywordAndValue(raw, MAX_CMD_LEN, cmdStr, valStr);

    const int hasVal = (valStr[0] != '\0');

    if (!strcmp_embedded(cmdStr, "SETSPD") && hasVal) {
        /* Set motor speed (0–100%). Currently prints confirmation only. */
        int spd = atoi_embedded(valStr);
        if (spd >= 0 && spd <= 100) {
            printString("Speed "); printInt64(spd); printChar('\n');
        }

    } else if (!strcmp_embedded(cmdStr, "SETPOS") && hasVal) {
        /* Move to an absolute encoder position using the PID controller. */
        pidSetpoint = atoi_embedded(valStr);
        printString("Pos "); printString(valStr); printChar('\n');
        driveMode = MOTOR_PID;

    } else if (!strcmp_embedded(cmdStr, "FLOAT")) {
        /* Coast — remove drive from both H-bridge channels. */
        driveMode = MOTOR_FLOAT;
        UpdatePWMDutyCycle(TIM_CHANNEL_3, 0);
        UpdatePWMDutyCycle(TIM_CHANNEL_4, 0);

    } else if (!strcmp_embedded(cmdStr, "BRAKE")) {
        /* Active brake — apply 50% PWM to both channels simultaneously. */
        driveMode = MOTOR_BRAKE;
        uint32_t half = htim1.Init.Period / 2;
        UpdatePWMDutyCycle(TIM_CHANNEL_3, half);
        UpdatePWMDutyCycle(TIM_CHANNEL_4, half);

    } else if (!strcmp_embedded(cmdStr, "GETSPD")) {
        /* Report encoder speed in ticks/ms. */
        printFloat((float)computeEncoderCounterPeriod(), 4); printChar('\n');

    } else if (!strcmp_embedded(cmdStr, "GETPOS")) {
        /* Read position (output suppressed; extend here if needed). */
        (void)get_full_encoder_count();

    } else if (!strcmp_embedded(cmdStr, "GET_PID")) {
        /* Print all three PID gain values. */
        printString("Kp "); printFloat(Kp, 7);
        printString(",Ki "); printFloat(Ki, 7);
        printString(",Kd "); printFloat(Kd, 7);
        printChar('\n');

    } else if (!strcmp_embedded(cmdStr, "SET_P") && hasVal) {
        Kp = ratof(valStr);

    } else if (!strcmp_embedded(cmdStr, "SET_I") && hasVal) {
        Ki = ratof(valStr);

    } else if (!strcmp_embedded(cmdStr, "SET_D") && hasVal) {
        Kd = ratof(valStr);

    } else if (!strcmp_embedded(cmdStr, "GET_ADC")) {
        /* Read and print instantaneous motor current in mA. */
        uint32_t raw = readADCValue();
        float    mA  = adcToMilliamps((float)raw);
        printString("ADC:"); printFloat(mA, 3); printString(" mA\n");

    } else if (!strcmp_embedded(cmdStr, "MOVETO") && hasVal) {
        /* Start a trapezoidal profile move to the specified position. */
        driveMode = MOTOR_TRAPEZOID;
        int64_t tgt = atoi_embedded(valStr);
        if (confirmOn) { printString("Moving to "); printString(valStr); printChar('\n'); }
        initializeTrajectory(get_full_encoder_count(), tgt);

    } else if (!strcmp_embedded(cmdStr, "LOG") && hasVal) {
        /* Enable periodic telemetry logging at the given interval (ms). */
        logPeriod = (uint32_t)atoi_embedded(valStr);

    } else if (!strcmp_embedded(cmdStr, "CONFIRM") && hasVal) {
        /* Enable/disable verbose command confirmations (0 = off, 1 = on). */
        confirmOn = (uint32_t)atoi_embedded(valStr);

    } else if (!strcmp_embedded(cmdStr, "SETMAX_VEL") && hasVal) {
        V_max = ratof(valStr);

    } else if (!strcmp_embedded(cmdStr, "SETMAX_ACC") && hasVal) {
        A_max = ratof(valStr);

    } else if (!strcmp_embedded(cmdStr, "STATUS")) {
        /* Drive mode status — output suppressed; extend here if needed. */

    } else if (!strcmp_embedded(cmdStr, "GETMAX_VEL")) {
        /* Suppressed — uncomment print calls to enable. */

    } else if (!strcmp_embedded(cmdStr, "GETMAX_ACC")) {
        /* Suppressed — uncomment print calls to enable. */

    } else if (!strcmp_embedded(cmdStr, "RESET")) {
        /* System reset — uncomment NVIC_SystemReset() to enable. */
    }
}

/* ===========================================================================
 * I2C slave transmit IRQ handler
 *
 * Called from the I2C1 IRQ vector (wired in via the HAL or stm32c0xx_it.c).
 * Drains the I2C TX cyclic buffer into the hardware TXDR register one byte
 * at a time, waiting for the TXIS (transmit register empty) flag between
 * bytes.  Exits cleanly on STOPF (master issued a STOP condition) or timeout.
 * ===========================================================================*/

/**
 * @brief  Service the I2C slave transmit path from the I2C1 IRQ handler.
 * @param  hi2c  Pointer to the I2C handle (hi2c1).
 */
void I2C_IRQHandler_User(I2C_HandleTypeDef *hi2c)
{
    /* Nothing to do if neither TXIS nor STOPF is pending */
    if (!__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) &&
        !__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF))
        return;

    while (!__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF))
    {
        uint32_t t0 = HAL_GetTick();

        /* Wait for hardware to be ready to accept the next byte */
        while (!__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS))
        {
            if ((HAL_GetTick() - t0) > I2C_TXIS_TIMEOUT_MS)
                return;  /* Timeout — abort to avoid hanging in IRQ */

            if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) {
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
                if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR))
                    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
                return;
            }
        }

        /* Send next byte from the cyclic buffer, or NUL if the buffer is empty */
        char ch = '\0';
        buffer_remove(&ch);
        hi2c->Instance->TXDR = (uint8_t)ch;
    }

    /* Clear STOPF and ADDR flags at end of transaction */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR))
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
}

/* ===========================================================================
 * HAL callbacks
 * ===========================================================================*/

/**
 * @brief  UART RX complete callback.
 *         DMA is already in circular mode so a restart is not strictly needed,
 *         but is kept here as a safety net against DMA de-sync.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(&huart2, rxBuffer, DMA_BUF_SIZE);
}

/**
 * @brief  I2C slave RX complete callback.
 *         Prints the received bytes and restarts DMA reception for the next
 *         incoming I2C transaction.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    printString("I2C Received: ");
    printBuffer(i2cRxBuffer, I2C_BUF_SIZE);
    HAL_I2C_Slave_Receive_DMA(hi2c, i2cRxBuffer, I2C_BUF_SIZE);
}

/**
 * @brief  I2C address match callback (unused — placeholder for future use).
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr)
{
    (void)hi2c; (void)dir; (void)addr;
}

/* ===========================================================================
 * main
 * ===========================================================================*/
int main(void)
{
    /* --- Core init --- */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();

    HAL_TIM_Encoder_Start_optimized_(&htim3);
    __enable_irq();

    /* Start UART DMA reception into the circular rxBuffer */
    HAL_UART_Receive_DMA(&huart2, rxBuffer, DMA_BUF_SIZE);
    HAL_Delay(1000);

    /*
     * Re-initialise I2C to clear any glitches from startup.
     * DeInit + re-Init forces the peripheral back to a known state.
     */
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(100);
    HAL_I2C_Init(&hi2c1);
    HAL_Delay(500);
    while (hi2c1.State != HAL_I2C_STATE_READY) {}
    HAL_Delay(500);

    /* Start I2C slave DMA reception; print result code for debug */
    HAL_StatusTypeDef i2cResult = HAL_I2C_Slave_Receive_DMA(&hi2c1, i2cRxBuffer, I2C_BUF_SIZE);
    printUInt64((uint64_t)i2cResult);
    HAL_Delay(100);

    /*
     * Pare back I2C interrupts to only those needed for slave TX:
     * disable error/NACK/RX IRQs, clear stale flags, then enable
     * STOP, ADDR and TX-buffer-empty interrupts.
     */
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_NACKI | I2C_IT_RXI);
    HAL_Delay(100);
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF | I2C_FLAG_STOPF | I2C_FLAG_OVR |
                                  I2C_FLAG_BUSY | I2C_FLAG_ALERT | I2C_FLAG_BERR);
    __HAL_I2C_ENABLE_IT(&hi2c1, I2C_IT_STOPI | I2C_IT_ADDRI | I2C_IT_TXI);

    /* Capture initial encoder state for speed measurement */
    encLastPos  = get_full_encoder_count();
    encLastTick = HAL_GetTick();

    uint32_t adcLastTick    = HAL_GetTick();
    uint32_t encLogLastTick = HAL_GetTick();
    uint32_t frameLastTick  = HAL_GetTick();

    static uint16_t uartParseIdx = 0;   /* Persistent UART DMA read cursor */
    static uint16_t i2cParseIdx  = 0;   /* Persistent I2C  DMA read cursor */

    /* ==== Main control loop ==== */
    while (1)
    {
        /* --- Process incoming commands from UART and I2C --- */
        ProcessCommand(rxBuffer,    DMA_BUF_SIZE, huart2.hdmarx, &uartParseIdx);
        ProcessCommand(i2cRxBuffer, I2C_BUF_SIZE, hi2c1.hdmarx,  &i2cParseIdx);

        /* Clear I2C error flags that may be set during normal bus activity */
        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF | I2C_FLAG_STOPF |
                                      I2C_FLAG_OVR | I2C_FLAG_BUSY);

        /* --- Update encoder snapshot for speed calculation --- */
        encCurPos  = get_full_encoder_count();
        encCurTick = HAL_GetTick();

        /* --- Motion control --- */
        if (driveMode == MOTOR_TRAPEZOID) {
            uint32_t now  = HAL_GetTick();
            float    dt   = (float)(now - frameLastTick) / 1000.0f;
            frameLastTick = now;
            updateTrajectory(dt);           /* Advance profile setpoint */
            update_PID(trajSetpointPos);    /* Track profile with PID */
        }

        if (driveMode == MOTOR_PID)
            update_PID(pidSetpoint);        /* Hold fixed position with PID */

        /* --- Periodic telemetry logging --- */
        if (logPeriod != 0) {
            if (logPeriod < 20) logPeriod = 20;  /* Enforce minimum 20 ms interval */

            static int64_t prevEncCount = 0;

            /* Sample ADC into the SMA buffer every 80 ms */
            if (HAL_GetTick() - adcLastTick > 80) {
                adcbuf_insert(readADCValue());
                adcLastTick = HAL_GetTick();
            }

            /* Emit telemetry line at logPeriod interval */
            if (HAL_GetTick() - encLogLastTick > logPeriod) {
                int64_t  pos     = get_full_encoder_count();
                float    mA      = adcToMilliamps(adcbuf_sma());
                uint32_t delta_t = HAL_GetTick() - encLogLastTick;
                int64_t  delta_c = pos - prevEncCount;

                /* Ticks per second — guard against zero/stale delta_t */
                int64_t tps = (delta_t == 0)   ? 99999999LL
                            : (delta_t > 1000) ? 0LL
                            : delta_c * 1000 / (int64_t)delta_t;

                printString("pos ");   printInt64(pos);
                printString(",amps "); printFloat(mA, 3);
                printString(",tps ");  printInt64(tps);
                printChar('\n');

                prevEncCount   = pos;
                encLastPos     = encCurPos;
                encLastTick    = encCurTick;
                encLogLastTick = HAL_GetTick();
            }
        }
    }
}

/* ===========================================================================
 * Peripheral initialisation functions (generated by STM32CubeMX, lightly
 * cleaned up).  Each function configures one peripheral and calls
 * Error_Handler() on any HAL failure.
 * ===========================================================================*/

/** @brief Configure ADC1 for single-conversion software-triggered reads on CH8. */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig;

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode      = DISABLE;
    hadc1.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_8;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/** @brief Configure the system clock: HSI oscillator / 4 → 4 MHz SYSCLK. */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {
        .OscillatorType      = RCC_OSCILLATORTYPE_HSI,
        .HSIState            = RCC_HSI_ON,
        .HSIDiv              = RCC_HSI_DIV4,
        .HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT,
    };
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    RCC_ClkInitTypeDef clk = {
        .ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1,
        .SYSCLKSource   = RCC_SYSCLKSOURCE_HSI,
        .SYSCLKDivider  = RCC_SYSCLK_DIV1,
        .AHBCLKDivider  = RCC_HCLK_DIV1,
        .APB1CLKDivider = RCC_APB1_DIV1,
    };
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/**
 * @brief Configure I2C1 as a 7-bit slave at address 0x20 (100 kHz).
 *        PB7 = SCL, PC14 = SDA.  Clock stretching disabled (NoStretch).
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x40000A0B;  /* 100 kHz @ PCLK1 */
    hi2c1.Init.OwnAddress1      = 64;           /* 7-bit address 0x20 (shifted) */
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_ENABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();

    /* Route I2C1 clock to PCLK1 */
    RCC_PeriphCLKInitTypeDef pclk = {
        .PeriphClockSelection = RCC_PERIPHCLK_I2C1,
        .I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1,
    };
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) Error_Handler();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB7 → SCL, PC14 → SDA (open-drain, alternate function 14) */
    GPIO_InitTypeDef gpio = {
        .Mode      = GPIO_MODE_AF_OD,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF14_I2C1,
    };
    gpio.Pin = GPIO_PIN_7;  HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_14; HAL_GPIO_Init(GPIOC, &gpio);

    __HAL_RCC_I2C1_CLK_ENABLE();
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/**
 * @brief Configure TIM1 for dual-channel PWM output (H-bridge drive).
 *        Period = 65535 counts; both channels start at 0% duty cycle.
 *        CH3 = forward, CH4 = reverse.
 */
static void MX_TIM1_Init(void)
{
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 65535;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    TIM_MasterConfigTypeDef master = {
        .MasterOutputTrigger  = TIM_TRGO_RESET,
        .MasterOutputTrigger2 = TIM_TRGO2_RESET,
        .MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE,
    };
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {
        .OCMode       = TIM_OCMODE_PWM1,
        .Pulse        = 0,
        .OCPolarity   = TIM_OCPOLARITY_HIGH,
        .OCNPolarity  = TIM_OCNPOLARITY_HIGH,
        .OCFastMode   = TIM_OCFAST_DISABLE,
        .OCIdleState  = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };
    if (HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    TIM_BreakDeadTimeConfigTypeDef bdt = {
        .OffStateRunMode  = TIM_OSSR_DISABLE,
        .OffStateIDLEMode = TIM_OSSI_DISABLE,
        .LockLevel        = TIM_LOCKLEVEL_OFF,
        .DeadTime         = 0,
        .BreakState       = TIM_BREAK_DISABLE,
        .BreakPolarity    = TIM_BREAKPOLARITY_HIGH,
        .BreakFilter      = 0,
        .BreakAFMode      = TIM_BREAK_AFMODE_INPUT,
        .Break2State      = TIM_BREAK2_DISABLE,
        .Break2Polarity   = TIM_BREAK2POLARITY_HIGH,
        .Break2Filter     = 0,
        .Break2AFMode     = TIM_BREAK_AFMODE_INPUT,
        .AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE,
    };
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief Configure TIM3 for quadrature encoder input on CH3 (PB0) and CH4 (PB1).
 *        The update interrupt is enabled so TIM3_IRQHandler can extend the
 *        16-bit counter to 64 bits.
 */
static void MX_TIM3_Init(void)
{
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) Error_Handler();

    TIM_Encoder_InitTypeDef enc = {
        .EncoderMode  = TIM_ENCODERMODE_TI1,   /* Count on TI1 edges only */
        .IC1Polarity  = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter    = 0,
        .IC2Polarity  = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter    = 0,
    };
    if (HAL_TIM_Encoder_Init(&htim3, &enc) != HAL_OK) Error_Handler();

    TIM_MasterConfigTypeDef master = {
        .MasterOutputTrigger = TIM_TRGO_RESET,
        .MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE,
    };
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) Error_Handler();

    TIM_IC_InitTypeDef ic = {
        .ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING,
        .ICSelection = TIM_ICSELECTION_DIRECTTI,
        .ICPrescaler = TIM_ICPSC_DIV1,
        .ICFilter    = 0,
    };
    if (HAL_TIM_IC_ConfigChannel_optimized(&htim3, &ic, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_ConfigChannel_optimized(&htim3, &ic, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    NVIC_EnableIRQ(TIM3_IRQn);
}

/** @brief GPIO MSP init for TIM3 encoder (PB0, PB1 as AF push-pull, pull-down). */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin       = GPIO_PIN_0 | GPIO_PIN_1,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_PULLDOWN,
        .Speed     = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF1_TIM1,
    };
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/** @brief Configure USART2: 115200 8N1, TX+RX, no hardware flow control. */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = 115200;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/**
 * @brief Configure DMA1 channels for USART2 RX, USART2 TX, and I2C1 RX.
 *
 *        Channel assignments:
 *          DMA1_Ch1 — USART2 RX (circular, low priority)
 *          DMA1_Ch2 — USART2 TX (normal,   low priority)
 *          DMA1_Ch3 — I2C1   RX (circular, high priority)
 */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* I2C1 RX — DMA1 Channel 3 (circular for continuous slave reception) */
    hdma_i2c1_rx.Instance                 = DMA1_Channel3;
    hdma_i2c1_rx.Init.Request             = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2c1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

    /* USART2 RX — DMA1 Channel 1 (circular for continuous reception) */
    hdma_usart2_rx.Instance                 = DMA1_Channel1;
    hdma_usart2_rx.Init.Request             = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* USART2 TX — DMA1 Channel 2 (normal mode, one transfer at a time) */
    hdma_usart2_tx.Instance                 = DMA1_Channel2;
    hdma_usart2_tx.Init.Request             = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    /* NVIC priorities — all at preemption 0; UART/I2C at sub-priority 1 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,   0, 0); HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0); HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn,          0, 1); HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(I2C1_IRQn,            0, 1); HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/** @brief Enable GPIO clocks and configure PF2 as a digital input (no pull). */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin  = GPIO_PIN_2,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(GPIOF, &gpio);
}

/** @brief Default error handler — halts execution in an infinite loop. */
void Error_Handler(void)
{
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
