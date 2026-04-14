#ifndef CONFIG_H
#define CONFIG_H

#define NUM_CHANNELS  2

/* --------------- Default PID gains (per-channel, adjustable at runtime) --- */
#define DEFAULT_KP               4.0f
#define DEFAULT_KI               2.0f
/* Start with D=0; add small Kd only if needed. D uses meas derivative (see pid.c). */
#define DEFAULT_KD               0.0f

/* --------------- Feedback polarity ---------------------------------------- */
/*  +1: more drive = more signal (diffracted beam)                            */
/*  -1: more drive = less signal (0th-order / pass-through)                   */
#define DEFAULT_ERROR_SIGN       1.0f

/* --------------- Setpoint (V at fast ADC, LV range ~ +/-1 V) ---------------- */
#define DEFAULT_SETPOINT_V       0.50f

/* --------------- PID output limits (normalised: 0 = null, 1 = max) -------- */
#define PID_OUT_MIN              0.0f
#define PID_OUT_MAX              1.0f

/* --------------- Anti-windup: max integral accumulator (V*s) -------------- */
#define PID_INTEGRAL_MAX         0.5f

/* --------------- Control-loop period (microseconds) ----------------------- */
/* Adjustable at runtime via TCP; this is just the power-on default.          */
#define DEFAULT_LOOP_PERIOD_US   1000   /* 1 kHz */

/* --------------- Analog I/O calibration ----------------------------------- */
/* No scaling -- raw Red Pitaya voltages are used directly.                    */
#define DEFAULT_INPUT_SCALE      1.0f
#define DEFAULT_INPUT_OFFSET     0.0f
#define DEFAULT_OUTPUT_SCALE     1.0f
#define DEFAULT_OUTPUT_OFFSET    0.0f

/* Clamp the raw Red Pitaya output to the hardware range.                     */
#define RP_OUTPUT_MIN           -1.0f
#define RP_OUTPUT_MAX            1.0f

/* --------------- AOM LUT -------------------------------------------------- */
/* Per-channel flag: when true the PID's normalised output (0-1) is passed    */
/* through aom_linearize() before hitting the DAC.                            */
#define DEFAULT_USE_LUT          0

/* --------------- TCP server ----------------------------------------------- */
#define TCP_PORT                 5000
#define TCP_MAX_CLIENTS          4

/* --------------- Telemetry decimation ------------------------------------- */
/* Send one telemetry frame every N loop iterations (per channel).            */
#define DEFAULT_TELEMETRY_DECIM  10

/* First-order LPF on ADC before PID: y += alpha*(raw-y). Higher = faster, noisier. */
#define INPUT_LPF_ALPHA          0.20f

/* --------------- Autotune (relay-feedback, Astrom-Hagglund) --------------- */
#define AUTOTUNE_RELAY_AMP       0.50f   /* relay output half-amplitude (V)  */
#define AUTOTUNE_HYSTERESIS      0.005f  /* noise band around setpoint (V)   */
#define AUTOTUNE_MIN_CYCLES      5       /* full cycles to average           */
#define AUTOTUNE_SETTLE_CYCLES   2       /* initial cycles to discard        */
#define AUTOTUNE_TIMEOUT_S       10.0f   /* abort after this many seconds    */

#endif /* CONFIG_H */
