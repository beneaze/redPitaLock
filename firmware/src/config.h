#ifndef CONFIG_H
#define CONFIG_H

#define NUM_CHANNELS  2

/* --------------- Default PID gains (per-channel, adjustable at runtime) --- */
#define DEFAULT_KP               4.0f
#define DEFAULT_KI               2.0f
#define DEFAULT_KD               0.04f

/* --------------- Feedback polarity ---------------------------------------- */
/*  +1: more drive = more signal (diffracted beam)                            */
/*  -1: more drive = less signal (0th-order / pass-through)                   */
#define DEFAULT_ERROR_SIGN       1.0f

/* --------------- Setpoint (volts, in the *physical* input domain) --------- */
#define DEFAULT_SETPOINT_V       1.5f

/* --------------- PID output limits (normalised: 0 = null, 1 = max) -------- */
#define PID_OUT_MIN              0.0f
#define PID_OUT_MAX              1.0f

/* --------------- Anti-windup: max integral accumulator (V*s) -------------- */
#define PID_INTEGRAL_MAX         0.5f

/* --------------- Control-loop period (microseconds) ----------------------- */
/* Adjustable at runtime via TCP; this is just the power-on default.          */
#define DEFAULT_LOOP_PERIOD_US   1000   /* 1 kHz */

/* --------------- Analog I/O calibration ----------------------------------- */
/*                                                                            */
/* physical_V = raw_rp_V * INPUT_SCALE + INPUT_OFFSET                         */
/*                                                                            */
/* For a 3.3:1 resistive divider on each input (maps 0-3.3 V to 0-1 V on     */
/* the Red Pitaya LV input):                                                  */
#define DEFAULT_INPUT_SCALE      3.3f
#define DEFAULT_INPUT_OFFSET     0.0f

/* raw_rp_V = (physical_V - OUTPUT_OFFSET) / OUTPUT_SCALE                     */
/*                                                                            */
/* Adjust per channel to match your external op-amp conditioning stage.        */
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

#endif /* CONFIG_H */
