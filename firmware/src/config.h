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

/* --------------- Manual output / waveform (when PID is off) --------------- */
/* Output mode: 0 = DC (manual voltage), 1 = triangle, 2 = sine              */
#define OUT_MODE_DC              0
#define OUT_MODE_TRIANGLE        1
#define OUT_MODE_SINE            2

#define DEFAULT_OUT_MODE         OUT_MODE_DC
#define DEFAULT_MANUAL_V         0.0f
#define DEFAULT_WAVE_FREQ_HZ     1.0f   /* Hz */
#define DEFAULT_WAVE_AMPLITUDE   0.5f   /* peak, in volts */
#define DEFAULT_WAVE_OFFSET      0.0f   /* DC offset, in volts */

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
#define AUTOTUNE_MIN_CYCLES      3       /* full cycles to average           */
#define AUTOTUNE_SETTLE_CYCLES   1       /* initial cycles to discard        */
#define AUTOTUNE_TIMEOUT_S       30.0f   /* abort after this many seconds    */

/* --------------- Sample and hold ------------------------------------------ */
/* While the gate is asserted the PID is frozen: integrator, input filter and  */
/* DAC output keep their last values. On release the loop resumes bumplessly.  */
/* Gate source (per channel, runtime selectable):                              */
/*   0 = off, 1 = external TTL on a DIO<n>_P pin of E1, 2 = input threshold,   */
/*   3 = TTL on the OTHER fast analog input (IN2 gates channel 1 and vice      */
/*       versa). A 5 V TTL is safe on an LV input (abs. max 6 V) and clips at  */
/*       +1 V, so hold_thresh ~0.5 V / hold_hyst ~0.1 V work without divider.  */
/* A manual hold (SET <ch> hold 1) is OR-ed in regardless of the mode.         */
#define HOLD_MODE_OFF            0
#define HOLD_MODE_DIO            1
#define HOLD_MODE_THRESHOLD      2
#define HOLD_MODE_ANALOG         3

#define DEFAULT_HOLD_MODE        HOLD_MODE_OFF
/* DIO pin index n -> DIO<n>_P. Channel i defaults to DIO<i>_P (E1 pin 3 + 2i). */
#define DEFAULT_HOLD_PIN(ch)     (ch)
#define DEFAULT_HOLD_POLARITY    0       /* 0: hold while pin LOW, 1: while HIGH */
#define DEFAULT_HOLD_THRESH_V    0.05f   /* threshold mode: hold while raw < this */
#define DEFAULT_HOLD_HYST_V      0.01f   /* ... release when raw > thresh + hyst  */
#define DEFAULT_HOLD_DEBOUNCE    2       /* consecutive gated samples to enter    */
#define DEFAULT_RESUME_DELAY_US  200     /* keep holding this long after release  */
/* What "last output" means when a hold starts. By default it is the output
 * of the previous loop iteration. If the gate arrives a little after the beam
 * is already gone, that value contains the controller's reaction to the
 * invalid input (a proportional kick, integrator wind-up). hold_lookback_ms
 * takes the output from that long BEFORE the gate instead, and hold_avg_ms
 * averages over a window of that length ending there. 0/0 = old behaviour. */
#define DEFAULT_HOLD_LOOKBACK_MS 0
#define DEFAULT_HOLD_AVG_MS      0
#define HOLD_HIST_LEN            8192    /* output history kept per channel (samples) */
#define DEFAULT_HOLD_PARK        0       /* 0: hold last output, 1: park output   */
#define DEFAULT_HOLD_PARK_V      0.0f    /* park voltage when hold_park = 1       */

/* --------------- Step-response capture --------------------------------- */
/* SET <ch> step <delta_v> applies a setpoint step and records input/output   */
/* at the loop rate; the capture is streamed to the requesting client as a    */
/* "STEP" header followed by "S" lines. Used by scripts/optimize_step.py.     */
#define STEP_MAX                 4096    /* samples per capture                */
#define DEFAULT_STEP_MS          20      /* capture length after the step (ms) */
#define DEFAULT_STEP_PRE_MS      1       /* pre-trigger length (ms)            */
#define STEP_IDLE                0
#define STEP_ARMED               1
#define STEP_RUNNING             2
#define STEP_DONE                3
#define STEP_ABORTED             4

/* --------------- External setpoint ------------------------------------- */
/* The setpoint can follow an analog control voltage on the OTHER fast input */
/* (IN2 for channel 1):                                                       */
/*   setpoint = sp_ext_min + (sp_ext_max - sp_ext_min) * clamp(ctrl/sp_ext_ref, 0, 1) */
/* The range is normally set by a calibration (SET <ch> spcal 1): the output  */
/* ramps to the PID out_min, the input is averaged ("zero" input), the output */
/* ramps to out_max, the input is averaged again ("full" input), then         */
/*   sp_ext_min = zero,  sp_ext_max = zero + sp_ext_pct/100 * (full - zero).  */
/* The output ramps back and the PID resumes bumplessly.                      */
#define SP_SOURCE_INTERNAL       0
#define SP_SOURCE_EXTERNAL       1
#define DEFAULT_SP_EXT_PCT       90.0f   /* max setpoint, % of the zero..full span */
#define DEFAULT_SP_EXT_REF_V     1.0f    /* control voltage that gives the max setpoint */
#define DEFAULT_SP_EXT_ALPHA     0.2f    /* LPF on the control voltage (1 = none) */
#define DEFAULT_SPCAL_RAMP_MS    200     /* each output ramp                  */
#define DEFAULT_SPCAL_AVG_MS     100     /* settle time, then averaging time  */
/* Only samples with the hold gate idle (beam on) are averaged; the averaging
 * window extends until enough were seen, at most this many times avg_ms.   */
#define SPCAL_GATE_TIMEOUT       50
#define SPCAL_IDLE               0
#define SPCAL_ARMED              1
#define SPCAL_RAMP_MIN           2       /* 2..8: running                     */
#define SPCAL_SETTLE_MIN         3
#define SPCAL_MEASURE_MIN        4
#define SPCAL_RAMP_MAX           5
#define SPCAL_SETTLE_MAX         6
#define SPCAL_MEASURE_MAX        7
#define SPCAL_RAMP_BACK          8
#define SPCAL_DONE               9
#define SPCAL_FAILED             10
#define SPCAL_ABORT              11      /* requested by the client            */

/* --------------- Fast PSD (power spectral density, bulk-read from FPGA) --- */
#define PSD_N                    16384   /* FFT length = RP acq buffer depth */
#define PSD_BINS                 (PSD_N / 2 + 1)   /* one-sided spectrum    */
#define PSD_DEFAULT_AVG          8       /* Welch segments to average        */
#define PSD_DEFAULT_INTERVAL_MS  1000    /* ms between PSD updates           */
#define PSD_FS                   125e6f  /* ADC sample rate with RP_DEC_1    */

#endif /* CONFIG_H */
