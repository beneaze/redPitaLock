#ifndef CONFIG_H
#define CONFIG_H

#define NUM_CHANNELS  2

/* ==========================================================================
 * Hardware PI -- the actual control loop runs in the FPGA at 125 MHz
 * (~8 ns latency).  The C daemon is just a parameter pusher: it writes
 * gains to the FPGA whenever the user changes them, samples telemetry at
 * ~1 kHz, drives autotune, and handles PSD.
 *
 * The FPGA bitstream is the stock RP `red_pitaya_pid_block.v` (which has a
 * Kd register), but we drive it as PI -- see PI_PSR / PI_ISR below and
 * fpga_pi.c for why Kd is held at zero.
 * ========================================================================== */

/* --------------- FPGA PI register block (v0.94 bitstream, CS[3]) ---------- */
#define FPGA_PI_BASE             0x40300000UL
#define FPGA_PI_MAP_SIZE         0x1000UL

/* Hardware loop runs every ADC sample. */
#define HARDWARE_LOOP_HZ         125000000

/* red_pitaya_pid_block.v fixed-point shifts (signed 14-bit registers).
 * The block has a Kd register too (DSR=10) but its representable range is
 * only ~64 ns of derivative time, well below anything physically useful for
 * this plant -- the firmware does not expose Kd at all (PI mode).            */
#define PI_PSR                   12   /* P shift: kp_eff = set_kp / 2^12     */
#define PI_ISR                   18   /* I shift: ki_eff = set_ki * fs/2^18  */

/* 14-bit ADC and DAC, +/- 1 V LV range. */
#define ADC_LSB_PER_VOLT         8192.0f
#define DAC_LSB_PER_VOLT         8192.0f

/* Effective representable gain ranges (informational; helpful for tuning):
 *   Kp_user:  +/- (8191 / 4096)        ~ +/- 2.0  V/V
 *   Ki_user:  smallest step  ~ 477 Hz, max ~ 3.9e6 Hz                       */

/* --------------- Default PI gains (per channel, adjustable at runtime) --- */
#define DEFAULT_KP               0.5f       /* V / V                         */
#define DEFAULT_KI               1000.0f    /* 1 / s   (>= ~477 Hz step)     */

/* --------------- Feedback polarity ---------------------------------------- */
/*  +1: more drive = more signal (diffracted beam)                            */
/*  -1: more drive = less signal (0th-order / pass-through)                   */
#define DEFAULT_ERROR_SIGN       1.0f

/* --------------- Setpoint (V at fast ADC, LV range ~ +/-1 V) ---------------- */
#define DEFAULT_SETPOINT_V       0.50f

/* --------------- Supervisor period ---------------------------------------- */
/* This is NOT the control-loop period (the loop runs at 125 MHz in the
 * FPGA).  This is how often the C supervisor wakes up to push parameter
 * changes and collect telemetry.  Compile-time only -- there's no value in
 * exposing it on the wire.                                                  */
#define SUPERVISOR_PERIOD_US     1000   /* 1 kHz                             */

/* --------------- Analog I/O ------------------------------------------------ */
/* Hardware DAC envelope.  This is the only output bound the firmware can
 * honestly enforce -- the v0.94 PID block has no per-channel software
 * output-min/max register, so the FPGA's instantaneous PI output saturates
 * at +/- 1 V and nowhere tighter.  If a downstream load (e.g. an AOM
 * driver) needs a tighter window, add an external clamp circuit (Schottky
 * pair to a reference voltage, or an op-amp limiter) on the OUT line.      */
#define RP_OUTPUT_MIN           -1.0f
#define RP_OUTPUT_MAX            1.0f

/* --------------- Manual output / waveform (when PI is off) ---------------- */
/* Output mode: 0 = DC (manual voltage), 1 = triangle, 2 = sine.
 * Triangle / sine are produced by the FPGA signal generator (ASG) so the
 * waveform is sample-accurate, not stepped at the supervisor rate.          */
#define OUT_MODE_DC              0
#define OUT_MODE_TRIANGLE        1
#define OUT_MODE_SINE            2

#define DEFAULT_OUT_MODE         OUT_MODE_DC
#define DEFAULT_MANUAL_V         0.0f
#define DEFAULT_WAVE_FREQ_HZ     1.0f   /* Hz */
#define DEFAULT_WAVE_AMPLITUDE   0.5f   /* peak, in volts */
#define DEFAULT_WAVE_OFFSET      0.0f   /* DC offset, in volts */

/* --------------- TCP server ----------------------------------------------- */
#define TCP_PORT                 5000
#define TCP_MAX_CLIENTS          4

/* --------------- Autotune (relay-feedback, Astrom-Hagglund) --------------- */
#define AUTOTUNE_RELAY_AMP       0.50f   /* relay output half-amplitude (V)  */
#define AUTOTUNE_HYSTERESIS      0.005f  /* noise band around setpoint (V)   */
#define AUTOTUNE_MIN_CYCLES      3       /* full cycles to average           */
#define AUTOTUNE_SETTLE_CYCLES   1       /* initial cycles to discard        */
#define AUTOTUNE_TIMEOUT_S       30.0f   /* abort after this many seconds    */

/* --------------- Fast PSD (power spectral density, bulk-read from FPGA) --- */
#define PSD_N                    16384   /* FFT length = RP acq buffer depth */
#define PSD_BINS                 (PSD_N / 2 + 1)   /* one-sided spectrum    */
#define PSD_DEFAULT_AVG          8       /* Welch segments to average        */
#define PSD_DEFAULT_INTERVAL_MS  1000    /* ms between PSD updates           */
#define PSD_FS                   125e6f  /* ADC sample rate with RP_DEC_1    */

#endif /* CONFIG_H */
