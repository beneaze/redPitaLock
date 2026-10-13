#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "pid.h"
#include "analog_io.h"
#include "autotune.h"
#include "psd.h"

/*
 * Per-channel runtime state shared between the PID thread and the TCP server.
 * The PID thread writes telemetry fields; the TCP thread reads them and may
 * update parameter fields (guarded by the PID thread re-reading each loop).
 */
typedef struct {
    /* --- Parameters (written by TCP thread, read by PID thread) --- */
    volatile int     enabled;         /* 0 = PID off (default), 1 = on       */
    volatile float   setpoint_v;
    volatile float   error_sign;
    volatile int     loop_period_us;
    volatile int     use_lut;
    pid_state_t      pid;             /* gains modified via tcp SET commands  */
    analog_cal_t     cal;

    /* --- Manual output / waveform (active when PID is off) -------------- */
    volatile int     out_mode;        /* OUT_MODE_DC / TRIANGLE / SINE       */
    volatile float   manual_v;        /* DC voltage when out_mode == DC      */
    volatile float   wave_freq_hz;    /* waveform frequency (Hz)             */
    volatile float   wave_amplitude;  /* waveform peak amplitude (V)         */
    volatile float   wave_offset;     /* waveform DC offset (V)              */

    /* --- Autotune --------------------------------------------------------- */
    autotune_state_t autotune;
    volatile float   autotune_relay_amp;    /* configurable relay amplitude    */
    volatile float   autotune_hysteresis;   /* configurable noise band         */

    /* --- Sample and hold (parameters: TCP thread -> PID thread) ---------- */
    volatile int     hold_mode;        /* HOLD_MODE_OFF / DIO / THRESHOLD      */
    volatile int     hold_manual;      /* 1 = hold forced by SET <ch> hold 1   */
    volatile int     hold_pin;         /* DIO<n>_P index (0..7), DIO mode      */
    volatile int     hold_polarity;    /* 0 = hold while pin LOW, 1 = HIGH     */
    volatile float   hold_thresh_v;    /* threshold mode: hold while raw < th  */
    volatile float   hold_hyst_v;      /* release when raw > th + hyst         */
    volatile int     hold_debounce;    /* consecutive gated samples to enter   */
    volatile int     resume_delay_us;  /* keep holding after the gate releases */
    volatile int     hold_park;        /* 1 = drive hold_park_v while holding  */
    volatile float   hold_park_v;

    /* --- Telemetry (written by PID thread, read by TCP thread) ---  */
    volatile double  telem_time_s;         /* CLOCK_MONOTONIC secs since start */
    volatile float   telem_input_v;
    volatile float   telem_output_v;       /* target (pre-clamp)              */
    volatile float   telem_actual_output_v; /* actual (post-clamp, on the DAC) */
    volatile int     telem_holding;        /* 1 while the PID output is held   */
    volatile int     telem_gate;           /* raw gate state (1 = asserted)    */
    volatile int     telem_hold_count;     /* hold events since start          */

    /* --- PSD (written by PSD thread, read/cleared by TCP thread) -------- */
    volatile int     psd_ready;            /* 1 = new PSD frame available     */
    volatile float   psd_bins[PSD_BINS];   /* one-sided PSD (V^2/Hz)          */
    volatile float   psd_fs;               /* sample rate for this PSD frame  */
    volatile int     psd_avg;              /* Welch segments to average        */
    volatile int     psd_interval_ms;      /* ms between PSD updates           */
} channel_state_t;

/* Starts the TCP server on TCP_PORT.  Runs forever (call from a thread).
 * `channels` must point to an array of NUM_CHANNELS channel_state_t.        */
void tcp_server_run(channel_state_t *channels);

#endif /* TCP_SERVER_H */
