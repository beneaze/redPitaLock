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
    volatile int     hold_lookback_ms; /* hold the output from this long before the gate */
    volatile int     hold_avg_ms;      /* ... averaged over a window of this length      */
    volatile int     hold_park;        /* 1 = drive hold_park_v while holding  */
    volatile float   hold_park_v;

    /* --- External setpoint (see config.h) ------------------------------- */
    volatile int     sp_source;        /* SP_SOURCE_INTERNAL / EXTERNAL        */
    volatile float   sp_user_v;        /* setpoint entered by the user (internal mode) */
    volatile float   sp_ext_pct;       /* max setpoint as % of the full-power input */
    volatile float   sp_ext_min_v;     /* setpoint at ctrl <= 0 (V)            */
    volatile float   sp_ext_max_v;     /* setpoint at ctrl >= sp_ext_ref (V)   */
    volatile float   sp_ext_ref_v;     /* control voltage full scale (V)      */
    volatile float   sp_ext_alpha;     /* LPF coefficient on the control voltage */
    volatile int     spcal_state;      /* SPCAL_IDLE .. SPCAL_ABORT            */
    volatile int     spcal_ramp_ms;
    volatile int     spcal_avg_ms;
    volatile float   spcal_zero_v;     /* input measured at out_min (last calibration) */
    volatile float   spcal_full_v;     /* input measured at out_max            */
    volatile float   spcal_out_min_v;  /* outputs used for those measurements  */
    volatile float   spcal_out_v;
    volatile float   telem_sp_ext_v;   /* filtered control voltage (external mode) */

    /* --- Telemetry (written by PID thread, read by TCP thread) ---  */
    volatile double  telem_time_s;         /* CLOCK_MONOTONIC secs since start */
    volatile float   telem_input_v;
    volatile float   telem_output_v;       /* target (pre-clamp)              */
    volatile float   telem_actual_output_v; /* actual (post-clamp, on the DAC) */
    volatile int     telem_holding;        /* 1 while the PID output is held   */
    volatile int     telem_gate;           /* raw gate state (1 = asserted)    */
    volatile int     telem_level;          /* gate source level: 1 HIGH, 0 LOW, -1 none */
    volatile float   telem_input_raw;      /* unfiltered ADC sample of this iteration */
    volatile int     telem_hold_count;     /* hold events since start          */
    volatile float   telem_period_us;      /* measured loop period (EMA, us)   */

    /* --- Step-response capture (armed by TCP, filled by the PID thread) -- */
    volatile int     step_state;           /* STEP_IDLE .. STEP_ABORTED        */
    volatile float   step_delta_v;         /* setpoint step to apply           */
    volatile int     step_ms;              /* capture length after the step    */
    volatile int     step_pre_ms;          /* pre-trigger length               */
    volatile int     step_fd;              /* client that requested it         */
    volatile int     step_count;           /* samples captured so far          */
    volatile int     step_pre_n;           /* sample index at which the step applied */
    volatile int     step_period_us;       /* loop period during the capture   */
    volatile float   step_sp_before;       /* setpoint before the step         */
    float            step_t_us[STEP_MAX];  /* time since the step (us)         */
    float            step_in[STEP_MAX];    /* raw input                        */
    float            step_out[STEP_MAX];   /* DAC drive                        */

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
