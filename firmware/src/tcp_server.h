#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "autotune.h"
#include "psd.h"

/*
 * Per-channel runtime state shared between the supervisor thread and the
 * TCP server.  The supervisor pushes parameter changes down to the FPGA
 * PI block whenever they change, and writes telemetry fields that the
 * TCP server reads at ~100 Hz.
 *
 * The control loop itself runs in the FPGA at 125 MHz; the C side is just
 * a parameter pusher with autotune / PSD / telemetry on the side.
 */
typedef struct {
    /* --- PI parameters (written by TCP thread, read by supervisor thread) */
    volatile int     enabled;         /* 0 = PI off (default), 1 = on        */
    volatile float   setpoint_v;      /* photodiode target voltage (V)       */
    volatile float   error_sign;      /* +1 / -1: feedback polarity          */
    volatile float   kp;              /* V / V                               */
    volatile float   ki;              /* 1 / s                               */

    /* --- Manual output / waveform (active when PI is off) --------------- */
    volatile int     out_mode;        /* OUT_MODE_DC / TRIANGLE / SINE       */
    volatile float   manual_v;        /* DC voltage when out_mode == DC      */
    volatile float   wave_freq_hz;    /* waveform frequency (Hz)             */
    volatile float   wave_amplitude;  /* waveform peak amplitude (V)         */
    volatile float   wave_offset;     /* waveform DC offset (V)              */

    /* --- Autotune ------------------------------------------------------- */
    autotune_state_t autotune;
    volatile float   autotune_relay_amp;    /* configurable relay amplitude  */
    volatile float   autotune_hysteresis;   /* configurable noise band       */

    /* --- Telemetry (written by supervisor, read by TCP thread) ----------
     * While the PI is engaged the FPGA owns the DAC at 125 MHz and the
     * instantaneous output isn't cheaply readable from software, so
     * telem_output_v is sent as NaN.  The PSD pane / external scope show
     * the true high-rate output.                                            */
    volatile double  telem_time_s;          /* CLOCK_MONOTONIC s since start */
    volatile float   telem_input_v;
    volatile float   telem_output_v;        /* drive_v (PI off) or NaN (on)  */

    /* --- PSD (written by PSD thread, read/cleared by TCP thread) -------- */
    volatile int     psd_seq;               /* monotonic frame counter       */
    volatile float   psd_bins[PSD_BINS];    /* one-sided PSD (V^2/Hz)        */
    volatile float   psd_fs;                /* sample rate for this frame    */
    volatile int     psd_avg;               /* Welch segments to average     */
    volatile int     psd_interval_ms;       /* ms between PSD updates        */
} channel_state_t;

/* Run the TCP server on TCP_PORT.  Blocks until tcp_server_stop() is
 * called, then drains and joins all in-flight client threads before
 * returning -- it is therefore safe for the caller to tear down shared
 * resources (FPGA mmap, librp) only after this function has returned.
 * `channels` must point to an array of NUM_CHANNELS channel_state_t.       */
void tcp_server_run(channel_state_t *channels);

/* Wake tcp_server_run() out of accept().  Async-signal-safe: only sets a
 * sig_atomic_t flag and calls shutdown(2).  Idempotent.                    */
void tcp_server_stop(void);

#endif /* TCP_SERVER_H */
