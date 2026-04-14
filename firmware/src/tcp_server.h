#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "pid.h"
#include "analog_io.h"
#include "autotune.h"

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

    /* --- Telemetry (written by PID thread, read by TCP thread) ---  */
    volatile float   telem_input_v;
    volatile float   telem_output_v;       /* target (pre-clamp)              */
    volatile float   telem_actual_output_v; /* actual (post-clamp, on the DAC) */
} channel_state_t;

/* Starts the TCP server on TCP_PORT.  Runs forever (call from a thread).
 * `channels` must point to an array of NUM_CHANNELS channel_state_t.        */
void tcp_server_run(channel_state_t *channels);

#endif /* TCP_SERVER_H */
