#ifndef PSD_H
#define PSD_H

#include "config.h"

typedef struct {
    float  accum[PSD_BINS];  /* running sum of |X[k]|^2 (unnormalised)      */
    float  out[PSD_BINS];    /* latest averaged PSD result (V^2/Hz)         */
    float *work_re;          /* FFT workspace, PSD_N floats (heap)          */
    float *work_im;          /* FFT workspace, PSD_N floats (heap)          */
    float  fs;               /* sample rate for this PSD frame (Hz)         */
    int    avg_count;        /* segments accumulated so far                 */
    int    ready;            /* 1 = new averaged PSD available              */
} psd_state_t;

/* Returns 0 on success, -1 if allocation fails. */
int  psd_init(psd_state_t *p);
void psd_free(psd_state_t *p);

/*
 * Process one buffer of PSD_N time-domain samples:
 *   - apply Hann window
 *   - radix-2 FFT
 *   - accumulate |X[k]|^2 into accum[]
 *   - when avg_count reaches avg_target, normalise to V^2/Hz and set ready=1
 *
 * `buf` must contain PSD_N floats (voltages).
 * `fs`  is the effective sample rate (Hz).
 */
void psd_process_buffer(psd_state_t *p, const float *buf, float fs,
                        int avg_target);

#endif /* PSD_H */
