#ifndef PSD_H
#define PSD_H

#include "config.h"

#define PSD_BINS  (PSD_N / 2 + 1)   /* one-sided spectrum: 0 .. N/2 inclusive */

typedef struct {
    float  buf[PSD_N];              /* sample ring buffer                     */
    int    fill;                    /* samples collected so far (0..PSD_N)    */
    float  out[PSD_BINS];           /* latest PSD result (V^2/Hz)             */
    float  fs;                      /* sample rate used for this frame (Hz)   */
    int    ready;                   /* 1 = new PSD available, cleared by reader */
} psd_state_t;

void  psd_init(psd_state_t *p);
void  psd_push_sample(psd_state_t *p, float sample, float fs);

#endif /* PSD_H */
