#include "psd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------- bit-reversal */

static unsigned bit_reverse(unsigned x, int log2n) {
    unsigned r = 0;
    for (int i = 0; i < log2n; i++) {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    return r;
}

/* ------------------------------------------ in-place radix-2 DIT FFT */

static void fft_radix2(float *re, float *im, int n) {
    int log2n = 0;
    for (int tmp = n; tmp > 1; tmp >>= 1) log2n++;

    for (int i = 0; i < n; i++) {
        int j = (int)bit_reverse((unsigned)i, log2n);
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    for (int size = 2; size <= n; size <<= 1) {
        int half = size >> 1;
        float angle = -2.0f * (float)M_PI / (float)size;
        float wr = cosf(angle);
        float wi = sinf(angle);
        for (int k = 0; k < n; k += size) {
            float tr = 1.0f, ti = 0.0f;
            for (int j = 0; j < half; j++) {
                int u = k + j;
                int v = u + half;
                float xr = tr * re[v] - ti * im[v];
                float xi = tr * im[v] + ti * re[v];
                re[v] = re[u] - xr;
                im[v] = im[u] - xi;
                re[u] += xr;
                im[u] += xi;
                float tnr = tr * wr - ti * wi;
                ti = tr * wi + ti * wr;
                tr = tnr;
            }
        }
    }
}

/* ------------------------------------------------------------ public */

int psd_init(psd_state_t *p) {
    memset(p, 0, sizeof(*p));
    p->work_re = calloc(PSD_N, sizeof(float));
    p->work_im = calloc(PSD_N, sizeof(float));
    if (!p->work_re || !p->work_im) {
        free(p->work_re);
        free(p->work_im);
        p->work_re = p->work_im = NULL;
        return -1;
    }
    return 0;
}

void psd_free(psd_state_t *p) {
    free(p->work_re);
    free(p->work_im);
    p->work_re = p->work_im = NULL;
}

void psd_process_buffer(psd_state_t *p, const float *buf, float fs,
                        int avg_target) {
    float *re = p->work_re;
    float *im = p->work_im;

    for (int i = 0; i < PSD_N; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i
                                       / (float)(PSD_N - 1)));
        re[i] = buf[i] * w;
        im[i] = 0.0f;
    }

    fft_radix2(re, im, PSD_N);

    for (int k = 0; k < PSD_BINS; k++) {
        float mag2 = re[k] * re[k] + im[k] * im[k];
        p->accum[k] += mag2;
    }

    p->avg_count++;
    p->fs = fs;

    if (p->avg_count >= avg_target) {
        /* Hann window power: sum(w^2)/N = 0.375 */
        float win_power = 0.375f;
        float scale = 1.0f / ((float)PSD_N * fs * win_power
                               * (float)p->avg_count);

        p->out[0] = p->accum[0] * scale;
        p->out[PSD_N / 2] = p->accum[PSD_N / 2] * scale;

        for (int k = 1; k < PSD_N / 2; k++)
            p->out[k] = 2.0f * p->accum[k] * scale;

        p->ready = 1;

        memset(p->accum, 0, sizeof(p->accum));
        p->avg_count = 0;
    }
}
