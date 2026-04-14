#include "psd.h"
#include <math.h>
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

    /* bit-reversal permutation */
    for (int i = 0; i < n; i++) {
        int j = (int)bit_reverse((unsigned)i, log2n);
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    /* Cooley-Tukey butterfly stages */
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

void psd_init(psd_state_t *p) {
    memset(p, 0, sizeof(*p));
}

void psd_push_sample(psd_state_t *p, float sample, float fs) {
    p->buf[p->fill++] = sample;
    if (p->fill < PSD_N)
        return;

    /* buffer full -- compute PSD */
    static float work_re[PSD_N];
    static float work_im[PSD_N];

    /* apply Hann window and copy into workspace */
    for (int i = 0; i < PSD_N; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(PSD_N - 1)));
        work_re[i] = p->buf[i] * w;
        work_im[i] = 0.0f;
    }

    fft_radix2(work_re, work_im, PSD_N);

    /* Hann window power for normalisation: sum(w^2)/N = 0.375 */
    float win_power = 0.375f;
    float scale = 1.0f / ((float)PSD_N * fs * win_power);

    /* DC bin (k=0) and Nyquist bin (k=N/2): no doubling */
    p->out[0]     = (work_re[0] * work_re[0] + work_im[0] * work_im[0]) * scale;
    p->out[PSD_N / 2] = (work_re[PSD_N / 2] * work_re[PSD_N / 2]
                        + work_im[PSD_N / 2] * work_im[PSD_N / 2]) * scale;

    /* bins 1 .. N/2-1: multiply by 2 for one-sided spectrum */
    for (int k = 1; k < PSD_N / 2; k++) {
        p->out[k] = 2.0f * (work_re[k] * work_re[k] + work_im[k] * work_im[k]) * scale;
    }

    p->fs    = fs;
    p->ready = 1;
    p->fill  = 0;
}
