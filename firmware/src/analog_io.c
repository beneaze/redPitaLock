#include "analog_io.h"
#include "config.h"
#include <rp.h>
#include <stdio.h>

static const rp_channel_t rp_ch[] = { RP_CH_1, RP_CH_2 };

/* ------------------------------------------------------------------ init */

int analog_io_init(void) {
    if (rp_Init() != RP_OK) {
        fprintf(stderr, "rp_Init failed\n");
        return -1;
    }

    /* --- Fast acquisition: continuous, no trigger, no decimation --- */
    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_1);
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_DISABLED);
    rp_AcqStart();

    /* --- Fast generation: parked in DC mode at 0 V on both channels.
     * The output stays enabled for the lifetime of the daemon; the
     * supervisor reconfigures waveform / amplitude / offset on demand.    */
    rp_GenReset();
    for (int i = 0; i < NUM_CHANNELS; i++) {
        rp_GenWaveform(rp_ch[i], RP_WAVEFORM_DC);
        rp_GenAmp(rp_ch[i], 0.0f);
        rp_GenOffset(rp_ch[i], 0.0f);
        rp_GenOutEnable(rp_ch[i]);
    }

    return 0;
}

/* --------------------------------------------------------------- cleanup */

void analog_io_cleanup(void) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        rp_GenWaveform(rp_ch[i], RP_WAVEFORM_DC);
        rp_GenAmp(rp_ch[i], 0.0f);
        rp_GenOffset(rp_ch[i], 0.0f);
        rp_GenOutDisable(rp_ch[i]);
    }
    rp_AcqStop();
    rp_Release();
}

/* ------------------------------------------------------------ raw read */

float analog_read_raw(int ch) {
    float buf[1];
    uint32_t size = 1;
    rp_AcqGetLatestDataV(rp_ch[ch], &size, buf);
    return buf[0];
}

/* --------------------------------------------------------- bulk read */

uint32_t analog_read_bulk(int ch, float *buf, uint32_t n) {
    uint32_t size = n;
    rp_AcqGetLatestDataV(rp_ch[ch], &size, buf);
    return size;
}

/* ----------------------------------------------------------- raw write */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float analog_set_dc(int ch, float voltage) {
    voltage = clampf(voltage, RP_OUTPUT_MIN, RP_OUTPUT_MAX);
    rp_GenWaveform(rp_ch[ch], RP_WAVEFORM_DC);
    rp_GenAmp(rp_ch[ch], 0.0f);
    rp_GenOffset(rp_ch[ch], voltage);
    return voltage;
}

void analog_set_waveform(int ch, int mode, float freq_hz,
                         float amplitude_v, float offset_v) {
    if (mode == OUT_MODE_DC) {
        analog_set_dc(ch, offset_v);
        return;
    }

    if (freq_hz < 0.001f)        freq_hz = 0.001f;
    if (freq_hz > 60.0e6f)       freq_hz = 60.0e6f;
    amplitude_v = clampf(amplitude_v, 0.0f, RP_OUTPUT_MAX);
    offset_v    = clampf(offset_v,    RP_OUTPUT_MIN, RP_OUTPUT_MAX);
    /* Keep amp + |offset| inside the +/-1 V hardware envelope so the FPGA
     * doesn't silently clip the waveform tips.                              */
    float headroom = RP_OUTPUT_MAX - (offset_v >= 0.0f ? offset_v : -offset_v);
    if (amplitude_v > headroom) amplitude_v = headroom;

    rp_waveform_t wf = (mode == OUT_MODE_TRIANGLE)
                       ? RP_WAVEFORM_TRIANGLE
                       : RP_WAVEFORM_SINE;
    rp_GenWaveform(rp_ch[ch], wf);
    rp_GenFreq    (rp_ch[ch], freq_hz);
    rp_GenAmp     (rp_ch[ch], amplitude_v);
    rp_GenOffset  (rp_ch[ch], offset_v);
}
