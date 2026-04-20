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

    /* --- Fast generation: DC mode on both channels --- */
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

float analog_write_raw(int ch, float voltage) {
    voltage = clampf(voltage, RP_OUTPUT_MIN, RP_OUTPUT_MAX);
    rp_GenOffset(rp_ch[ch], voltage);
    return voltage;
}

/* ------------------------------------------------ calibrated wrappers */

float analog_read(int ch, const analog_cal_t *cal) {
    (void)cal;
    return analog_read_raw(ch);
}

float analog_write(int ch, float voltage, const analog_cal_t *cal) {
    (void)cal;
    return analog_write_raw(ch, voltage);
}
