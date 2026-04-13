#include "aom_lut.h"

/*
 * Inverse response table: normalised linear power -> drive voltage.
 *
 * Derived from measured dBm-vs-V data, converted to linear power
 * (10^(dBm/10)), then normalised so 0.0 = null at 0.18 V drive and
 * 1.0 = maximum useful output at 1.25 V drive.
 */
static const float lut_norm[] = {
    0.000f, 0.005f, 0.124f, 0.291f, 0.441f, 0.569f,
    0.684f, 0.767f, 0.841f, 0.885f, 0.937f, 0.988f, 1.000f
};

static const float lut_drive[] = {
    0.18f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f,
    0.70f, 0.80f, 0.90f, 1.00f, 1.10f, 1.20f, 1.25f
};

#define LUT_LEN 13

float aom_linearize(float normalized_power) {
    float x = normalized_power;
    if (x <= 0.0f) return lut_drive[0];
    if (x >= 1.0f) return lut_drive[LUT_LEN - 1];

    for (int i = 0; i < LUT_LEN - 1; i++) {
        if (x <= lut_norm[i + 1]) {
            float t = (x - lut_norm[i]) / (lut_norm[i + 1] - lut_norm[i]);
            return lut_drive[i] + t * (lut_drive[i + 1] - lut_drive[i]);
        }
    }
    return lut_drive[LUT_LEN - 1];
}
