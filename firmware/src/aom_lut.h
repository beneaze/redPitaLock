#ifndef AOM_LUT_H
#define AOM_LUT_H

/*
 * AOM output linearization.
 *
 * The AOM response (optical power vs drive voltage) is highly nonlinear:
 * steep near the null (~0.18 V) and flat above ~0.5 V.  To give the PID
 * a uniform plant gain, it outputs a normalised power level (0.0 - 1.0)
 * which this function converts to the physical drive voltage via piecewise-
 * linear interpolation of the measured response curve.
 */
float aom_linearize(float normalized_power);

#endif /* AOM_LUT_H */
