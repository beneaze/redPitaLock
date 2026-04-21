#ifndef FPGA_PI_H
#define FPGA_PI_H

/*
 * Direct mmap interface to the Red Pitaya v0.94 FPGA PID block, driven as
 * PI (Kd register held at zero).
 *
 * The stock bitstream provides 4 PID modules in a 2x2 MIMO matrix:
 *   PIDxy: ADC channel x -> DAC channel y
 *
 * For our diagonal use (each input drives only its own output) we use
 *   PID11 for ch0 and PID22 for ch1, holding PID12 / PID21 at zero.
 *
 * The loop runs every ADC sample at 125 MHz (~8 ns latency).  All gains and
 * the setpoint are 14-bit fixed-point registers; this layer hides the
 * fixed-point conversion behind the same V / V/V / 1/s units the rest of
 * the firmware uses.
 *
 * Channel indices: 0 = IN1/OUT1, 1 = IN2/OUT2.
 */

/* Open /dev/mem and mmap the PI register region.  Returns 0 on success.
 * Also zeroes the cross-coupling PIDs (PID12 / PID21) so the matrix is
 * effectively diagonal.                                                     */
int  fpga_pi_open(void);

/* Munmap and close.                                                         */
void fpga_pi_close(void);

/* Push setpoint and gains for a channel into the FPGA registers.
 *
 *   setpoint_v : photodiode setpoint in volts (clamped to +/- 1 V)
 *   kp         : proportional gain in V/V (effective range ~ +/- 2.0)
 *   ki         : integral gain in 1/s   (min representable ~ 477 Hz,
 *                                        max ~ 3.9e6 Hz)
 *   error_sign : +1 or -1; the sign is folded into Kp/Ki because the
 *                stock block has no explicit polarity register.
 *
 * The block's Kd register is held at zero (PI mode) -- its representable
 * range (max ~ 64 ns) is too narrow to be useful for this plant.
 *
 * Out-of-range values are clamped to the 14-bit register range with no
 * further warning.                                                          */
void fpga_pi_set(int ch, float setpoint_v, float kp, float ki,
                 float error_sign);

/* Disable a channel completely (Kp = Ki = setpoint = 0).
 * Used when PI is OFF and during autotune / manual / waveform output.       */
void fpga_pi_disable(int ch);

/* Pulse the per-channel integrator-reset bit.  Holds reset for a few
 * microseconds, then releases.  Used by the supervisor as software
 * anti-windup and on PI enable / disable transitions.                       */
void fpga_pi_reset_integrator(int ch);

#endif /* FPGA_PI_H */
