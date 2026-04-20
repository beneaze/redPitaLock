#ifndef ANALOG_IO_H
#define ANALOG_IO_H

#include <stdint.h>

/*
 * Red Pitaya fast analog I/O abstraction.
 *
 * Uses librp to configure the FPGA acquisition (continuous mode) and
 * signal generator (DC mode) so the PID loop can simply call
 * analog_read() / analog_write() each iteration.
 *
 * Channel indices: 0 = IN1/OUT1, 1 = IN2/OUT2.
 */

/* Calibration applied in software to bridge the Red Pitaya +/-1 V hardware
 * range and the physical voltage domain of the experiment.                   */
typedef struct {
    float input_scale;
    float input_offset;
    float output_scale;
    float output_offset;
} analog_cal_t;

/* Initialise librp, configure acquisition + generation for both channels.
 * Returns 0 on success, -1 on error.                                        */
int  analog_io_init(void);

/* Clean shutdown of librp.                                                   */
void analog_io_cleanup(void);

/* Read the latest sample from fast input IN<ch+1>.
 * Returns the raw Red Pitaya voltage (-1 .. +1 V).                          */
float analog_read_raw(int ch);

/* Write a DC level to fast output OUT<ch+1>.
 * `voltage` is clamped to the hardware range (-1 .. +1 V).
 * Returns the actual clamped voltage that was written.                       */
float analog_write_raw(int ch, float voltage);

/* Bulk-read the most recent `n` samples from the FPGA acquisition buffer.
 * Samples are at the full ADC rate (125 MS/s with RP_DEC_1).
 * Returns the number of samples actually read.                               */
uint32_t analog_read_bulk(int ch, float *buf, uint32_t n);

/* Convenience wrappers that apply calibration.
 * analog_write returns the actual physical voltage after clamping.           */
float analog_read(int ch, const analog_cal_t *cal);
float analog_write(int ch, float physical_v, const analog_cal_t *cal);

#endif /* ANALOG_IO_H */
