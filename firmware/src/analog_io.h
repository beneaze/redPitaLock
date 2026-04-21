#ifndef ANALOG_IO_H
#define ANALOG_IO_H

#include <stdint.h>

/*
 * Red Pitaya fast analog I/O abstraction.
 *
 * Uses librp to drive the FPGA acquisition (continuous mode) and signal
 * generator.  The PI loop runs in the FPGA -- this layer is only used for:
 *
 *   - sample-rate bulk reads for PSD                    (analog_read_bulk)
 *   - 1 kHz supervisor-rate single sample for telemetry (analog_read_raw)
 *   - the manual / waveform / autotune output path      (analog_set_dc /
 *     analog_set_waveform)
 *
 * The signal generator output is *summed* with the FPGA PI output at the
 * DAC, so when the PI is engaged the supervisor parks the generator at
 * DC = 0 V via analog_set_dc(ch, 0.0f).
 *
 * Channel indices: 0 = IN1/OUT1, 1 = IN2/OUT2.
 */

/* Initialise librp, configure acquisition + generation for both channels.
 * Returns 0 on success, -1 on error.                                        */
int  analog_io_init(void);

/* Clean shutdown of librp.                                                   */
void analog_io_cleanup(void);

/* Read the latest sample from fast input IN<ch+1>.
 * Returns the raw Red Pitaya voltage (-1 .. +1 V).                          */
float analog_read_raw(int ch);

/* Bulk-read the most recent `n` samples from the FPGA acquisition buffer.
 * Samples are at the full ADC rate (125 MS/s with RP_DEC_1).
 * Returns the number of samples actually read.                               */
uint32_t analog_read_bulk(int ch, float *buf, uint32_t n);

/* Park the ASG of OUT<ch+1> in DC mode at the given voltage.
 * Voltage is clamped to the hardware range (-1 .. +1 V).
 * Returns the actual clamped voltage.                                        */
float analog_set_dc(int ch, float voltage);

/* Drive OUT<ch+1> from the FPGA signal generator (sine or triangle).
 * `mode` is the OUT_MODE_* enum from config.h.  freq_hz / amplitude_v /
 * offset_v are clamped to the hardware-supported ranges.
 * For mode == OUT_MODE_DC this is equivalent to analog_set_dc(ch, offset_v).*/
void analog_set_waveform(int ch, int mode, float freq_hz,
                         float amplitude_v, float offset_v);

#endif /* ANALOG_IO_H */
