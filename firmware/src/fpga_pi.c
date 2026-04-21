#include "fpga_pi.h"
#include "config.h"

#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* ------------------------------------------- v0.94 PID register map ----
 *
 * We drive the stock RP PID block as a PI controller (Kd held at zero).
 *
 *   0x00 : config (bits[3:0] = integrator reset for PID22/21/12/11)
 *   0x10 : PID11 setpoint (signed 14-bit)
 *   0x14 : PID11 Kp       (signed 14-bit)
 *   0x18 : PID11 Ki       (signed 14-bit)
 *   0x1C : PID11 Kd       (signed 14-bit; held at 0 here)
 *   0x20 ..0x2C : PID12 (held at zero -- no cross coupling)
 *   0x30 ..0x3C : PID21 (held at zero -- no cross coupling)
 *   0x40 ..0x4C : PID22 (ch1)
 *
 * Effective gains in continuous units (PSR=12, ISR=18, fs=125 MHz,
 * 1 V = 8192 ADC LSBs = 8192 DAC LSBs):
 *
 *   Kp_eff (V/V)  = set_kp / 2^PSR        = set_kp / 4096
 *   Ki_eff (1/s)  = set_ki * fs / 2^ISR   = set_ki * 476.84
 *
 * Inverse (used here):
 *   set_kp = kp_user * 4096
 *   set_ki = ki_user / 476.84
 *
 * Kd is left at zero -- the block's DSR=10 fixed-point only spans ~64 ns of
 * derivative time, well below anything physically useful for this plant.
 * ----------------------------------------------------------------------- */

#define REG_CFG          0x00
#define PID11_BASE       0x10
#define PID12_BASE       0x20
#define PID21_BASE       0x30
#define PID22_BASE       0x40

#define OFF_SETPOINT     0x00
#define OFF_KP           0x04
#define OFF_KI           0x08
#define OFF_KD           0x0C

#define INT_RST_PID11    (1u << 0)
#define INT_RST_PID12    (1u << 1)
#define INT_RST_PID21    (1u << 2)
#define INT_RST_PID22    (1u << 3)

#define REG14_MAX        ( (1 << 13) - 1)   /*  +8191 */
#define REG14_MIN        (-(1 << 13))       /*  -8192 */

static volatile uint32_t *g_regs = NULL;
static int                g_mem_fd = -1;

/* Serialises read-modify-write on the shared REG_CFG (integrator-reset
 * bits for all 4 PIDs).  Per-channel setpoint/Kp/Ki/Kd writes are write-
 * only and don't need locking.                                              */
static pthread_mutex_t    g_cfg_lock = PTHREAD_MUTEX_INITIALIZER;

/* ------------------------------------------------------------------- helpers */

static int32_t clamp14(int32_t v) {
    if (v > REG14_MAX) return REG14_MAX;
    if (v < REG14_MIN) return REG14_MIN;
    return v;
}

static uint32_t encode14(int32_t v) {
    /* Hardware sign-extends the 14 LSBs; we hand it the 14-bit two's
     * complement value already masked into the low bits.                    */
    return ((uint32_t)v) & 0x3FFF;
}

static void wr(uint32_t off, uint32_t val) {
    g_regs[off / 4] = val;
}

static uint32_t rd(uint32_t off) {
    return g_regs[off / 4];
}

static uint32_t pid_base_for_ch(int ch) {
    return (ch == 0) ? PID11_BASE : PID22_BASE;
}

static uint32_t int_rst_bit_for_ch(int ch) {
    return (ch == 0) ? INT_RST_PID11 : INT_RST_PID22;
}

/* ----------------------------------------------------------------- public */

int fpga_pi_open(void) {
    g_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_mem_fd < 0) {
        perror("fpga_pi_open: open /dev/mem");
        return -1;
    }

    void *map = mmap(NULL, FPGA_PI_MAP_SIZE, PROT_READ | PROT_WRITE,
                     MAP_SHARED, g_mem_fd, FPGA_PI_BASE);
    if (map == MAP_FAILED) {
        perror("fpga_pi_open: mmap PI block");
        close(g_mem_fd);
        g_mem_fd = -1;
        return -1;
    }

    g_regs = (volatile uint32_t *)map;

    /* Force-zero all gains and hold every integrator in reset BEFORE doing
     * anything else, so any stale values from a previous run can't spike
     * the DAC while we probe.                                              */
    wr(REG_CFG, INT_RST_PID11 | INT_RST_PID12 | INT_RST_PID21 | INT_RST_PID22);
    for (uint32_t base = PID11_BASE; base <= PID22_BASE; base += 0x10) {
        wr(base + OFF_SETPOINT, 0);
        wr(base + OFF_KP,       0);
        wr(base + OFF_KI,       0);
        wr(base + OFF_KD,       0);
    }

    /* Sanity check: write a 14-bit pattern to PID11 setpoint and read it
     * back.  A proper v0.94 bitstream will mask off the upper bits and
     * return exactly the low 14.  If the read doesn't match (e.g. all
     * ones, all zeros, or a stale value), the PI region isn't backed by
     * the expected hardware -- almost always a missing or wrong bitstream.
     * Restore zero before returning so we don't leave a phantom setpoint. */
    const uint32_t probe = 0x1A5;
    wr(PID11_BASE + OFF_SETPOINT, probe);
    uint32_t readback = rd(PID11_BASE + OFF_SETPOINT) & 0x3FFF;
    wr(PID11_BASE + OFF_SETPOINT, 0);
    if (readback != probe) {
        fprintf(stderr,
                "fpga_pi_open: PI register loopback failed "
                "(wrote 0x%03x, read 0x%03x). Is the v0.94 bitstream loaded?\n",
                probe, readback);
        munmap(map, FPGA_PI_MAP_SIZE);
        g_regs = NULL;
        close(g_mem_fd);
        g_mem_fd = -1;
        return -1;
    }

    /* All gains are already zero from the pre-probe init above; just
     * release the integrator resets we held.  The cross-coupling PIDs
     * (PID12, PID21) stay at zero gain forever -- each ADC channel only
     * drives its own DAC.                                                  */
    wr(REG_CFG, 0);

    return 0;
}

void fpga_pi_close(void) {
    if (g_regs) {
        /* Leave the FPGA in a safe state: all gains zero, integrators
         * forced low.                                                       */
        fpga_pi_disable(0);
        fpga_pi_disable(1);
        wr(REG_CFG, INT_RST_PID11 | INT_RST_PID22);

        munmap((void *)g_regs, FPGA_PI_MAP_SIZE);
        g_regs = NULL;
    }
    if (g_mem_fd >= 0) {
        close(g_mem_fd);
        g_mem_fd = -1;
    }
}

void fpga_pi_set(int ch, float setpoint_v, float kp, float ki,
                 float error_sign) {
    if (!g_regs || (ch != 0 && ch != 1)) return;

    /* The stock block has no polarity register; absorb error_sign into the
     * gains.  Stock block computes error = setpoint - measurement, which
     * already matches our error_sign = +1 convention.                       */
    float sign = (error_sign < 0.0f) ? -1.0f : 1.0f;

    int32_t sp_lsb = (int32_t)lrintf(setpoint_v * (float)ADC_LSB_PER_VOLT);
    int32_t kp_lsb = (int32_t)lrintf(sign * kp * (float)(1 << PI_PSR));
    int32_t ki_lsb = (int32_t)lrintf(sign * ki * (float)(1 << PI_ISR)
                                     / (float)HARDWARE_LOOP_HZ);

    sp_lsb = clamp14(sp_lsb);
    kp_lsb = clamp14(kp_lsb);
    ki_lsb = clamp14(ki_lsb);

    uint32_t base = pid_base_for_ch(ch);
    wr(base + OFF_SETPOINT, encode14(sp_lsb));
    wr(base + OFF_KP,       encode14(kp_lsb));
    wr(base + OFF_KI,       encode14(ki_lsb));
    /* Kd register left at 0 (set in fpga_pi_open). */

#ifdef FPGA_PI_DEBUG_PUSH
    /* The supervisor pushes every tick now, so this is ~1000 lines/s when
     * enabled -- gate it behind a compile-time flag.                        */
    printf("[fpga_pi] ch%d push sp=%.4fV (lsb=%d) kp=%.4f (lsb=%d) "
           "ki=%.4f (lsb=%d) esign=%+0.0f\n",
           ch, (double)setpoint_v, sp_lsb,
           (double)kp, kp_lsb,
           (double)ki, ki_lsb,
           (double)error_sign);
#endif
}

void fpga_pi_disable(int ch) {
    if (!g_regs || (ch != 0 && ch != 1)) return;

    uint32_t base = pid_base_for_ch(ch);
    wr(base + OFF_SETPOINT, 0);
    wr(base + OFF_KP,       0);
    wr(base + OFF_KI,       0);
    wr(base + OFF_KD,       0);
    fpga_pi_reset_integrator(ch);
}

void fpga_pi_reset_integrator(int ch) {
    if (!g_regs || (ch != 0 && ch != 1)) return;

    uint32_t bit = int_rst_bit_for_ch(ch);

    pthread_mutex_lock(&g_cfg_lock);
    uint32_t cfg = rd(REG_CFG);
    /* Hold reset high for a few hundred ns -- AXI bus latency makes the
     * read-back below already long enough.                                  */
    wr(REG_CFG, cfg | bit);
    (void)rd(REG_CFG);              /* read-back: serialises the write       */
    wr(REG_CFG, cfg & ~bit);
    pthread_mutex_unlock(&g_cfg_lock);
}
