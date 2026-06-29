/*
 * ads1256.c  -  ADS1256 24-bit SPI ADC driver for sigrok-pico (PICO_MODE 3)
 *
 * Implements:
 *   ads1256_hw_init()       - one-time hardware and register setup
 *   ads1256_encode_sample() - encode 24-bit result into 3 x 7-bit wire bytes
 *   ads1256_core1_entry()   - core1 sampling loop dispatcher
 *
 * Compiled only when ADS1256_MODE is defined (PICO_MODE == 3 in sr_device.h).
 *
 * Design references:
 *   ADS1256 datasheet SBAS288:
 *     Section 8.5.3  - Serial Interface
 *     Section 8.5.4  - Continuous Read Mode (RDATAC)
 *     Figure 29      - Initialization sequence
 *     Figure 33      - Pipelined channel multiplexing
 *     Table 13       - DRATE register values
 *     Table 23       - Register map
 *     Table 24       - SPI command bytes
 *     Table 25       - STATUS register
 *     Table 27       - ADCON register (PGA bits)
 *
 *   libsigrok src/hardware/raspberrypi-pico/protocol.c:
 *     process_slice() multi-byte analog decoding (a_size = 3)
 */

// Guard is defined in the header includes.
#include "ads1256.h"

#ifdef ADS1256_MODE

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Shared state definitions (declared extern in ads1256.h)
 * ----------------------------------------------------------------------- */
volatile uint8_t ads1256_ring[ADS1256_RING_BYTES];
volatile uint32_t ads1256_ring_wr = 0;
volatile uint32_t ads1256_ring_rd = 0;
volatile bool ads1256_ring_overflow = false;

volatile bool ads1256_core1_run = false;
volatile bool ads1256_multichan = false;
volatile uint8_t ads1256_single_ch = 0;

/* External references from pico_sdk_sigrok.c used in multi-channel mode */
extern sr_device_t dev;
extern volatile uint32_t dma_halves;

/* -----------------------------------------------------------------------
 * Low-level SPI / GPIO helpers
 * CS is GPIO-controlled (not hardware NSS) for precise timing.
 * ----------------------------------------------------------------------- */

static inline void cs_assert(void) { gpio_put(ADS1256_PIN_CS, 0); }
static inline void cs_deassert(void) { gpio_put(ADS1256_PIN_CS, 1); }

/*
 * wait_drdy(timeout_us)
 * Busy-polls DRDY until it goes low (active-low, conversion complete).
 * Returns true if DRDY asserted within timeout_us, false on timeout.
 */
static bool wait_drdy(uint32_t timeout_us)
{
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (gpio_get(ADS1256_PIN_DRDY) != 0)
    {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
            return false;
    }
    return true;
}

static inline void spi_write_byte(uint8_t b)
{
    spi_write_blocking(spi0, &b, 1);
}

static inline void spi_read_bytes(uint8_t *buf, size_t count)
{
    spi_read_blocking(spi0, 0xFF, buf, count);
}

/* -----------------------------------------------------------------------
 * ads1256_hw_init()
 *
 * Sequence (ADS1256 datasheet Figure 29 / section 8.5.3):
 *   1. Init SPI0: CPOL=0, CPHA=1, 8-bit, MSB-first at ADS1256_SPI_HZ.
 *   2. Configure MISO/SCLK/MOSI as SPI function; CS and DRDY as GPIO.
 *   3. Assert CS, send RESET, deassert CS, wait 10 ms.
 *   4. Assert CS, WREG starting at STATUS (4 registers in one burst):
 *        STATUS = ACAL | (BUFEN if ADS1256_BUF_ENABLE)
 *        MUX    = AIN0 vs AINCOM (default; will be updated before sampling)
 *        ADCON  = PGA encoding for ADS1256_PGA_GAIN (no CLK out, no SDCS)
 *        DRATE  = ADS1256_DRATE_REG
 *   5. Send SELFCAL (offset + gain self-calibration).
 *   6. Wait DRDY low (calibration complete, timeout 1 s).
 * ----------------------------------------------------------------------- */
void ads1256_hw_init(void)
{
    /* 1. SPI0: Mode 1 (CPOL=0, CPHA=1), 8-bit, MSB first */
    spi_init(spi0, ADS1256_SPI_HZ);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    /* 2a. SPI GPIO functions */
    gpio_set_function(ADS1256_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(ADS1256_PIN_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(ADS1256_PIN_MOSI, GPIO_FUNC_SPI);

    /* 2b. CS: output, deasserted (high) */
    gpio_init(ADS1256_PIN_CS);
    gpio_set_dir(ADS1256_PIN_CS, GPIO_OUT);
    gpio_put(ADS1256_PIN_CS, 1);

    /* 2c. DRDY: input, pull-up (active-low) */
    gpio_init(ADS1256_PIN_DRDY);
    gpio_set_dir(ADS1256_PIN_DRDY, GPIO_IN);
    gpio_pull_up(ADS1256_PIN_DRDY);

    /* 3. Hardware reset */
    cs_assert();
    spi_write_byte(ADS1256_CMD_RESET);
    cs_deassert();
    sleep_ms(10); /* conservatively >= t_STBY ~0.6 ms */

    /*
     * 4. Write STATUS, MUX, ADCON, DRATE in a single WREG burst.
     *    Command: 0x50 | start_reg,  count-1,  data[0..3]
     *
     *    STATUS value:
     *      bit 3 (ORDER) = 0   -> MSB first (default, datasheet Table 25)
     *      bit 2 (ACAL)  = 1   -> auto-calibration after WREG
     *      bit 1 (BUFEN) = ADS1256_BUF_ENABLE (compile-time, default 1)
     *      bit 0 (DRDY)  = read-only, ignored on write
     *
     *    ADCON value:
     *      bits [6:5] (CLK)  = 00  -> CLKOUT off
     *      bits [4:3] (SDCS) = 00  -> sensor detect off
     *      bits [2:0] (PGA)  = encoding of ADS1256_PGA_GAIN
     */
    uint8_t status_val = ADS1256_STATUS_ACAL |
                         (ADS1256_BUF_ENABLE ? ADS1256_STATUS_BUFEN : 0);
    uint8_t adcon_val = ads1256_pga_bits(ADS1256_PGA_GAIN);
    uint8_t mux_val = ads1256_mux_single(0); /* AIN0 vs AINCOM */

    cs_assert();
    spi_write_byte(ADS1256_CMD_WREG | ADS1256_REG_STATUS);
    spi_write_byte(3);                 /* write 4 registers: count - 1 = 3 */
    spi_write_byte(status_val);        /* STATUS */
    spi_write_byte(mux_val);           /* MUX    */
    spi_write_byte(adcon_val);         /* ADCON  */
    spi_write_byte(ADS1256_DRATE_REG); /* DRATE */
    cs_deassert();

    /*
     * 5. Self-calibration (offset + gain).
     *    Required after PGA or DRATE change (datasheet section 8.4.5).
     *    ACAL in STATUS will also trigger auto-cal after the WREG above;
     *    issuing SELFCAL explicitly ensures calibration completes before
     *    we start sampling.
     */
    cs_assert();
    spi_write_byte(ADS1256_CMD_SELFCAL);
    cs_deassert();

    /* 6. Wait for DRDY low: calibration done. Timeout 1 s. */
    wait_drdy(1000000);
}

/* -----------------------------------------------------------------------
 * ads1256_encode_sample(raw24, out[3])
 *
 * Encodes a 24-bit two's-complement ADS1256 result into 3 wire bytes.
 *
 * Single-ended only (AINCOM = AGND): negative raw values indicate noise
 * below ground reference; clamp to 0 before encoding.
 *
 * Encoding:
 *   V = (uint32_t) max(raw24, 0) >> ADS1256_A_RSHIFT   [21-bit unsigned]
 *   out[0] = 0x80 | ((V >> 14) & 0x7F)   bits 20..14
 *   out[1] = 0x80 | ((V >>  7) & 0x7F)   bits 13..7
 *   out[2] = 0x80 | ( V        & 0x7F)   bits  6..0
 *
 * The 0x80 framing bit is required by the libsigrok raspberrypi-pico
 * protocol: process_slice() reads (buffer[rdptr] - 0x80) for each byte,
 * so all analog bytes must be >= 0x80.
 *
 * Scale reported by the 'a' command (sr_device.c):
 *   ADS1256_SCALE_UV  = (ADS1256_VREF_UV / ADS1256_PGA_GAIN) / 2^20
 *   ADS1256_OFFSET_UV = 0
 * Both are integer microvolts. ADS1256_BUF_ENABLE does not change the
 * full-scale span for an external VREF; buffer state only affects
 * valid input range (AIN <= AVDD-2V when buffer enabled).
 * ----------------------------------------------------------------------- */
void ads1256_encode_sample(int32_t raw24, uint8_t out[ADS1256_A_BYTES])
{
    if (raw24 < 0)
        raw24 = 0;
    uint32_t v = (uint32_t)raw24 >> ADS1256_A_RSHIFT;
    out[0] = 0x80 | (v & 0x7F);
    out[1] = 0x80 | ((v >> 7) & 0x7F);
    out[2] = 0x80 | ((v >> 14) & 0x7F);
}

/* -----------------------------------------------------------------------
 * Internal: read 24-bit result via RDATA command (single-channel mode)
 * CS must be asserted by caller.  Sends RDATA, waits t6, reads 3 bytes.
 * Sign-extends the 24-bit result to int32_t.
 * ----------------------------------------------------------------------- */
static int32_t ads1256_read24_rdata(void)
{
    uint8_t buf[3];
    spi_write_byte(ADS1256_CMD_RDATA);
    busy_wait_us_32(ADS1256_T6_US);
    spi_read_bytes(buf, 3);
    int32_t raw = ((int32_t)buf[0] << 16) |
                  ((int32_t)buf[1] << 8) |
                  (int32_t)buf[2];
    if (raw & 0x800000)
        raw |= (int32_t)0xFF000000;
    return raw;
}

/* Internal: read 24-bit result in RDATAC mode.
 * CS is asserted; no command byte needed - just clock 3 bytes.
 * Called immediately after DRDY asserts. */
static int32_t ads1256_read24_rdatac(void)
{
    uint8_t buf[3];
    spi_read_bytes(buf, 3);
    int32_t raw = ((int32_t)buf[0] << 16) |
                  ((int32_t)buf[1] << 8) |
                  (int32_t)buf[2];
    if (raw & 0x800000)
        raw |= (int32_t)0xFF000000;
    return raw;
}

/* -----------------------------------------------------------------------
 * run_single_channel()
 *
 * Single-channel continuous read mode (RDATAC).
 * Runs on core1; core0 drains ads1256_ring via send logic.
 *
 * Flow:
 *   1. Write MUX for ads1256_single_ch
 *   2. SYNC + WAKEUP to settle (t11 between each)
 *   3. Wait DRDY, enter RDATAC
 *   4. Loop: wait DRDY, read 24 bits, encode, write to ring buffer
 *   5. On exit: deassert CS, send SDATAC
 *
 * Ring buffer overflow: if the write pointer would lap the read pointer,
 * set ads1256_ring_overflow = true and exit.
 * ----------------------------------------------------------------------- */
static void run_single_channel(void)
{
    /* Program MUX, SYNC, WAKEUP (datasheet section 8.5.3 Figure 30) */
    cs_assert();
    spi_write_byte(ADS1256_CMD_WREG | ADS1256_REG_MUX);
    spi_write_byte(0); /* count - 1 = 0 (one register) */
    spi_write_byte(ads1256_mux_single(ads1256_single_ch));
    cs_deassert();

    busy_wait_us_32(ADS1256_T11_US);
    cs_assert();
    spi_write_byte(ADS1256_CMD_SYNC);
    cs_deassert();

    busy_wait_us_32(ADS1256_T11_US);
    cs_assert();
    spi_write_byte(ADS1256_CMD_WAKEUP);
    cs_deassert();

    /* Wait for first DRDY before entering RDATAC */
    wait_drdy(200000);

    /* Enter RDATAC - CS stays asserted through the sampling loop */
    cs_assert();
    spi_write_byte(ADS1256_CMD_RDATAC);

    while (ads1256_core1_run)
    {
        /* Wait for DRDY (next conversion complete) */
        while (gpio_get(ADS1256_PIN_DRDY) != 0)
        {
            if (!ads1256_core1_run)
                goto done_rdatac;
        }

        /* Read 24-bit result (no RDATA command needed in RDATAC) */
        int32_t raw = ads1256_read24_rdatac();

        /* Compute next write position */
        uint32_t next_wr = ads1256_ring_wr + ADS1256_A_BYTES;
        if (next_wr >= ADS1256_RING_BYTES)
            next_wr = 0;

        /* Overflow check: would we lap the read pointer? */
        if (next_wr == ads1256_ring_rd)
        {
            ads1256_ring_overflow = true;
            // The wrapper of this call will restart on overflow conditions, so we can quit "safely".
            // ALthough this goto is gnarly, and gonna make for leaks.
            goto done_rdatac;
        }

        /* Encode and write into ring buffer */
        uint8_t enc[ADS1256_A_BYTES];
        ads1256_encode_sample(raw, enc);
        uint32_t wr = ads1256_ring_wr;
        ads1256_ring[wr] = enc[0];
        ads1256_ring[wr + 1] = enc[1];
        ads1256_ring[wr + 2] = enc[2];

        /* Commit: advance write pointer after data is written */
        ads1256_ring_wr = next_wr;
    }

done_rdatac:
    cs_deassert();

    /* Exit RDATAC mode (datasheet section 8.5.4) */
    busy_wait_us_32(ADS1256_T6_US);
    cs_assert();
    spi_write_byte(ADS1256_CMD_SDATAC);
    cs_deassert();
}

/* -----------------------------------------------------------------------
 * run_multi_channel()
 *
 * Pipelined multi-channel MUX cycling (ADS1256 datasheet Figure 33).
 * Digital channels are NOT sampled in this mode.
 *
 * While channel[cur] conversion is in progress, we program MUX for
 * channel[next] and issue SYNC+WAKEUP to start its conversion.  When
 * DRDY asserts, channel[cur]'s result is ready; we read it with RDATA.
 *
 * Encoded bytes are written into dev.abuf0_start / dev.abuf1_start.
 * dma_halves is incremented by core1 when a half-buffer fills, which
 * triggers the existing send_half logic on core0.
 * ----------------------------------------------------------------------- */
static void run_multi_channel(void)
{
    /* Build list of enabled channels from dev.a_mask */
    uint8_t chans[NUM_A_CHAN];
    uint8_t nchan = 0;
    for (int i = 0; i < NUM_A_CHAN; i++)
    {
        if ((dev.a_mask >> i) & 1)
            chans[nchan++] = (uint8_t)i;
    }
    if (nchan == 0)
        return;

    // uint8_t *buf_ptrs[2] = {
    //     (uint8_t *)(uintptr_t)dev.abuf0_start,
    //     (uint8_t *)(uintptr_t)dev.abuf1_start};
    extern uint8_t *capture_buf;
    uint8_t *buf_ptrs[2] = {
        capture_buf + dev.abuf0_start,
        capture_buf + dev.abuf1_start};
    uint32_t half_bytes = dev.a_size; /* set by pico_sdk_sigrok.c */
    uint32_t half_idx = 0;
    uint32_t byte_off = 0;

    uint32_t total_enc = dev.num_samples *
                         (uint32_t)nchan * ADS1256_A_BYTES;
    uint32_t enc_done = 0;

    /*
     * Prime: program MUX for chans[0], issue SYNC+WAKEUP, wait DRDY.
     * This starts the first conversion before we enter the main loop.
     */
    cs_assert();
    spi_write_byte(ADS1256_CMD_WREG | ADS1256_REG_MUX);
    spi_write_byte(0);
    spi_write_byte(ads1256_mux_single(chans[0]));
    cs_deassert();

    busy_wait_us_32(ADS1256_T11_US);
    cs_assert();
    spi_write_byte(ADS1256_CMD_SYNC);
    cs_deassert();
    busy_wait_us_32(ADS1256_T11_US);
    cs_assert();
    spi_write_byte(ADS1256_CMD_WAKEUP);
    cs_deassert();

    if (!wait_drdy(200000))
        return;

    uint8_t cur = 0; /* index into chans[] for the result being read */

    while (ads1256_core1_run)
    {
        uint8_t next = (uint8_t)((cur + 1) % nchan);

        /*
         * While channel[cur] data is being held:
         * Program MUX[next] and start its conversion.
         * SYNC + WAKEUP sequence per datasheet Figure 33.
         */
        cs_assert();
        spi_write_byte(ADS1256_CMD_WREG | ADS1256_REG_MUX);
        spi_write_byte(0);
        spi_write_byte(ads1256_mux_single(chans[next]));
        cs_deassert();

        busy_wait_us_32(ADS1256_T11_US);
        cs_assert();
        spi_write_byte(ADS1256_CMD_SYNC);
        cs_deassert();
        busy_wait_us_32(ADS1256_T11_US);
        cs_assert();
        spi_write_byte(ADS1256_CMD_WAKEUP);
        cs_deassert();

        /* Wait for DRDY: channel[cur] conversion complete */
        if (!wait_drdy(500000))
            break;

        /* Read channel[cur] result via RDATA */
        cs_assert();
        int32_t raw = ads1256_read24_rdata();
        cs_deassert();

        /* Encode into current half-buffer */
        uint8_t enc[ADS1256_A_BYTES];
        ads1256_encode_sample(raw, enc);

        uint8_t *dst = buf_ptrs[half_idx] + byte_off;
        dst[0] = enc[0];
        dst[1] = enc[1];
        dst[2] = enc[2];
        byte_off += ADS1256_A_BYTES;
        enc_done += ADS1256_A_BYTES;

        /* Half-buffer full: signal core0 and switch halves */
        if (byte_off >= half_bytes)
        {
            dma_halves++;
            half_idx ^= 1;
            byte_off = 0;
        }

        /* Fixed-length run: exit when all expected encoded bytes written */
        if (!dev.cont && enc_done >= total_enc)
            break;

        cur = next;
    }
}

/* -----------------------------------------------------------------------
 * ads1256_core1_entry()
 *
 * Core1 main function.  Launched from pico_sdk_sigrok.c via
 * multicore_launch_core1(ads1256_core1_entry).
 *
 * Calls ads1256_hw_init() once after core1 starts, then loops:
 *   - idles (tight loop) until core0 sets ads1256_core1_run = true
 *   - dispatches to run_single_channel() or run_multi_channel()
 *   - clears ads1256_core1_run after sampling ends
 * ----------------------------------------------------------------------- */
void ads1256_core1_entry(void)
{
    ads1256_hw_init();

    // for (;;)
    while (!ads1256_ring_overflow)
    {
        /* Idle until core0 signals start of acquisition */
        while (!ads1256_core1_run)
            tight_loop_contents();

        /* Reset ring buffer for each new run */
        ads1256_ring_wr = 0;
        ads1256_ring_rd = 0;
        ads1256_ring_overflow = false;

        if (ads1256_multichan)
            run_multi_channel();
        else
            run_single_channel();

        If we get here because we overflowed the ring, just restart.
    }

    /* Signal core0 that sampling has finished */
    ads1256_core1_run = false;
}

#endif /* ADS1256_MODE */
