#ifndef ADS1256_H
#define ADS1256_H

/*
 * ads1256.h  -  ADS1256 SPI driver interface for sigrok-pico (PICO_MODE 3)
 *
 * All register addresses, command bytes, and timing constants from the
 * Texas Instruments ADS1256 datasheet (SBAS288).
 *
 * Only compiled when ADS1256_MODE is defined (i.e. PICO_MODE == 3 in
 * sr_device.h).  Other modes do not link ads1256.c at all.
 */
#ifdef ADS1256_MODE

#include <stdint.h>
#include <stdbool.h>
#include "sr_device.h"   /* ADS1256_* compile-time constants */
#include "hardware/spi.h"

/* -----------------------------------------------------------------------
 * ADS1256 register addresses (Table 23, ADS1256 datasheet SBAS288)
 * ----------------------------------------------------------------------- */
#define ADS1256_REG_STATUS  0x00
#define ADS1256_REG_MUX     0x01
#define ADS1256_REG_ADCON   0x02
#define ADS1256_REG_DRATE   0x03
#define ADS1256_REG_IO      0x04
#define ADS1256_REG_OFC0    0x05
#define ADS1256_REG_OFC1    0x06
#define ADS1256_REG_OFC2    0x07
#define ADS1256_REG_FSC0    0x08
#define ADS1256_REG_FSC1    0x09
#define ADS1256_REG_FSC2    0x0A

/* -----------------------------------------------------------------------
 * ADS1256 SPI command bytes (Table 24, ADS1256 datasheet SBAS288)
 * ----------------------------------------------------------------------- */
#define ADS1256_CMD_WAKEUP   0x00
#define ADS1256_CMD_RDATA    0x01
#define ADS1256_CMD_RDATAC   0x03
#define ADS1256_CMD_SDATAC   0x0F
#define ADS1256_CMD_RREG     0x10
#define ADS1256_CMD_WREG     0x50
#define ADS1256_CMD_SELFCAL  0xF0
#define ADS1256_CMD_SELFOCAL 0xF1
#define ADS1256_CMD_SELFGCAL 0xF2
#define ADS1256_CMD_SYSOCAL  0xF3
#define ADS1256_CMD_SYSGCAL  0xF4
#define ADS1256_CMD_SYNC     0xFC
#define ADS1256_CMD_STANDBY  0xFD
#define ADS1256_CMD_RESET    0xFE

/* -----------------------------------------------------------------------
 * STATUS register bit fields (Table 25, ADS1256 datasheet)
 * ----------------------------------------------------------------------- */
#define ADS1256_STATUS_BUFEN  (1 << 1)   /* input buffer enable              */
#define ADS1256_STATUS_ACAL   (1 << 2)   /* auto-calibration enable          */
#define ADS1256_STATUS_ORDER  (1 << 3)   /* bit order: 0=MSB first (default) */

/* -----------------------------------------------------------------------
 * ADCON register: PGA gain field encoding (bits [2:0], Table 27)
 *
 * ADS1256_PGA_GAIN is a compile-time constant (default 1, set in
 * sr_device.h or overridden via -DADS1256_PGA_GAIN=N on the cmake line).
 * Valid values: 1, 2, 4, 8, 16, 32, 64.
 * ----------------------------------------------------------------------- */
static inline uint8_t ads1256_pga_bits(int gain)
{
    switch (gain) {
        case 1:  return 0;
        case 2:  return 1;
        case 4:  return 2;
        case 8:  return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        default: return 0;   /* fall back to PGA=1 for any invalid value */
    }
}

/* -----------------------------------------------------------------------
 * MUX byte: single-ended channel N vs AINCOM (NSEL = 8 = AINCOM code)
 * PSEL[3:0] = ch & 0x07,  NSEL[3:0] = 0x8
 * ----------------------------------------------------------------------- */
static inline uint8_t ads1256_mux_single(uint8_t ch)
{
    return (uint8_t)(((ch & 0x07) << 4) | 0x08);
}

/* -----------------------------------------------------------------------
 * Timing constants derived from ADS1256_CLKIN_HZ
 *
 * ADS1256_BUF_ENABLE is a compile-time constant (default 1, set in
 * sr_device.h).  When enabled, AIN inputs must be held within AVDD-2V.
 * Buffer state does not change the full-scale span for external VREF.
 *
 * t6  (RDATA command to first DOUT bit)   = 50 / CLKIN  (~6.51 us @ 7.68 MHz)
 * t11 (SYNC/WAKEUP settling time)         = 24 / CLKIN  (~3.13 us @ 7.68 MHz)
 * +1 us margin; use busy_wait_us_32() on core1 to avoid SDK scheduler.
 * ----------------------------------------------------------------------- */
#define ADS1256_T6_US  ((50000000UL / ADS1256_CLKIN_HZ) + 1)
#define ADS1256_T11_US ((24000000UL / ADS1256_CLKIN_HZ) + 1)

/* -----------------------------------------------------------------------
 * Ring buffer: single-channel RDATAC mode
 *
 * core1 writes encoded samples (ADS1256_A_BYTES = 3 each), core0 drains.
 * Both indices are byte offsets that wrap at ADS1256_RING_BYTES.
 * Declared volatile so both cores see updates without cache aliasing.
 * ----------------------------------------------------------------------- */
extern volatile uint8_t  ads1256_ring[ADS1256_RING_BYTES];
extern volatile uint32_t ads1256_ring_wr;   /* next write offset, owned by core1 */
extern volatile uint32_t ads1256_ring_rd;   /* next read  offset, owned by core0 */
extern volatile bool     ads1256_ring_overflow; /* set by core1 on overflow */

/* -----------------------------------------------------------------------
 * Shared control flags (written by core0, read by core1)
 * ----------------------------------------------------------------------- */
extern volatile bool    ads1256_core1_run;  /* core0 sets true to start sampling  */
extern volatile bool    ads1256_multichan;  /* true = multi-ch; false = RDATAC    */
extern volatile uint8_t ads1256_single_ch;  /* channel index for single-ch mode   */

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/*
 * ads1256_hw_init()
 * Configure SPI0 pins and peripheral (CPOL=0, CPHA=1, MSB-first,
 * ADS1256_SPI_HZ).  Configure CS (GPIO output) and DRDY (GPIO input,
 * pull-up).  Issue RESET, write STATUS/MUX/ADCON/DRATE registers, run
 * SELFCAL, wait for DRDY.  Called once from core1 before any sampling.
 *
 * ADS1256_PGA_GAIN and ADS1256_BUF_ENABLE are used here to build the
 * ADCON and STATUS register values at compile time.
 */
void ads1256_hw_init(void);

/*
 * ads1256_encode_sample(raw24, out)
 * Encode a 24-bit two's-complement ADS1256 result into ADS1256_A_BYTES (3)
 * wire bytes for the libsigrok raspberrypi-pico protocol (a_size = 3).
 * Negative values (below AINCOM) are clamped to 0 (single-ended mode).
 *
 * Bit layout:
 *   V = (uint32_t)max(raw24,0) >> ADS1256_A_RSHIFT   (21-bit unsigned)
 *   out[0] = 0x80 | ((V >> 14) & 0x7F)   bits 20..14
 *   out[1] = 0x80 | ((V >>  7) & 0x7F)   bits 13..7
 *   out[2] = 0x80 | ( V        & 0x7F)   bits  6..0
 *
 * The scale and offset reported by the 'a' command are computed from
 * ADS1256_PGA_GAIN and ADS1256_BUF_ENABLE in sr_device.h:
 *   ADS1256_SCALE_UV = (ADS1256_VREF_UV / PGA_GAIN) / 2^20  [uV/count]
 *   ADS1256_OFFSET_UV = 0
 */
void ads1256_encode_sample(int32_t raw24, uint8_t out[ADS1256_A_BYTES]);

/*
 * ads1256_core1_entry()
 * Core1 main function, launched via multicore_launch_core1().
 * Calls ads1256_hw_init() once, then loops indefinitely:
 *   - waits for ads1256_core1_run == true
 *   - runs single-channel RDATAC (ads1256_multichan == false) or
 *     multi-channel MUX-cycling (ads1256_multichan == true)
 *   - issues SDATAC, clears ads1256_core1_run, returns to idle
 */
void ads1256_core1_entry(void);

#endif /* ADS1256_MODE */
#endif /* ADS1256_H */
