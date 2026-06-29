/*Raspberry Pi PICO/RP2040 code to implement a logic analyzer and oscilloscope
 * Some Original code from the pico examples project:
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/stdlib.h" //uart definitions
#include <stdlib.h>      //atoi,atol, malloc
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
// #include "hardware/sio.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "tusb.h" //.tud_cdc_write...

#include "sr_device.h"
#ifdef ADS1256_MODE // Defined in sr_device.h right above us, if applicable.
#include "ads1256.h"
#endif

// forced_test_mode is a special mode that puts the device into an active sampling
// state out of reset.  It is used for a quick way to debug features without needed
// pulseview or sigrok-cli to initiate a transfer.
// The enable means we will start the mode once, the run indicates it is currently running.
// To enable the feature, set forced_test_mode_en to true, but leave forced_test_mode_run false.
bool forced_test_mode_en = false; // true;
bool forced_test_mode_run = false;
PIO pio = pio0;
uint piosm = 0;
uint8_t *capture_buf;
sr_device_t dev;
volatile uint32_t tstart;
volatile bool send_resp = false;

uint8_t txbuf[TX_BUF_SIZE];
uint16_t txbufidx;
uint32_t rxbufdidx, rxbufaidx;
uint32_t rlecnt;
uint32_t bytecnt = 0; // count of characters sent serially
#ifdef PIN_TEST_MODE
struct repeating_timer pt_timer;
uint32_t pin_test_cnt = 0;
// The systick code is used to determine whether the 100us is reliably
// being generated.  If it is not then the test samples can look "stretched"
#define SYSTICK_SIZE 128
#define SYSTICK_PRINT 64
uint32_t systick_array[SYSTICK_SIZE];
uint32_t systick_idx = 0;
#endif // PIN_TEST_MODE
// Number of bytes stored as DMA per slice, must be 1,2 or 4 to support aligned access
// This will be be zero for 1-4 digital channels.
uint8_t d_dma_bps;
uint32_t samp_remain;
uint32_t lval, cval; // last and current digital sample values
uint32_t num_halves; // track the number of halves we have processed
uint32_t exp_halves; // the number of halves we expect in non-continous mode
uint32_t halves_seen = 0;
uint admachan0, admachan1, pdmachan0, pdmachan1;
// While 1 maintenance DMA could write the pio and adc DMAs, they may complete
// at slightly different times, and based on channel configurations an adc or pio
// may not be enabled so dedicate a maintence channel for each.
uint amaintchan0, amaintchan1, pmaintchan0, pmaintchan1;
dma_channel_config acfg0, acfg1, pcfg0, pcfg1, amcfg0, amcfg1, pmcfg0, pmcfg1;
// Two addresses for adc and pio dma engines
// These are read by the maintenance DMA engines and written to the
// PIO/ADC DMA engine write addrs
uint32_t *amaddrs[2] = {0, 0};
uint32_t *pmaddrs[2] = {0, 0};
uint32_t tmpaddr0, tmpaddr1; // temp variables for address generation
uint32_t *tmpptr;
uint32_t dma_halves; // number of halves observed by the dma int handler
volatile bool mask_xfer_err;
int usbintin;
uint8_t uartch;               // rx uart character -ignored as only uart tx is used
uint8_t h0intmask, h1intmask; // The required masks of ints for each half
// Keeps tracks of all interrupts we received.  Sometimes we might get an analog and not a digital
// DMA int (or vice versa) and need to exit that handler but need to clear out pending interrupt state
// so this stores what we have seen.
uint8_t currintmask; // The current mask of received ints.
uint32_t sho_cnt;    // number of times we entered the loop
uint32_t tx_cnt;     // number of times we did any kind of send
uint32_t acnt, bcnt, ccnt, dcnt, ecnt;

void print_DMA()
{
  // Print out the read addr, write addr, transaction count, and control/status
  // of the lowest 8 DMA controllers. Note that relative to RP2040, the RP2350 moved the
  // location of busy and chain_to (and maybe others)
  /*
  tmpptr=(uint32_t *)DMA_BASE;
  for(int i=0;i<8;i++){
    //note that in pointer math +1 adds 4B for a uint, so this is offset 0,4,8,c
    //and the 0x10 multiple is really 0x40
     Dprintf("RA%d 0x%X\n\r",i,*(tmpptr+0x10*i));
     Dprintf("WA%d 0x%X\n\r",i,*(tmpptr+0x10*i+1));
     Dprintf("TA%d %d\n\r",i,*(tmpptr+0x10*i+2));
     Dprintf("CA%d 0x%X\n\r",i,*(tmpptr+0x10*i+3));
*/
}

void print_DMA_chan(int chan)
{
  // Print out the read addr, write addr, transaction count, and control/status
  // of one channel
  tmpptr = (uint32_t *)DMA_BASE;
  // note that in pointer math +1 adds 4B for a uint, so this is offset 0,4,8,c
  // and the 0x10 multiple is really 0x40
  Dprintf("RA%d 0x%X\n\r", chan, *(tmpptr + 0x10 * chan));
  Dprintf("WA%d 0x%X\n\r", chan, *(tmpptr + 0x10 * chan + 1));
  Dprintf("TA%d %d\n\r", chan, *(tmpptr + 0x10 * chan + 2));
  Dprintf("CA%d 0x%X\n\r", chan, *(tmpptr + 0x10 * chan + 3));
}
// The function stdio_usb_out_chars is part of the PICO sdk usb library.
// However the function is not externally visible from the library and rather than
// figure out the build dependencies to do that it is just copied here.
// This is much faster than a printf of the string, and even faster than the puts_raw
// supported by the PICO SDK stdio library, which doesn't allow the known length of the buffer
// to be specified. (The C standard write function doesn't seem to work at all).
// This function also avoids the inserting of CR/LF in certain modes.
// The tud_cdc_write_available function returns 256, and thus we have a 256B buffer to feed into
// but the CDC serial issues in groups of 64B.
// Since there is another memory fifo inside the TUD code this might possibly be optimized
// to directly write to it, rather than writing txbuf.  That might allow faster rle processing
// but is a bit too complicated.

void my_stdio_usb_out_chars(const char *buf, int length)
{
  static uint64_t last_avail_time;
  uint32_t owner;
  // See https://github.com/pico-coder/sigrok-pico/pull/63/.
  // tud_ready does not rely on DTR
  // so use it rather than tud_cdc_connected
  //    if (tud_cdc_connected()) {
  if (tud_ready())
  {
    for (int i = 0; i < length;)
    {
      int n = length - i;
      int avail = (int)tud_cdc_write_available();
      if (n > avail)
        n = avail;
      if (n)
      {
        int n2 = (int)tud_cdc_write(buf + i, (uint32_t)n);
        tud_task();
        tud_cdc_write_flush();
        i += n2;
        last_avail_time = time_us_64();
      }
      else
      {
        tud_task();
        tud_cdc_write_flush();
        //                if (!tud_cdc_connected() || -replaced per pull request 63
        if (!tud_ready() ||
            (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US))
        {
          break;
        }
      }
    }
  }
  else
  {
    // reset our timeout
    last_avail_time = 0;
  }
}

// A common init for all send_slice modes
void send_slice_init(sr_device_t *d, uint8_t *dbuf)
{
  rxbufdidx = 0;
  rxbufaidx = 0;
  txbufidx = 0;
  rlecnt = 0;
  // Adjust the number of samples to send if there are more in the dma buffer
  samp_remain = d->samples_per_half;
  if ((d->cont == false) && ((d->scnt + samp_remain) > (d->num_samples)))
  {
    samp_remain = d->num_samples - d->scnt;
    d->scnt += samp_remain;
    // Dprintf("SSIa sph %d scnt %d lval 0x%X rem %d ns %d\n\r",d->samples_per_half,d->scnt,lval,samp_remain,d->num_samples);
  }
  else
  {
    d->scnt += d->samples_per_half;
    // Dprintf("SSIb sph %d scnt %d lval 0x%X rem %d ns %d\n\r",d->samples_per_half,d->scnt,lval,samp_remain,d->num_samples);
  }
}

// This is an optimized transmit of trace data for configurations with 4 or fewer digital channels
// and no analog.  Run length encoding (RLE) is used to send counts of repeated values to effeciently utilize
// USB CDC link bandwidth.  This is the only mode where a given serial byte can have both sample information
// and RLE counts.
// Samples from PIO are dma'd in 32 bit words, each containing 8 samples of 4 bits (1 nibble).
// RLE Encoding:
// Values 0x80-0xFF encode an rle cnt of a previous value with a new value:
//   Bit 7 is 1 to distinguish from the rle only values.
//   Bits 6:4 indicate a run length up to 7 cycles of the previous value
//   Bits 3:0 are the new value.
// For longer runs, an RLE only encoding uses decimal values 48 to 127 (0x30 to 0x7F)
// as x8 run length values of 8..640.
// All other ascii values (except from the abort and the end of run byte_cnt) are reserved.
uint32_t send_slices_D4(sr_device_t *d, uint8_t *dbuf)
{
  uint8_t nibcurr, niblast;
  uint32_t cword, lword; // current and last word
  uint32_t *cptr;
  // Note that this function always sends the first 8 samples, even if
  // send_slice_init sets remaining samples to zero.  That shouldn't happen
  // as we should also be in free running mode or split the two halves
  // into something with 8 samples.
  send_slice_init(d, dbuf);
  // Don't optimize the first word (eight samples) perfectly, just send them to make the for loop easier,
  // and setup the initial conditions for rle tracking
  cptr = (uint32_t *)&(dbuf[0]);
  cword = *cptr;
#ifdef D4_DBG
  Dprintf("Dbuf %p cptr %p data 0x%X\n\r", (void *)&(dbuf[0]), (void *)cptr, cword);
#endif
  lword = cword;
  for (int j = 0; j < 8; j++)
  {
    nibcurr = cword & 0xF;
    txbuf[j] = (nibcurr) | 0x80;
    cword >>= 4;
  }
  niblast = nibcurr;
  cptr = (uint32_t *)&(txbuf[0]);
  txbufidx += 8;
  rxbufdidx += 4;
  rlecnt = 0;
  // Note that it is generally assumed that each half buffer has far more than
  // 8 samples in it, especially if pulseview is running.  But for some useages it
  // may be only 8 so exit on the first 8. This is just mostly to prevent underflow
  // of samp_remain when we subtract 8 from it.
  if (d->samples_per_half <= 8)
  {
    my_stdio_usb_out_chars(txbuf, txbufidx);
    d->scnt += d->samples_per_half;
    return txbufidx;
  }
  // The total number of 4 bit samples remaining to process from this half.
  // Subtract 8 because we procesed the word above.
  samp_remain -= 8;

  // Process one  word (8 samples) at a time.
  for (int i = 0; i < (samp_remain >> 3); i++)
  {
    cptr = (uint32_t *)&(dbuf[rxbufdidx]);
    cword = *cptr;
    rxbufdidx += 4;
#ifdef D4_DBG2
    Dprintf("dbuf0 %p dbufr %p cptr %p \n\r", dbuf, &(dbuf[rxbufdidx]), cptr);
#endif
    // Send maximal RLE counts in this outer section to the txbuf, and if we accumulate a few of them
    // push to the device so that we don't accumulate large numbers
    // of unsent RLEs.  That allows the host to process them gradually rather than in a flood
    // when we get a value change.
    while (rlecnt >= 640)
    {
      txbuf[txbufidx++] = 127;
      rlecnt -= 640;
      if (txbufidx > 3)
      {
        my_stdio_usb_out_chars(txbuf, txbufidx);
        bytecnt += txbufidx;
        txbufidx = 0;
      }
    }
    // Coarse rle looks across the full word and allows a faster compare in cases with low activity factors
    // We must make sure cword==lword and that all nibbles of cword are the same
    if ((cword == lword) && ((cword >> 4) == (cword & 0x0FFFFFFF)))
    {
      rlecnt += 8;
#ifdef D4_DBG2
      Dprintf("coarse word 0x%X\n\r", cword);
#endif
    }
    else
    { // if coarse rle didn't match
#ifdef D4_DBG2
      Dprintf("cword 0x%X nibcurr 0x%X i %d rx idx %u  rlecnt %u \n\r", cword, nibcurr, i, rxbufdidx, rlecnt);
#endif
      lword = cword;
      for (int j = 0; j < 8; j++)
      { // process all 8 nibbles
        nibcurr = cword & 0xF;
        if (nibcurr == niblast)
        {
          rlecnt++;
        }
        else
        {
          // If the value changes we must push all remaing rles to the txbuf
          // chngcnt++;
          // Send intermediate 8..632 RLEs
          if (rlecnt > 7)
          {
            int rlemid = rlecnt & 0x3F8;
            txbuf[txbufidx++] = (rlemid >> 3) + 47;
          }
          // And finally the 0..7 rle along with the new value
          rlecnt &= 0x7;
#ifdef D4_DBG2 // print when sample value changes
          Dprintf("VChang val 0x%X rlecnt %d i%d j%d \n\r", nibcurr, rlecnt, i, j);
#endif
          txbuf[txbufidx++] = 0x80 | nibcurr | rlecnt << 4;
          rlecnt = 0;
        } // nibcurr!=last
        cword >>= 4;
        niblast = nibcurr;
      } // for j
    } // else (not a coarse rle )
#ifdef D4_DBG2
    Dprintf("i %d rx idx %u  rlecnt %u \n\r", i, rxbufdidx, rlecnt);
    Dprintf("i %u tx idx %d bufs 0x%X 0x%X 0x%X\n\r", i, txbufidx, txbuf[txbufidx - 3], txbuf[txbufidx - 2], txbuf[txbufidx - 1]);
#endif
    // Emperically found that transmitting groups of around 32B gives optimum bandwidth
    if (txbufidx >= 64)
    {
      my_stdio_usb_out_chars(txbuf, txbufidx);
      bytecnt += txbufidx;
      txbufidx = 0;
    }
  } // for i in samp_send>>3
  // At the end of processing the half send any residual samples as we don't maintain state between the halves
  // Maximal 640 values first
  while (rlecnt >= 640)
  {
    txbuf[txbufidx++] = 127;
    rlecnt -= 640;
  }
  // Middle rles 8..632
  if (rlecnt > 7)
  {
    int rleend = rlecnt & 0x3F8;
    txbuf[txbufidx++] = (rleend >> 3) + 47;
  }
  // 1..7 RLE
  // The rle and value encoding counts as both a sample count of rle and a new sample
  // thus we must decrement rlecnt by 1 and resend the current value which will match the previous values
  //(if the current value didn't match, the rlecnt would be 0).
  if (rlecnt)
  {
    rlecnt &= 0x7;
    rlecnt--;
    txbuf[txbufidx++] = 0x80 | nibcurr | rlecnt << 4;
    rlecnt = 0;
  }
  if (txbufidx)
  {
    my_stdio_usb_out_chars(txbuf, txbufidx);
    bytecnt += txbufidx;
    txbufidx = 0;
  }

} // send_slices_D4

// Send a digital sample of multiple bytes with the 7 bit encoding
static void inline tx_d_samp(sr_device_t *d, uint32_t cval)
{
  for (char b = 0; b < d->d_tx_bps; b++)
  {
    txbuf[txbufidx++] = (cval | 0x80);
    //      Dprintf("txds b %d cv 0x%X idx %d \n\r",b,cval,txbufidx);
    cval >>= 7;
  }
}

// Allow for 1,2 or 4B reads of sample data to reduce memory read overhead when
// parsing digital sample data.  This function is correct for all uses, but if included
// the compiled code is substantially slower to the point that digital only transfers
// can't keep up with USB rate.  Thus it is only used by the send_slices_analog which is already
// limited to 500khz, and in the starting send_slice_1/2/4
uint32_t get_cval(uint8_t *dbuf)
{
  uint32_t cval;
  if (d_dma_bps == 1)
  {
    cval = dbuf[rxbufdidx];
  }
  else if (d_dma_bps == 2)
  {
    cval = (*((uint16_t *)(dbuf + rxbufdidx)));
  }
  else
  {
    cval = (*((uint32_t *)(dbuf + rxbufdidx)));
// Mask off undefined channels
#ifdef DIG_26_MODE
    // push out the gap of 3 channels
    cval = (cval & MEM_D_MASK_L) | ((cval & MEM_D_MASK_U) >> 3);
#elif BASE_MODE
    // mask off upper unused
    cval = cval & MEM_D_MASK_L;
    // No change for DIG_32_MODE as all are defined
#endif
  }
  rxbufdidx += d_dma_bps;
  return cval;
}
/*RLE encoding for 5 or more channels has two ranges.
Decimal 48 to  79 are RLEs of 1 to 32 respectively.
Decimal 80 to 127 are (N-78)*32 thus 64,96..80,120..1568
Note that it is the responsibility of the caller to
forward txbuf bytes to USB to prevent txbufidx from overflowing the size
of txbuf. We do not always push to USB to reduce its impact
on performance.
 */
static void inline check_rle()
{
  //  Dprintf("RLEx %d\n\r",rlecnt);
  while (rlecnt >= 1568)
  {
    txbuf[txbufidx++] = 127;
    rlecnt -= 1568;
  }
  if (rlecnt > 32)
  {
    uint16_t rlediv = rlecnt >> 5;
    txbuf[txbufidx++] = rlediv + 78; // was 86;
    rlecnt -= rlediv << 5;
  }
  if (rlecnt)
  {
    txbuf[txbufidx++] = 47 + rlecnt;
    rlecnt = 0;
  }
}

// Send txbuf to usb based on an input threshold
void check_tx_buf(uint16_t cnt)
{
  if (txbufidx >= cnt)
  {
    //     Dprintf("txbuf idx %d cnt %d\n\r",txbufidx,cnt);
    my_stdio_usb_out_chars(txbuf, txbufidx);
    bytecnt += txbufidx;
    txbufidx = 0;
  }
}
// A common first digital byte to send to establish RLE.
// Not used for send_analog because it doesn't use RLE, and not used for D4 because it
// has a different RLE encoding
void send_first_dig_sample(sr_device_t *d, uint8_t *dbuf)
{
  lval = get_cval(dbuf);
  tx_d_samp(d, lval);
  samp_remain--;
  rlecnt = 0;
}
// There are three very similar functions send_slices_1B/2B/4B.
// Each of which  is very similar but exist because if a common function
// is used with the get_cval in the inner loop, the performance drops
// substantially.  Thus each function has a 1,2, or 4B aligned read respectively.
// We can't just always read a 4B value because the core doesn't support non-aligned accesses.
// These must be marked noinline to ensure they remain separate functions for good performance
// 1B is 5-8 channels
void __attribute__((noinline)) send_slices_1B(sr_device_t *d, uint8_t *dbuf)
{
  send_slice_init(d, dbuf);
  //   Dprintf("Enter 1Ba sts %d sr %d\n\r",d->samples_per_half,samp_remain);
  send_first_dig_sample(d, dbuf);
  for (int s = 0; s < samp_remain; s++)
  {
    cval = dbuf[rxbufdidx++];
    if (cval == lval)
    {
      rlecnt++;
    }
    else
    {
      //         Dprintf("SB n 0x%X o 0x%X rle %d ridx %d\n\r",cval,lval,rlecnt,rxbufdidx);
      check_rle();
      tx_d_samp(d, cval);
      check_tx_buf(TX_BUF_THRESH);
    } // if cval!=lval
    lval = cval;
  } // for s
  check_rle();
  check_tx_buf(1);
} // send_slices_1B

// 2B is 9-16 channels
// For all modes the sample bits are always continous/fully packed
void __attribute__((noinline)) send_slices_2B(sr_device_t *d, uint8_t *dbuf)
{
  send_slice_init(d, dbuf);
  send_first_dig_sample(d, dbuf);
  for (int s = 0; s < samp_remain; s++)
  {
    cval = (*((uint16_t *)(dbuf + rxbufdidx)));
    rxbufdidx += 2;
    if (cval == lval)
    {
      rlecnt++;
    }
    else
    {
      check_rle();
      tx_d_samp(d, cval);
      check_tx_buf(TX_BUF_THRESH);
    } // if cval!=lval
    lval = cval;
  } // for s
  check_rle();
  check_tx_buf(1);
} // send_slices_2B
// 4B is 17-21 channels in BASE_MODE
// It is also used with 17-26 channels in DIG_26_MODE, and 17-32 channels in DIG_32_MODE
void __attribute__((noinline)) send_slices_4B(sr_device_t *d, uint8_t *dbuf)
{
  send_slice_init(d, dbuf);
  send_first_dig_sample(d, dbuf);
  for (int s = 0; s < samp_remain; s++)
  {
    cval = (*((uint32_t *)(dbuf + rxbufdidx)));
    rxbufdidx += 4;
// Mask invalid bits, remove 29-31, and shift down 26-28 over 23-25
#ifdef DIG_26_MODE
    cval = (cval & MEM_D_MASK_L) | ((cval & MEM_D_MASK_U) >> 3);
#elif BASE_MODE
    // mask off upper unused
    cval = cval & MEM_D_MASK_L;
    // No change for DIG_32_MODE as all are defined
#endif
    if (cval == lval)
    {
      rlecnt++;
    }
    else
    {
      check_rle();
      tx_d_samp(d, cval);
      check_tx_buf(TX_BUF_THRESH);
    } // if cval!=lval
    lval = cval;
  } // for s
  check_rle();
  check_tx_buf(1);
} // send_slices_4B

// Slice transmit code, used for all cases with any analog channels
// All digital channels for one slice are sent first in 7 bit bytes using values 0x80 to 0xFF
// Analog channels are sent next, with each channel taking one 7 bit byte using values 0x80 to 0xFF.
// This does not support run length encoding because it's not clear how to define RLE on analog signals
// This functional will only be called in BASE_MODE, as neither DIG_26_MODE or DIG_32 mode
// have analog support
uint32_t send_slices_analog(sr_device_t *d, uint8_t *dbuf, uint8_t *abuf)
{
  send_slice_init(d, dbuf);
  uint32_t lval = 0;
  for (int s = 0; s < samp_remain; s++)
  {
    if (d->d_mask)
    {
      cval = get_cval(dbuf);
      tx_d_samp(d, cval);
      // Dprintf("s %d cv %X bps %d idx t %d r %d \n\r",s,cval,d_dma_bps,txbufidx,rxbufdidx);
    }
    for (char i = 0; i < d->a_chan_cnt; i++)
    {
#ifdef ADS1256_MODE
      // ADS1256 uses 3 wire bytes per sample
      if (!ads1256_multichan)
      {
        // Wait for a sample to be available in ring buffer
        while (ads1256_ring_rd == ads1256_ring_wr)
        {
          if (!ads1256_core1_run)
            break;
          tight_loop_contents();
        }
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
      }
      else
      {
        txbuf[txbufidx++] = abuf[rxbufaidx++];
        txbuf[txbufidx++] = abuf[rxbufaidx++];
        txbuf[txbufidx++] = abuf[rxbufaidx++];
      }
#else
      txbuf[txbufidx] = (abuf[rxbufaidx] >> 1) | 0x80;
      txbufidx++;
      rxbufaidx++;
      // Dprintf("av %X cnt %d idx t %d r %d\n\r",abuf[rxbufaidx-1],d->a_chan_cnt,txbufidx,rxbufaidx);
#endif
    }
    // Since this doesn't support RLEs we don't need to buffer
    // extra bytes to prevent txbuf overflow, but this value
    // works well anyway
    check_tx_buf(TX_BUF_THRESH);
  } // for s
  check_tx_buf(1);
}
// send_slices_analog

// This function monitors the dma interrupt handler outputs to send the remainder of a full DMA buffer.
void send_half(void)
{
  bool sendlower;
  uint32_t dbuf_start, abuf_start;
  sendlower = (num_halves & 1) ? false : true;
  // return immediately if not in a sending state
  if ((dev.state == SENDING) || (dev.state == DMA_DONE))
  {
    sho_cnt++;
  }
  else
  {
    return;
  }
  // We have a full DMA buffer, send it.
  if (dma_halves > num_halves)
  {
    tx_cnt++;
    dbuf_start = sendlower ? dev.dbuf0_start : dev.dbuf1_start;
    abuf_start = sendlower ? dev.abuf0_start : dev.abuf1_start;
    // Dprintf("d buffers %d %d %d\n\r",dev.dbuf0_start,dev.dbuf1_start,dbuf_start);
    // Dprintf("a buffers %d %d %d\n\r",dev.abuf0_start,dev.abuf1_start,abuf_start);
    if (dev.a_mask)
    {
      send_slices_analog(&dev, &(capture_buf[dbuf_start]),
                         &(capture_buf[abuf_start]));
    }
    else if (d_dma_bps == 0)
    {
      send_slices_D4(&dev, &(capture_buf[dbuf_start]));
    }
    else if (d_dma_bps == 1)
    {
      send_slices_1B(&dev, &(capture_buf[dbuf_start]));
    }
    else if (d_dma_bps == 2)
    {
      send_slices_2B(&dev, &(capture_buf[dbuf_start]));
    }
    else
    {
      send_slices_4B(&dev, &(capture_buf[dbuf_start]));
    }
    num_halves++;
  } // if dma_halves>num_halves
#ifdef ADS1256_MODE
  if (dev.state == SENDING && !dev.cont && dma_halves >= exp_halves)
  {
    dev.state = DMA_DONE;
  }
#endif
  // If we ever recieve a usb_plus, consider all samples to be sent, even if not in continuous mode
  if (dev.usb_plus)
  {
    dev.state = SAMPLES_SENT;
    //   Dprintf("SH_USB_PLUS_SS\n\r");
    // At DMA_DONE transition to SAMPLES_SENT when all samples are sent
  }
  else if (dev.state == DMA_DONE)
  {
    if ((dev.scnt >= dev.num_samples) || (dev.cont == true))
    {
      dev.state = SAMPLES_SENT;
      Dprintf("SH_SSENT %d %d\n\r", dev.scnt, dev.num_samples);
    }
    else
    {
      // Even with dma disabled, we still might have one more half buffer to send
      // so allow this loop to be called again for the remaining half buffer
      if (dma_halves - num_halves == 1)
      {
        // Dprintf("ONEMORE\n\r");
      }
      else
      {
        if (mask_xfer_err == false)
        {
          // If we have more than one extra we have some kind of overflow/error/abort etc
          // that isn't expected.
          Dprintf("Unexpected state cnt %d %d halves %d %d %d\n\r", dev.scnt, dev.num_samples, exp_halves, dma_halves, num_halves);
          dev.state = ABORTED;
        }
      } // else
    } // DMA_DONE
  } // else
} // send_half

// Handle interrupts generated by ADC or PIO.  If both are enabled they may come in
// either order, so wait for both if only one is seen.
void dma_int_handler()
{
  int sts;
  // Have we detected any cases were dma should be turnned off and interrupts disabled?
  // this includes error/abort and non error/abort cases
  bool dma_done = false;
  bool partial = true;
  sts = dma_hw->ints0;
  currintmask |= sts;
  // We shouldn't reach the interrupt handler until state transitions from
  // started to sending
  if (dev.state == STARTED)
  {
    Dprintf("ERR: int handler at started %X\n\r", dma_hw->ints0);
    dma_hw->ints0 = dma_hw->ints0;
    dev.state = ABORTED;
    dma_done = true;
  }
  // If we aren't in sending state then we don't need DMA results
  else if (dev.state != SENDING)
  {
    Dprintf("INT skip state %d \n\r", dev.state);
    dma_done = true;
  }
  // The "+" from the host ends a transmit, in both continuous and non-continuous modes
  else if (dev.usb_plus)
  {
    Dprintf("INT skip plus\n\r");
    dma_done = true;
  }
  // This first checks says that if we have seen an IRQ for either of the
  // lower halves and an IRQ for either of the upper halves that we have overflowed.
  // This is rather exceptional as it means we managed to finish DMAs for both
  // halves and only called the interrupt handler once.
  else if ((mask_xfer_err == false) && ((currintmask & h0intmask) && (currintmask & h1intmask)))
  {
    Dprintf("Int Overflow0 a %d b %d c %d d %d e %d halves %d %d masks %X %X %X \n\r",
            acnt, bcnt, ccnt, dcnt, ecnt, dma_halves, num_halves, currintmask, h0intmask, h1intmask);
    dma_done = true;
    dev.state = ABORTED;
  }
  // If we haven't detected any errors process the current interrupt mask
  // Note that in very high sample rates with low number of samples we may reach this interrupt handler
  // with both halves being valid, so check and clear each half independently
  else
  {
    if ((currintmask & h0intmask) == h0intmask)
    {
      // Dprintf("H0\n\r");
      partial = false;
      currintmask = currintmask & ~h0intmask;
      dma_halves++;
      // print_DMA();
    }
    // We have gotten the expected interrupts for upper
    if ((currintmask & h1intmask) == h1intmask)
    {
      // Dprintf("H1\n\r");
      partial = false;
      currintmask = currintmask & ~h1intmask;
      dma_halves++;
      // print_DMA();
    }
  }
  // A partial means we have gotten either the PIO or the DMA for a given half
  // but not both, in that case exit the handler waiting for the other to catch up
  if (partial)
  {
    // Dprintf("PRT %X\n",currintmask);
  }

  // This 2nd overflow check says if dma_halves is more than one ahead of num_halves then we are starting to
  // overwrite a buffer we are sending. Note that it is after we increment dma_halves .
  if ((mask_xfer_err == false) && (dma_halves - num_halves > 1))
  {
    Dprintf("Int Overflow1 a %d b %d c %d d %d e %d halves %d %d masks %X %X %X \n\r",
            acnt, bcnt, ccnt, dcnt, ecnt, dma_halves, num_halves, currintmask, h0intmask, h1intmask);
    dma_done = true;
    dev.state = ABORTED;
  }
  // Stop non continous mode when we reach expected number of halves
  if (dma_halves == exp_halves)
  {
    // Dprintf("EXP_DNE %d\n\r",exp_halves);
    dma_done = true;
  }
  // Once dma_done is detected disable all the dma channels and then disable
  // the interrupts, and then change the state.  This is not a full
  // cleanup of all state because we still must send the remaining samples
  // and the bytecnt etc, so it's just enough to stop dma transfers and interrupts
  if (dma_done)
  {
    // Dprintf("DMADNE %X\n\r",sts);
    currintmask = 0;
    // Aborting a DMA engine can lead to undefined behavior.  Thus the main
    // channels are not aborted as data corruption was seen in some cases.
    // Aborting the maintenance channels is generally safe because they are only one
    // DMA operation and aborting them is sufficient to break the chain and prevent
    // new DMA operations from overwritting data that hasn't been sent across USB.
    // dma_channel_abort(admachan0);
    // dma_channel_abort(admachan1);
    // dma_channel_abort(pdmachan0);
    // dma_channel_abort(pdmachan1);
    dma_channel_abort(amaintchan0);
    dma_channel_abort(amaintchan1);
    dma_channel_abort(pmaintchan0);
    dma_channel_abort(pmaintchan1);
    dma_channel_set_irq0_enabled(admachan0, false);
    dma_channel_set_irq0_enabled(pdmachan0, false);
    dma_channel_set_irq0_enabled(admachan1, false);
    dma_channel_set_irq0_enabled(pdmachan1, false);
    if ((dev.state != ABORTED) && (dev.usb_plus == false))
    {
      dev.state = DMA_DONE;
    }
  }
  // clear the pended interrupt
  dma_hw->ints0 = sts;
}
#ifdef PIN_TEST_MODE
// Force a test pattern to ensure the design is working.  Ideally we could DMA
// from a counter and write to the GPIOs. However, DMA isn't allowed access to SIO.
// We could also use the PIOs, but PIOs are used for the main functionaliy
// and PIO clocks are modified based on sample rate, so using the normal timer ensures
// more predictable timing.
// Note: This function makes no attempt to determine which pins are enabled.
// Note: During transitions between DMA engines the delay between timers may jump from
// 100us to ~300us, thus "stretching" the sample values of the test.  Not sure why
// such a large delta exists, but just be aware of the limitation. It may only happen with
// high rates of serial print outs...
bool pin_test_timer_callback(__unused struct repeating_timer *t)
{
  //  Dprintf(".\n\r");
  //  pin_test_cnt=pin_test_cnt+0x00100001; //0xF070301;
  pin_test_cnt = pin_test_cnt + 0x01001001; // 0xF070301;
  if (dev.state == SENDING)
  {
    systick_array[systick_idx++] = time_us_32();
    systick_idx %= 128;
  }
  // In case the user forgets to not drive the pins, only drive the test signal when not idle
  // to limit drive fights.
  if (dev.state != IDLE)
  {
    sio_hw->gpio_out = pin_test_cnt;
    // mask        value
    gpio_set_dir_masked(PIN_TEST_MASK, PIN_TEST_MASK); // masked set per pin.  1 is output, 0 is input
  }
  else
  {
    sio_hw->gpio_out = 0;
    // mask        value
    gpio_set_dir_masked(PIN_TEST_MASK, 0); // masked set per pin.  1 is output, 0 is input
  }
  return true;
}
void core1_entry()
{
  Dprintf("*********WARNING PIN TEST MODE-DO NOT CONNECT TO PINS\n\r");
  gpio_init_mask(PIN_TEST_MASK); // set to function SIO as input
  // Note the pin directions are handled in the timer call back.
  // gpio_set_dir_masked(PIN_TEST_MASK,PIN_TEST_MASK); //masked set per pin.  1 is output, 0 is input
  //                      delay in us, call back,      userdata, timer
  add_repeating_timer_us(100, pin_test_timer_callback, NULL, &pt_timer);
  gpio_set_dir_masked(PIN_TEST_MASK, PIN_TEST_MASK); // masked set per pin.  1 is output, 0 is input
  Dprintf("Pin Test Mode Timer Added\n\r");
  while (1)
  {
    // Attempt to sleep the core to reduce contention for the memory bus
    // and maybe save some power
    __wfe();
  }
} // core1_entry

#endif // PIN_TEST_MODE
