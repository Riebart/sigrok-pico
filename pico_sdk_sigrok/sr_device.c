#include "sr_device.h"
#include "hardware/uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/time.h"    // sleep_ms
#include "pico/bootrom.h" // rom_reset_usb_boot

int Dprintf(const char *fmt, ...)
{

#if (UART_EN == 1)
   va_list argptr;
   int len = 1;
   char _dstr[256];

   memset(&_dstr, 0x00, sizeof(_dstr));
   va_start(argptr, fmt);
   len = vsnprintf(_dstr, sizeof(_dstr), fmt, argptr);
   va_end(argptr);

   if ((len > 0) && (len < 240))
   {
      uart_puts(uart0, _dstr);
      uart_tx_wait_blocking(uart0);
   }
   else
   {
      uart_puts(uart0, "UART OVRFLW");
      uart_tx_wait_blocking(uart0);
   }
   return len;
#else
   return 0;
#endif
}

// reset as part of init, or on a completed send
void reset(sr_device_t *d)
{
   d->state == IDLE;
   d->cont = 0;
   d->scnt = 0;
};

// initial post reset state
void init(sr_device_t *d)
{
   reset(d);
   d->a_mask = 0;
   d->d_mask = 0;
   d->sample_rate = 5000;
   d->num_samples = 10;
   d->a_chan_cnt = 0;
   d->d_nps = 0;
   d->cmdstrptr = 0;
}

void tx_init(sr_device_t *d)
{
   d->a_chan_cnt = 0;
   for (int i = 0; i < NUM_A_CHAN; i++)
   {
      if (((d->a_mask) >> i) & 1)
      {
         d->a_chan_cnt++;
      }
   }
   // Nibbles per slice controls how PIO digital data is stored
   d->d_nps = (d->d_mask & 0xF) ? 1 : 0;
   d->d_nps = (d->d_mask & 0xF0) ? (d->d_nps) + 1 : d->d_nps;
   d->d_nps = (d->d_mask & 0xFF00) ? (d->d_nps) + 2 : d->d_nps;
   d->d_nps = (d->d_mask & 0xFFFF0000) ? (d->d_nps) + 4 : d->d_nps;
   if ((d->d_nps == 1) && (d->a_chan_cnt > 0))
   {
      d->d_nps = 2;
   }

   d->d_chan_cnt = 0;
   for (int i = 0; i < NUM_D_CHAN; i++)
   {
      if (((d->d_mask) >> i) & 1)
      {
         d->d_chan_cnt++;
      }
   }
   d->d_tx_bps = (d->d_chan_cnt + 6) / 7;
   d->state = STARTED;
}

// Process incoming character stream
// Return 1 if the device rspstr has a response to send to host
// Be sure that rspstr does not have \n  or \r.
int process_char(sr_device_t *d, char charin)
{
   int tmpint, tmpint2, ret;
   // set default rspstr for all commands that have a dataless ack
   d->rspstr[0] = '*';
   d->rspstr[1] = 0;
   // the reset character works by itself
   if (charin == '*')
   {
      reset(d);
      Dprintf("RST* %d\n\r", d->state);
      return 0;
   }
   else if ((charin == '\r') || (charin == '\n'))
   {
      d->cmdstr[d->cmdstrptr] = 0;
      switch (d->cmdstr[0])
      {
      case 'b':
         if (!strcmp(d->cmdstr, "bootsel"))
         {
            Dprintf("Entering Bootsel mode\n\r");
            sleep_ms(1000);
            rom_reset_usb_boot(0, 0);
         }
         else
         {
            Dprintf("Invald bootsel command - enter \"bootsel\"\n\r");
         }
         ret = 0;
         break;

      case 'i':
         // SRPICO,AxxyDzz,00
         // xx  = number of analog channels (NUM_A_CHAN)
         // y   = bytes per analog sample on the wire (asize):
         //         1 for onboard ADC (7-bit, baseline mode)
         //         ADS1256_A_BYTES (3) for ADS1256 mode (21-bit, 3x7-bit)
         // zz  = number of digital channels (NUM_D_CHAN)
         // ,02 = firmware version
#ifdef ADS1256_MODE
         sprintf(d->rspstr, "SRPICO,A%02d%dD%02d,02",
                 NUM_A_CHAN, ADS1256_A_BYTES, NUM_D_CHAN);
#else
         sprintf(d->rspstr, "SRPICO,A%02d1D%02d,02", NUM_A_CHAN, NUM_D_CHAN);
#endif
         Dprintf("ID rsp %s\n\r", d->rspstr);
         ret = 1;
         break;

      case 'R':
         tmpint = atol(&(d->cmdstr[1]));
         if ((tmpint >= 5000) && (tmpint <= 120000016))
         {
            d->sample_rate = tmpint;
            ret = 1;
         }
         else
         {
            Dprintf("unsupported smp rate %s\n\r", d->cmdstr);
            ret = 0;
         }
         break;

      case 'L':
         tmpint = atol(&(d->cmdstr[1]));
         if (tmpint > 0)
         {
            d->num_samples = tmpint;
            ret = 1;
         }
         else
         {
            Dprintf("bad num samples %s\n\r", d->cmdstr);
            ret = 0;
         }
         break;

      // get analog scale
      // Format response: "<scale_uV>x<offset_uV>"
      // scale_uV and offset_uV are integers, as expected by the
      // libsigrok raspberrypi-pico driver which divides by 1e6 to get volts.
      case 'a':
         tmpint = atoi(&(d->cmdstr[1])); // extract channel number
         if (tmpint >= 0)
         {
#ifdef ADS1256_MODE
            // Scale and offset are computed at compile time from:
            //   ADS1256_PGA_GAIN  (default 1, can be 1/2/4/8/16/32/64)
            //   ADS1256_BUF_ENABLE (default 1; buffer state does not change
            //                       the full-scale span when VREF is external)
            //   ADS1256_VREF_UV   (2500000 uV = 2.5 V)
            //   ADS1256_A_RSHIFT  (3 bits dropped from 24-bit result)
            //   Denominator       (2^20 = 1048576 counts for 21-bit range)
            // All channels share the same scale/offset in single-ended mode.
            sprintf(d->rspstr, "%ldx%ld",
                    (long)ADS1256_SCALE_UV,
                    (long)ADS1256_OFFSET_UV);
#else
            sprintf(d->rspstr, "25700x0"); // 3.3/(2^7) and 0V offset
#endif
            Dprintf("ASCL%d rsp %s\n\r", tmpint, d->rspstr);
            ret = 1;
         }
         else
         {
            Dprintf("bad ascale %s\n\r", d->cmdstr);
            ret = 1; // returns '*' causing the host to fail on bad channel
         }
         break;

      case 'F':
         Dprintf("STRT_FIX\n\r");
         tx_init(d);
         d->cont = 0;
         ret = 0;
         break;

      case 'C':
         tx_init(d);
         d->cont = 1;
         Dprintf("STRT_CONT\n\r");
         ret = 0;
         break;

      case 't':
         ret = 1;
         break;

      case 'p':
         tmpint = atoi(&(d->cmdstr[1]));
         Dprintf("Pre-trigger samples %d cmd %s\n\r", tmpint, d->cmdstr);
         ret = 1;
         break;

      // Enable/disable analog channel
      // format is Axyy where x is 0/1 and yy is channel number
      case 'A':
         tmpint = d->cmdstr[1] - '0';
         tmpint2 = atoi(&(d->cmdstr[2]));
         if ((tmpint >= 0) && (tmpint <= 1) && (tmpint2 >= 0) && (tmpint2 <= 31))
         {
            d->a_mask = d->a_mask & ~(1 << tmpint2);
            d->a_mask = d->a_mask | (tmpint << tmpint2);
            ret = 1;
         }
         else
         {
            ret = 0;
         }
         break;

      // Enable/disable digital channel
      // format is Dxyy where x is 0/1 and yy is channel number
      case 'D':
         tmpint = d->cmdstr[1] - '0';
         tmpint2 = atoi(&(d->cmdstr[2]));
         if ((tmpint >= 0) && (tmpint <= 1) && (tmpint2 >= 0) && (tmpint2 <= 31))
         {
            d->d_mask = d->d_mask & ~(1 << tmpint2);
            d->d_mask = d->d_mask | (tmpint << tmpint2);
            Dprintf("D%d EN %d Msk 0x%X\n\r", tmpint2, tmpint, d->d_mask);
            ret = 1;
         }
         else
         {
            ret = 0;
         }
         break;

      // Get signal name
      // format: n(A/D)xx
      case 'n':
         tmpint = atoi(&(d->cmdstr[2]));
         if (d->cmdstr[1] == 'D')
         {
#ifdef ADS1256_MODE
            // Dxx maps to GPxx with an offset based on which SPI peripheral we're using.
            tmpint2 = tmpint + ADS1256_D_CHAN_OFFSET;
#elif defined(BASE_MODE)
            tmpint2 = tmpint + 2; // D0-D20 are GP2..GP22
#elif defined(DIG_26_MODE)
            tmpint2 = (tmpint <= 22) ? tmpint : tmpint + 3;
#else // DIG_32_MODE
            tmpint2 = tmpint;
#endif
            Dprintf("NameD %c %d %d\n\r", d->cmdstr[1], tmpint, tmpint2);
            sprintf(d->rspstr, "GP%d", tmpint2);
            ret = 1;
         }
         else if (d->cmdstr[1] == 'A')
         {
#ifdef ADS1256_MODE
            // AIN0-AIN7 single-ended vs AINCOM
            sprintf(d->rspstr, "AIN%d", tmpint);
#else
            tmpint2 = tmpint + 26;
            sprintf(d->rspstr, "ADC%d_GP%d", tmpint, tmpint2);
#endif
            Dprintf("NameA %c %d\n\r", d->cmdstr[1], tmpint);
            ret = 1;
         }
         else
         {
            ret = 0;
         }
         break;

      default:
         Dprintf("bad command %s\n\r", d->cmdstr);
         ret = 0;
      } // switch
      d->cmdstrptr = 0;
   }
   else // no CR/LF yet
   {
      if (d->cmdstrptr >= 19)
      {
         d->cmdstr[18] = 0;
         Dprintf("Command overflow %s\n\r", d->cmdstr);
         d->cmdstrptr = 0;
      }
      d->cmdstr[d->cmdstrptr++] = charin;
      ret = 0;
   }
   return ret;
} // process_char
