int main()
{
  int delay = 100;
  stdio_usb_init();
#if (UART_EN == 1)
  uart_set_format(uart0, 8, 1, 0);
  uart_init(uart0, UART_BAUD);
  gpio_set_function(0, GPIO_FUNC_UART);
  // The uart Rx has never been used, but left in for the baseline definition
  gpio_set_function(1, GPIO_FUNC_UART);
#endif
  // This sleep may not be necessary, but was added to give USB extra time to come up.
  // But an extra .1 seconds won't bother anything...
  sleep_us(100000);
  Dprintf("\n\rHello from PICO sigrok device \n\r");
// Pulse the LED in a morse code "P" to confirm programming
#ifdef HAS_LED
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  // More Code "P"
  // dit
  gpio_put(PICO_DEFAULT_LED_PIN, true);
  sleep_ms(delay);
  gpio_put(PICO_DEFAULT_LED_PIN, false);
  sleep_ms(delay);
  // dah
  gpio_put(PICO_DEFAULT_LED_PIN, true);
  sleep_ms(delay * 3);
  gpio_put(PICO_DEFAULT_LED_PIN, false);
  sleep_ms(delay);
  // dah
  gpio_put(PICO_DEFAULT_LED_PIN, true);
  sleep_ms(delay * 3);
  gpio_put(PICO_DEFAULT_LED_PIN, false);
  sleep_ms(delay);
  // dit
  gpio_put(PICO_DEFAULT_LED_PIN, true);
  sleep_ms(delay);
  gpio_put(PICO_DEFAULT_LED_PIN, false);
  sleep_ms(delay * 2);

#endif
  uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
  Dprintf("pll_sys = %dkHz\n\r", f_pll_sys);
  uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  Dprintf("clk_sys = %dkHz\n\r", f_clk_sys);
#ifndef DIG_32_MODE
  // Set GPIO23 (TP4) to control switched mode power supply noise
  // This may reduce noise into the ADC in some use cases.
  gpio_init_mask(1 << 23);
  gpio_set_dir_masked(1 << 23, 1 << 23);
  gpio_put_masked(1 << 23, 1 << 23);
#endif
  // Early CDC IO code had lots of sleep statements, but the TUD code seems to have sufficient
  // checks that this isn't needed, but it doesn't hurt...
  sleep_us(100000);
// GPIOs 26 through 28 (the ADC ports) ar
// GPIOs 26 through 28 (the ADC ports) are on the PICO, GPIO29 is not a pin on the PICO
// Note that digital only modes don't block all configuration related to ADC, but does enough
// to ensure we can properly sample the pins digitally.
#ifdef BASE_MODE
#ifndef ADS1256_MODE
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_init();
#endif
#endif

  halves_seen = 0;

  admachan0 = dma_claim_unused_channel(true);
  pdmachan0 = dma_claim_unused_channel(true);
  admachan1 = dma_claim_unused_channel(true);
  pdmachan1 = dma_claim_unused_channel(true);
  amaintchan0 = dma_claim_unused_channel(true);
  amaintchan1 = dma_claim_unused_channel(true);
  pmaintchan0 = dma_claim_unused_channel(true);
  pmaintchan1 = dma_claim_unused_channel(true);
  Dprintf("DMA Channels A0 %d P0 %d A1 %d P1 %d M %d %d %d %d\n",
          admachan0, pdmachan0, admachan1, pdmachan1, amaintchan0, amaintchan1, pmaintchan0, pmaintchan1);
  acfg0 = dma_channel_get_default_config(admachan0);
  acfg1 = dma_channel_get_default_config(admachan1);
  pcfg0 = dma_channel_get_default_config(pdmachan0);
  pcfg1 = dma_channel_get_default_config(pdmachan1);
  amcfg0 = dma_channel_get_default_config(amaintchan0);
  amcfg1 = dma_channel_get_default_config(amaintchan1);
  pmcfg0 = dma_channel_get_default_config(pmaintchan0);
  pmcfg1 = dma_channel_get_default_config(pmaintchan1);
  // ADC transfer 1 bytes, PIO transfer the 4B default
  // maintenance transfers 1 32b write pointers
  channel_config_set_transfer_data_size(&acfg0, DMA_SIZE_8);
  channel_config_set_transfer_data_size(&acfg1, DMA_SIZE_8);
  channel_config_set_transfer_data_size(&pcfg0, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&pcfg1, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&amcfg0, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&amcfg1, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&pmcfg0, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&pmcfg1, DMA_SIZE_32);
  // no dmas do read increments
  channel_config_set_read_increment(&acfg0, false);
  channel_config_set_read_increment(&acfg1, false);
  channel_config_set_read_increment(&pcfg0, false);
  channel_config_set_read_increment(&pcfg1, false);
  channel_config_set_read_increment(&amcfg0, false);
  channel_config_set_read_increment(&amcfg1, false);
  channel_config_set_read_increment(&pmcfg0, false);
  channel_config_set_read_increment(&pmcfg1, false);
  // ADC and PIO do write increments, the maintenance don't
  channel_config_set_write_increment(&acfg0, true);
  channel_config_set_write_increment(&acfg1, true);
  channel_config_set_write_increment(&pcfg0, true);
  channel_config_set_write_increment(&pcfg1, true);

  channel_config_set_write_increment(&amcfg0, false);
  channel_config_set_write_increment(&amcfg1, false);
  channel_config_set_write_increment(&pmcfg0, false);
  channel_config_set_write_increment(&pmcfg1, false);

  // Pace transfers based on availability of ADC samples or PIO samples
  channel_config_set_dreq(&acfg0, DREQ_ADC);
  channel_config_set_dreq(&acfg1, DREQ_ADC);
  channel_config_set_dreq(&pcfg0, pio_get_dreq(pio, piosm, false));
  channel_config_set_dreq(&pcfg1, pio_get_dreq(pio, piosm, false));

  // Maintenance transfer once as fast as possible
  // Note that this most likely does not use DMA channel 0, but that's the best
  // value defined by the SDK that is common to rp2040 and rp2350
  channel_config_set_dreq(&amcfg0, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
  channel_config_set_dreq(&amcfg1, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
  channel_config_set_dreq(&pmcfg0, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);
  channel_config_set_dreq(&pmcfg1, DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT);

  // Chaining is always enabled with PIO0->PMAINT0->PIO1->PMAINT1->PIO0
  // and ADMA0->AMAINT0->ADMA1->AMAINT1->ADMA0, except in mask_xfer_err case where
  // we know we will only have two half buffers and will likely fill them way before
  // we can process them.  In that case we end the chaining by not doing a chaint_to
  // from maintenance 1.
  // The mask_xfer_err override is handled as part of the init for each sample run
  channel_config_set_chain_to(&acfg0, amaintchan0);
  channel_config_set_chain_to(&amcfg0, admachan1);
  channel_config_set_chain_to(&acfg1, amaintchan1);
  channel_config_set_chain_to(&pcfg0, pmaintchan0);
  channel_config_set_chain_to(&pmcfg0, pdmachan1);
  channel_config_set_chain_to(&pcfg1, pmaintchan1);
  // PIO status
  // TODO (coding style) - use a better csr reference name that points diretly i.e. pio0_hw->ctrl
  volatile uint32_t *pioctrl, *piofstts, *piodbg, *pioflvl;
  pioctrl = (volatile uint32_t *)(PIO0_BASE);        // PIO CTRL
  piofstts = (volatile uint32_t *)(PIO0_BASE + 0x4); // PIO FSTAT
  piodbg = (volatile uint32_t *)(PIO0_BASE + 0x8);   // PIO DBG
  pioflvl = (volatile uint32_t *)(PIO0_BASE + 0x10); // PIO FLVL
  // TODO (coding style)- use a better csr reference name that points diretly
  volatile uint32_t *pio0sm0clkdiv;
  pio0sm0clkdiv = (volatile uint32_t *)(PIO0_BASE + 0xc8);
  // Give High priority to DMA to ensure we don't overflow the PIO or DMA fifos
  // The DMA controller must read across the common bus to read the PIO fifo so enabled both reads and write
  bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

  init(&dev);
  // Since RP2040 is 32 bit this should always be 4B aligned, and it must be because the PIO
  // does DMA on a per byte basis
  // If either malloc fails the code will just hang
  Dprintf("Malloc %d bytes\n\r", DMA_BUF_SIZE);
  capture_buf = malloc(DMA_BUF_SIZE);
  Dprintf("DMA capture buf start %p\n\r", (void *)capture_buf);
  // Ensure we leave 10k or more left for any dynamic allocations that need it
  uint8_t *tptr;
  tptr = malloc(10000);
  Dprintf("10K free start %p\n\r", (void *)tptr);
  free(tptr);

  gpio_init_mask(GPIO_D_MASK);         // set as GPIO_FUNC_SIO and clear output enable
  gpio_set_dir_masked(GPIO_D_MASK, 0); // Set all to input
#ifdef ADS1256_MODE
  multicore_launch_core1(ads1256_core1_entry);
#endif
// Core1 for PIN_TEST_MODE is called at the end so that gpio inits are done
// after all of the other previous ones that define them as inputs only.
// It also helps above Dprintf/UART conflicts between the two cores
#ifdef PIN_TEST_MODE
  multicore_launch_core1(core1_entry);
  // Avoid UART print conflicts by delaying core0 as core1 starts
  sleep_us(100000);
#endif // PIN_TEST_MODE

  while (1)
  {
    ecnt++;
#ifdef ADS1256_MODE
    // Issue #3: Cleanly catch ring buffer overflow and abort
    if (ads1256_ring_overflow)
    {
      Dprintf("ADS1256 ring buffer overflow! Aborting.\n\r");
      dev.state = ABORTED;
      ads1256_ring_overflow = false;
    }

    // Drain ADS1256 single-channel ring buffer when digital is disabled
    if ((dev.state == SENDING || dev.state == DMA_DONE) && dev.d_mask == 0)
    {
      while (ads1256_ring_rd != ads1256_ring_wr)
      {
        uint32_t available = (ads1256_ring_wr >= ads1256_ring_rd) ? (ads1256_ring_wr - ads1256_ring_rd) : (ADS1256_RING_BYTES - ads1256_ring_rd + ads1256_ring_wr);
        if (available < 3)
        {
          break;
        }
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
        txbuf[txbufidx++] = ads1256_ring[ads1256_ring_rd];
        ads1256_ring_rd = (ads1256_ring_rd + 1) % ADS1256_RING_BYTES;
        dev.scnt++;
        if (txbufidx >= TX_BUF_THRESH)
        {
          my_stdio_usb_out_chars(txbuf, txbufidx);
          bytecnt += txbufidx;
          txbufidx = 0;
        }
        if (!dev.cont && dev.scnt >= dev.num_samples)
        {
          if (txbufidx > 0)
          {
            my_stdio_usb_out_chars(txbuf, txbufidx);
            bytecnt += txbufidx;
            txbufidx = 0;
          }
          dev.state = SAMPLES_SENT;
          break;
        }
      }
    }
#endif
    if (send_resp)
    {
      acnt++;
      int mylen = strlen(dev.rspstr);
      // Don't mix printf with direct to usb commands
      // printf("%s",dev.rspstr);
      my_stdio_usb_out_chars(dev.rspstr, mylen);
      send_resp = false;
    }
    // This testmode forces the device into a capture state without needing
    // sigrok cli/pulseview to initiate it
    // It must be placed immediately before the dev.sending/started check just below
    if (forced_test_mode_en)
    {
      dev.cont = false;
      // These must start from 0 and go up.
      dev.d_mask = 0xF;
      dev.a_mask = 0x1;
      // min 5khz sample rate
      // TODO (ADC)-Need to add support for ADC clocking/overclocking
      //- setting to 1MHZ seems to break if ADC is enabled .
      // To do this likely requires a sysclk boost
      dev.sample_rate = 2000000;
      dev.num_samples = 2000000;
      dev.scnt = 0; // number of samples sent

      // Clear this on first past
      forced_test_mode_en = false;
      forced_test_mode_run = true;
      Dprintf("Enter forced test mode %d %X \n\r", dev.a_chan_cnt, dev.a_mask);
      tx_init(&dev);
    }
    if (dev.state == STARTED)
    {
      bool adc_aborting = false;
      Dprintf("STRTING\n\r");

#ifdef ADS1256_MODE
      // Issue #6: only start Core 1 ADS1256 sampling if analog channels
      // are actually requested. A pure digital capture must not touch the
      // ADS1256 or the SPI bus.
      if (dev.a_chan_cnt > 0)
      {
        ads1256_multichan = (dev.a_chan_cnt > 1);
        if (!ads1256_multichan)
        {
          // Find which single channel is enabled
          for (int i = 0; i < NUM_A_CHAN; i++)
          {
            if ((dev.a_mask >> i) & 1)
            {
              ads1256_single_ch = i;
              break;
            }
          }
        }
        ads1256_core1_run = true;
      }
      dev.state = SENDING;
#endif

      // Sample rate must always be even.  Pulseview code enforces this
      // because it specifies a fixed set of frequencies, but sigrok cli can still odd ones.
      dev.sample_rate >>= 1;
      dev.sample_rate <<= 1;
      // Adjust up and align to 4 to avoid rounding errors etc
      if (dev.num_samples < 16)
      {
        dev.num_samples = 16;
      }
      dev.num_samples = (dev.num_samples + 3) & 0xFFFFFFFC;
      // Divide capture buf evenly based on channel enables
      // d_size is aligned to 4 bytes because pio operates on words
      // These are the sizes for each half buffer in bytes
      // Calculate relative size in terms of nibbles which is the smallest unit.
      // For onboard ADC modes, each analog sample is 1 byte = 2 nibbles, so a_nibbles = a_chan_cnt * 2.
      // For ADS1256 mode, each analog sample is ADS1256_A_BYTES (3) bytes = 6 nibbles per channel.
      // Using the correct nibble count ensures d_size, a_size, and samples_per_half are all
      // proportionally correct and that the half_bytes trigger in run_multi_channel() fires
      // at the right boundary.
      uint32_t d_nibbles, a_nibbles, t_nibbles; // digital, analog and total nibbles
      d_nibbles = dev.d_nps;                    // digital is in groups of 4 bits
#ifdef ADS1256_MODE
      // Issue #5 fix: ADS1256 produces 3 wire bytes per sample, not 1.
      // a_nibbles must reflect the true byte width so that chunk_size,
      // dev.a_size, and samples_per_half are computed correctly.
      a_nibbles = dev.a_chan_cnt * ADS1256_A_BYTES * 2;
#else
      // Note that this code only supports 7 bit accurate ADC modes.
      a_nibbles = dev.a_chan_cnt * 2; // 1 byte per sample
#endif
      t_nibbles = d_nibbles + a_nibbles;
      // total buf size must be a multiple of a_nibbles*2, d_nibbles*8, and t_nibbles so that
      // division is always in whole samples.
      // Also set a multiple of 32  because the dma buffer is split in half, and
      // the PIO does writes on 4B boundaries, and then a 4x factor for any other size/alignment issues
      uint32_t chunk_size = t_nibbles * 32;
      if (a_nibbles)
        chunk_size *= a_nibbles;
      if (d_nibbles)
        chunk_size *= d_nibbles;
      uint32_t dig_bytes_per_chunk = chunk_size * d_nibbles / t_nibbles;
      uint32_t dig_samples_per_chunk = (d_nibbles) ? dig_bytes_per_chunk * 2 / d_nibbles : 0;
      uint32_t chunk_samples = d_nibbles ? dig_samples_per_chunk : (chunk_size * 2) / (a_nibbles);
      // total chunks in entire buffer-round to 2 since we split it in half
      uint32_t buff_chunks = (DMA_BUF_SIZE / chunk_size) & 0xFFFFFFFE;
      // round up and force power of two since we cut it in half
      uint32_t chunks_needed = ((dev.num_samples / chunk_samples) + 2) & 0xFFFFFFFE;
      Dprintf("Initial buf calcs nibbles d %d a %d t %d \n\r", d_nibbles, a_nibbles, t_nibbles);
      Dprintf("chunk size %d(bytes) samples per chunk %d total chunks in both halves %d chunks needed %d\n\r", chunk_size, chunk_samples, buff_chunks, chunks_needed);
      Dprintf("dbytes per chunk %d dig samples per chunk %d\n\r", dig_bytes_per_chunk, dig_samples_per_chunk);
      // If all of the samples we need fit in two half buffers or less then we can mask the error
      // logic that is looking for cases where we didn't send one half buffer to the host before
      // the 2nd buffer ended because we only use each half buffer once.
      mask_xfer_err = false;
      // If requested samples are smaller than the buffer, reduce the size so that the
      // transfer completes sooner.
      // Also, mask the sending of aborts if the requested number of samples fit into RAM
      // Don't do this in continuous mode as the final size is unknown
      if (dev.cont == false)
      {
        if (buff_chunks > chunks_needed)
        {
          mask_xfer_err = true;
          buff_chunks = chunks_needed;
          // Dprintf("Reduce buf chunks to %d\n\r",buff_chunks);
        }
      }
      // In mask_xfer_err mode we don't want the 2nd half to trigger back to the 1st half
      // and overwrite it's data, so set the maintenace config1's to themselves to disable chaining
      if (mask_xfer_err)
      {
        channel_config_set_chain_to(&amcfg1, amaintchan1);
        channel_config_set_chain_to(&pmcfg1, pmaintchan1);
      }
      else
      {
        channel_config_set_chain_to(&amcfg1, admachan0);
        channel_config_set_chain_to(&pmcfg1, pdmachan0);
      }
      // Give dig and analog equal fractions
      // This is the size of each half buffer in bytes
      dev.d_size = (buff_chunks * chunk_size * d_nibbles) / (t_nibbles * 2);
      dev.a_size = (buff_chunks * chunk_size * a_nibbles) / (t_nibbles * 2);
      dev.samples_per_half = chunk_samples * buff_chunks / 2;
      exp_halves = dev.cont ? -1 : dev.num_samples / dev.samples_per_half;
      if (dev.cont == false && (dev.num_samples % dev.samples_per_half))
        exp_halves++;
      Dprintf("Final sizes d %d a %d mask err %d samples per half %d exp %d\n\r", dev.d_size, dev.a_size, mask_xfer_err, dev.samples_per_half, exp_halves);

      // Clear any previous ADC over/underflow
      volatile uint32_t *adcfcs;
      adcfcs = (volatile uint32_t *)(ADC_BASE + 0x8); // ADC FCS
      *adcfcs |= 0xC00;
      // Ensure any previous dma is done
      // The cleanup loop also does this but it doesn't hurt to do it twice
      dma_channel_abort(admachan0);
      dma_channel_abort(admachan1);
      dma_channel_abort(pdmachan0);
      dma_channel_abort(pdmachan1);
      dma_channel_abort(amaintchan0);
      dma_channel_abort(amaintchan1);
      dma_channel_abort(pmaintchan0);
      dma_channel_abort(pmaintchan1);

      dev.dbuf0_start = 0;
      bytecnt = 0;
      dev.dbuf1_start = dev.d_size;
      dev.abuf0_start = dev.dbuf1_start + dev.d_size;
      dev.abuf1_start = dev.abuf0_start + dev.a_size;
      volatile uint32_t *adcdiv;
      adcdiv = (volatile uint32_t *)(ADC_BASE + 0x10); // ADC DIV
      //   Dprintf("adcdiv start %u\n\r",*adcdiv);
      //	  Dprintf("starting d_nps %u a_chan_cnt %u d_size %u a_size %u a_mask %X\n\r"
      //         ,dev.d_nps,dev.a_chan_cnt,dev.d_size,dev.a_size,dev.a_mask);
      Dprintf("start offsets d0 0x%X d1 0x%X a0 0x%X a1 0x%X samperhalf %u\n\r", dev.dbuf0_start, dev.dbuf1_start, dev.abuf0_start, dev.abuf1_start, dev.samples_per_half);
// For debug clear out initial values, but not needed in normal operation
//           for(uint32_t x=0;x<DMA_BUF_SIZE;x++){
//             capture_buf[x]=0x12;
//           }
#ifdef PIN_TEST_MODE
      for (uint32_t x = 0; x < SYSTICK_SIZE; x++)
      {
        systick_array[x] = 0x0;
      }
      systick_idx = 0;
#endif // PIN_TEST_MODE
       // Dprintf("starting data buf values 0x%X 0x%X\n\r",capture_buf[dev.dbuf0_start],capture_buf[dev.dbuf1_start]);
      uint32_t adcdivint = 48000000ULL / (dev.sample_rate * dev.a_chan_cnt);

#ifndef ADS1256_MODE // SKip onbord ADC work if ADS1256 is enabled.
      if (dev.a_chan_cnt)
      {
        adc_run(false);
        //             en, dreq_en,dreq_thresh,err_in_fifo,byte_shift to 8 bit
        adc_fifo_setup(false, true, 1, false, true);
        adc_fifo_drain();
        // Dprintf("astart cnt %u div %f\n\r",dev.a_chan_cnt,(float)adcdivint);
        // This sdk function doesn't support support the fractional divisor
        //  adc_set_clkdiv((float)(adcdivint-1));
        // The ADC divisor has some not well documented limitations.
        //-A value of 0 actually creates a 500khz sample clock.
        //-Values below 96 are clamped to 96 because a conversion takes a minimum of 96 cycles.
        // It is also import to subtract one from the desired divisor
        // because the period of ADC clock is 1+INT+FRAC/256
        // For the case of a requested 500khz clock, we would normally write
        // a divisor of 95, but doesn't give the desired result, so we use
        // the 0 value instead.
        // Fractional divisors should generally be avoided because it creates
        // skew with digital samples.
        uint8_t adc_frac_int;
        adc_frac_int = (uint8_t)(((48000000ULL % dev.sample_rate) * 256ULL) / dev.sample_rate);
        if (adcdivint <= 96)
        {
          Dprintf("adcdivint of %d below 96, aborting\n\r", adcdivint);
          dev.state = ABORTED;
          adc_aborting = true;
          *adcdiv = 0;
        }
        else
        { // adcdivint legal
          *adcdiv = ((adcdivint - 1) << 8) | adc_frac_int;
          Dprintf("adcdiv %u frac %d adcdivint %d\n\r", *adcdiv, adc_frac_int, adcdivint);
          // This is needed to clear the AINSEL so that when the round robin arbiter starts
          // we start sampling on channel 0
          adc_select_input(0);
          adc_set_round_robin(dev.a_mask & 0x7);
          //             en, dreq_en,dreq_thresh,err_in_fifo,byte_shift to 8 bit
          adc_fifo_setup(true, true, 1, false, true);
          // set adc0 to immediate trigger (but without adc_run it shouldn't start)
          // adc1 and the maintenance aren't triggered because they are chained to each other
          //                       channel, config, write_addr,                   read_addr,transfer_count,trigger)
          dma_channel_configure(admachan0, &acfg0, &(capture_buf[dev.abuf0_start]), &adc_hw->fifo, dev.a_size, true);
          dma_channel_configure(admachan1, &acfg1, &(capture_buf[dev.abuf1_start]), &adc_hw->fifo, dev.a_size, false);
          // The maintenance DMA for ADC reads the capture_buff offset value and updates the ADC DMAs with it
          amaddrs[0] = (uint32_t *)&capture_buf[dev.abuf0_start];
          amaddrs[1] = (uint32_t *)&capture_buf[dev.abuf1_start];
          // This is about as close to a common/portable address offset definition between devices to
          // find the offset of the write_addr_offset that is non-triggering.
          tmpaddr0 = DMA_BASE + DMA_CH0_WRITE_ADDR_OFFSET + (DMA_CH1_READ_ADDR_OFFSET * admachan0);
          tmpaddr1 = DMA_BASE + DMA_CH0_WRITE_ADDR_OFFSET + (DMA_CH1_READ_ADDR_OFFSET * admachan1);
          // Dprintf("ADMA Maint %X %X %X %X %X %X\n\r",amaddrs[0],amaddrs[1],tmpaddr0,tmpaddr1,&amaddrs[0],&amaddrs[1]);
          //                       channel, config, write_addr,            read_addr,transfer_count,trigger)
          dma_channel_configure(amaintchan0, &amcfg0, (uint32_t *)tmpaddr0, &amaddrs[0], 1, false);
          dma_channel_configure(amaintchan1, &amcfg1, (uint32_t *)tmpaddr1, &amaddrs[1], 1, false);
          adc_fifo_drain();
        } // adcdivint legal
      } // any analog enabled
#endif

      if (dev.d_mask)
      {
        // analyzer_init from pico-examples
        // Due to how PIO shifts in bits, if any digital channel within a group of 8 is set,
        // then all groups below it must also be set. We further restrict it in the tx_init function
        // by saying digital channel usage must be continous.
        /* pin count is restricted to 4,8,16 or 32, and pin count of 4 is only used
       Pin count is kept to a powers of 2 so that we always read a sample with a single byte/word/dword read
       for faster parsing.
          if analog is disabled and we are in D4 mode
           bits d_dma_bps   d_tx_bps
           0-4    0          1        No analog channels
           0-4    1          1        1 or more analog channels
           5-7    1          1
           8      1          2
           9-12   2          2
           13-14  2          2
           15-16  2          3
           17-21  4          3
       */
        dev.pin_count = 0;
        if (dev.d_mask & 0x0000000F)
          dev.pin_count += 4;
        if (dev.d_mask & 0x000000F0)
          dev.pin_count += 4;
        if (dev.d_mask & 0x0000FF00)
          dev.pin_count += 8;
        if (dev.d_mask & 0x0FFF0000)
          dev.pin_count += 16;
        // If 4 or less channels are enabled but ADC is also enabled, set a minimum size of 1B of PIO storage
        if ((dev.pin_count == 4) && (dev.a_chan_cnt))
        {
          dev.pin_count = 8;
        }
        d_dma_bps = dev.pin_count >> 3;
        // Dprintf("pin_count %d\n\r",dev.pin_count);
        uint16_t capture_prog_instr;
        capture_prog_instr = pio_encode_in(pio_pins, dev.pin_count);
        // Dprintf("capture_prog_instr 0x%X\n\r",capture_prog_instr);
        struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1};
        uint offset = pio_add_program(pio, &capture_prog);
        // Configure state machine to loop over this `in` instruction forever,
        // with autopush enabled.
        pio_sm_config c = pio_get_default_sm_config();
#ifdef DIG_26_MODE
        sm_config_set_in_pins(&c, 0); // start at GPIO0 since uart isn't used
#elif DIG_32_MODE
        sm_config_set_in_pins(&c, 0); // start at GPIO0 since uart isn't used
#else
        sm_config_set_in_pins(&c, 2); // start at GPIO2 (keep 0 and 1 for uart)
#endif
        sm_config_set_wrap(&c, offset, offset);
        uint16_t div_int;
        uint8_t frac_int;
        div_int = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000 / dev.sample_rate;
        if (div_int < 1)
          div_int = 1;
        frac_int = (uint8_t)(((frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000 % dev.sample_rate) * 256ULL) / dev.sample_rate);
        Dprintf("PIO sample clk %u divint %d divfrac %d \n\r", dev.sample_rate, div_int, frac_int);
        // Unlike the ADC, the PIO int divisor does not have to subtract 1.
        // Frequency=sysclkfreq/(CLKDIV_INT+CLKDIV_FRAC/256)
        sm_config_set_clkdiv_int_frac(&c, div_int, frac_int);

        // Since we enable digital channels in groups of 4, we always get 32 bit words
        sm_config_set_in_shift(&c, true, true, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        pio_sm_init(pio, piosm, offset, &c);
        // Analyzer arm from pico examples
        pio_sm_set_enabled(pio, piosm, false); // clear the enabled bit
        // XOR the shiftctrl field with PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
        // Do it twice to restore the value
        pio_sm_clear_fifos(pio, piosm);
        // write the restart bit of PIO_CTRL
        pio_sm_restart(pio, piosm);
        // Since PIO transfers 32 bit values but DMA transfers 8, the d_size is divided by 4.
        // Dprintf("DMABufCfg d0_start %d d1_start %d d_size %d\n\r",dev.dbuf0_start,dev.dbuf1_start,dev.d_size);
        //                     number    config   buffer target                  piosm          xfer size  trigger
        dma_channel_configure(pdmachan0, &pcfg0, &(capture_buf[dev.dbuf0_start]), &pio->rxf[piosm], dev.d_size >> 2, true);
        dma_channel_configure(pdmachan1, &pcfg1, &(capture_buf[dev.dbuf1_start]), &pio->rxf[piosm], dev.d_size >> 2, false);
        // The maintenance DMA for PIO reads the capture_buff offset value and updates the ADC DMAs with it
        pmaddrs[0] = (uint32_t *)&capture_buf[dev.dbuf0_start];
        pmaddrs[1] = (uint32_t *)&capture_buf[dev.dbuf1_start];
        // This is about as close to a common/portable address offset definition between devices to
        // find the offset of the write_addr_offset that is non-triggering.
        tmpaddr0 = DMA_BASE + DMA_CH0_WRITE_ADDR_OFFSET + (DMA_CH1_READ_ADDR_OFFSET * pdmachan0);
        tmpaddr1 = DMA_BASE + DMA_CH0_WRITE_ADDR_OFFSET + (DMA_CH1_READ_ADDR_OFFSET * pdmachan1);
        // Dprintf("PDMA Maint %X %X %X %X %X %X\n\r",pmaddrs[0],pmaddrs[1],tmpaddr0,tmpaddr1,&pmaddrs[0],&pmaddrs[1]);
        //                       channel, config, write_addr, read_addr,transfer_count,trigger)
        dma_channel_configure(pmaintchan0, &pmcfg0, (uint32_t *)tmpaddr0, &pmaddrs[0], 1, false);
        dma_channel_configure(pmaintchan1, &pmcfg1, (uint32_t *)tmpaddr1, &pmaddrs[1], 1, false);

      } // if dev.d_mask
      // Dprintf("LVL0mask 0x%X\n\r",dev.lvl0mask);
      // Dprintf("LVL1mask 0x%X\n\r",dev.lvl1mask);
      // Dprintf("risemask 0x%X\n\r",dev.risemask);
      // Dprintf("fallmask 0x%X\n\r",dev.fallmask);
      // Dprintf("edgemask 0x%X\n\r",dev.chgmask);

      // Dprintf("capture_buf base %p \n\r",capture_buf);
      // Dprintf("capture_buf dig %p %p \n\r",&(capture_buf[dev.dbuf0_start]),&(capture_buf[dev.dbuf1_start]));
      // Dprintf("capture_buf analog %p %p\n\r",&(capture_buf[dev.abuf0_start]),&(capture_buf[dev.abuf1_start]));
      // Dprintf("PIOSMCLKDIV 0x%X\n\r",*pio0sm0clkdiv);

      // Dprintf("PIO ctrl 0x%X fstts 0x%X dbg 0x%X lvl 0x%X\n\r",*pioctrl,*piofstts,*piodbg,*pioflvl);
      // Dprintf("DMA channel assignments a %d %d d %d %d\n\r",admachan0,admachan1,pdmachan0,pdmachan1);
      if (!adc_aborting)
      {
        // Clear any pending interrupts
        // All dma interrupts go through a common handler so that we can check for
        // overflows etc.
        if (dev.d_mask > 0)
        {
          dma_channel_set_irq0_enabled(pdmachan0, true);
          dma_channel_set_irq0_enabled(pdmachan1, true);
        }
#ifndef ADS1256_MODE
        if (dev.a_chan_cnt > 0)
        {
          dma_channel_set_irq0_enabled(admachan0, true);
          dma_channel_set_irq0_enabled(admachan1, true);
        }
#endif

        h0intmask = 0;
        h1intmask = 0;
        currintmask = 0;

        if (dev.d_mask)
        {
          h0intmask |= 1 << pdmachan0;
          h1intmask |= 1 << pdmachan1;
        }
#ifndef ADS1256_MODE
        if (dev.a_chan_cnt)
        {
          h0intmask |= 1 << admachan0;
          h1intmask |= 1 << admachan1;
        }
#endif
        dma_halves = 0;
        num_halves = 0;
        sho_cnt = 0;
        tx_cnt = 0;
        acnt = 0;
        bcnt = 0;
        bytecnt = 0;
        dcnt = 0;
        ecnt = 0;

        // Dprintf("XY %X %X %X %X\n",dev.d_mask,dev.a_chan_cnt,h0intmask,h1intmask);
        // Enable logic and analog close together for best possible alignment
        // warning - do not put printfs after this line as they will corrupt time measurement and sample start
        tstart = time_us_32();
        dev.state = SENDING;

        // Pending Interrupts must be cleared immediately before enabling the IRQ
        dma_hw->ints0 = dma_hw->ints0;
        irq_set_enabled(DMA_IRQ_0, true);
        irq_set_exclusive_handler(DMA_IRQ_0, dma_int_handler);

#ifdef BASE_MODE
        adc_run(true); // enable free run sample mode
#endif
        pio_sm_set_enabled(pio, piosm, true);
      } // if ~adcaborting
    } // if dev.sending and not started
    // Send sample data
    send_half();
// Drain all uart rxs (only tx is used for debug) if uart rx is not drained
// it can cause code in the sdk to lock up serial CDC. These are rare noise/reset events
// and thus not checked when dev.started to ensure the maintenance loop runs as fast
// as possible.
#if (UART_EN == 1)
    if (dev.state == IDLE)
    {
      while (uart_is_readable_within_us(uart0, 0))
      {
        uartch = uart_getc(uart0);
        Dprintf("Uart Char %d\n\r", uartch);
      }
    }
#endif
    // look for commands on usb cdc
    ccnt = time_us_32();
    usbintin = getchar_timeout_us(0);
    dcnt = time_us_32();
    // The '+' is the only character we track during normal sampling because it can end
    // a continuous trace or an aborted condition.
    // A reset '*' should only be seen after we have completed normally or hit an error condition.
    // The plus ends an aborted loop, is ignored by IDLE, and sends started to IDLE.
    // In all other cases the effect is not immediate and it's up to the interrupt handler
    // or send_half to make use of it.
    if (usbintin == '+')
    {
      // Dprintf("USB plus\n\r");
      if (dev.state == ABORTED)
      {
        // Clear abort so we stop sending "!"
        Dprintf("Plus ends abort\n\r");
        dev.state == IDLE;
      }
      else if (dev.state == IDLE)
      {
        Dprintf("Plus in idle ignored\n\r");
      }
      else if (dev.state == STARTED)
      {
        Dprintf("Plus ends started");
        dev.state == IDLE;
      }
      else
      {
        Dprintf("usb_plus set\n\r");
        dev.usb_plus = true;
      }
    }
    // send_resp is set to true but not processed immediately so we can get back to send_half ASAP
    else if (usbintin >= 0)
    {
      bcnt++;
      if (process_char(&dev, (char)usbintin))
      {
        send_resp = true;
      }
    }

    // The libsigrok processing of aborts is not clean, and may try to report the "!" as a bad rle value.
    // Emperically it seems that waiting for a long delay to ensure the host has drained it's Rx buffere
    // and then sending 3 "!" is the most reliable way to get it to exit.
    // As an additional failsafe, a final bytecnt with a count of 0 is sent.
    // In forced test mode we also don't get the usb plus, so the forced exit on abort covers that as well
    if (dev.state == ABORTED)
    {
      Dprintf("sending abort! ftm %d num_halves %d dma_halves %d sho cnt %d tx_cnt %d\n\r",
              forced_test_mode_run, num_halves, dma_halves, sho_cnt, tx_cnt);
      sleep_ms(1000);
      my_stdio_usb_out_chars("!!!", 3);
      sleep_ms(1000);
      my_stdio_usb_out_chars("$0+", 3);
      sleep_ms(1000);
      dev.state = IDLE;
    }
    // Once we reach SAMPLES_SENT, send the final byte count to ensure no bytes were lost
    if (dev.state == SAMPLES_SENT)
    {
      // The end of sequence byte_cnt uses a "$<byte_cnt>+" format.
      char brsp[16];
      // Give the host time to finish processing samples so that the bytecnt
      // isn't dropped on the wire
      sleep_us(10000);
      Dprintf("Cleanup bytecnt %d\n\r", bytecnt);
      sprintf(brsp, "$%d%c", bytecnt, '+');
      puts_raw(brsp);
      // Print out debug information after completing, rather than before so that it doesn't
      // delay the start of a capture
      Dprintf("Complete: SRate %d NSmp %d NHalves %d\n\r", dev.sample_rate, dev.num_samples, num_halves);
      Dprintf("Cont %d bcnt %d\n\r", dev.cont, bytecnt);
      Dprintf("DMsk 0x%X AMsk 0x%X\n\r", dev.d_mask, dev.a_mask);
      Dprintf("Half buffers exp %d DMA %d Sent %d sampperhalf %d\n\r", exp_halves, dma_halves, num_halves, dev.samples_per_half);
      dev.state = IDLE;
#ifdef PIN_TEST_MODE
      for (int y = 0; y < SYSTICK_PRINT; y++)
      {
        int delta;
        if (y == 0)
        {
          delta = 0;
        }
        else if (systick_array[y] == 0)
        {
          delta = 0;
        }
        else
        {
          delta = systick_array[y] - systick_array[y - 1];
        }
        // The callback should be every 100us, so print out any large anomolies
        // as a warning that the generated pattern may be stretched.
        if (delta > 120)
        {
          Dprintf("*****Systick Anomoly ST%3d %d %d\n\r***", y, systick_array[y], delta);
        }
      }
#endif // PIN_TEST_MODE
    }
    // Since there are so many FSM arcs, it's safest to just continually clear state
    // if we aren't sending
    if (dev.state == IDLE)
    {
#ifdef ADS1256_MODE
      ads1256_core1_run = false;
#endif
      // forced_test_mode is really a one shot deal as there is no way to restart it.
      // Exit the mode so that host accesses will work normally
      forced_test_mode_run = false;
#ifdef BASE_MODE
      adc_run(false);
      adc_fifo_drain();
#endif
      pio_sm_restart(pio, piosm);
      pio_sm_set_enabled(pio, piosm, false);
      pio_sm_clear_fifos(pio, piosm);
      pio_clear_instruction_memory(pio);
      dma_channel_abort(admachan0);
      dma_channel_abort(admachan1);
      dma_channel_abort(pdmachan0);
      dma_channel_abort(pdmachan1);
      dma_channel_abort(amaintchan0);
      dma_channel_abort(amaintchan1);
      dma_channel_abort(pmaintchan0);
      dma_channel_abort(pmaintchan1);
      dma_channel_set_irq0_enabled(admachan0, false);
      dma_channel_set_irq0_enabled(pdmachan0, false);
      dma_channel_set_irq0_enabled(admachan1, false);
      dma_channel_set_irq0_enabled(pdmachan1, false);
      // clear any pended dma interrupts
      dma_hw->ints0 = dma_hw->ints0;
      irq_set_enabled(DMA_IRQ_0, false);

      num_halves = 0;
      dma_halves = 0;
      exp_halves = 0;
      currintmask = 0;
      dev.usb_plus = false;
    } // if IDLE
  } // while(1)

} // main
// Depracated trigger logic
// This HW based trigger which should be part of send slices was tested enough to confirm the
// trigger value worked, however it
// was not fully implemented because the RP2040 wasn't able to perform the trigger detection and
// memory buffer management to support sample rates that were substantially higher than the
// stream rates across USB.  Thus there wasn't a compelling reason to have it.
// It's left as an example as to how the masks could be used.
/*
//   uint32_t tlval; Trigger last val
//   uint32_t all_mask=d->lvl0mask | d->lvl1mask| d->risemask | d->fallmask | d->chgmask;


       if(d->triggered==false) {
         uint32_t matches=0;
         matches|=(~cval & d->lvl0mask);
         matches|=(cval & d->lvl1mask);
         if(d->notfirst){
           matches|=(cval & ~tlval & d->risemask);
           matches|=(~cval & tlval & d->fallmask);
           matches|=(~cval & tlval & d->chgmask);
         }
         if(matches==all_mask){
     //Dprintf("Triggered c 0x%X l 0x%X \n\r",cval,tlval);
           d->triggered=true;
           //This sends the last val on a trigger because SW based trigger on the host needs to see its
           //value so that rising/falling/edge triggeers will fire there too.
           lbyte=0;
           for(char b=0;b < d->d_tx_bps;b++){
              cbyte=tlval&0xFF;
              txbuf[txbufidx]=(cbyte<<b)|lbyte|0x80;
              lbyte=cbyte>>(7-b);
              tlval>>=8;
        txbufidx++;
           } //for b
         }//matches==all_mask
         d->notfirst=true;
       }
       if(d->triggered){
             //Transmit samples if we have already triggered.
        }

//save trigger last value to support rising/falling/change values
//      tlval=lval;
End of depracated trigger logic
*/
