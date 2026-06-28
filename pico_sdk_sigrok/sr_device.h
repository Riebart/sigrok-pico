#ifndef SR_DEVICE_H
#define SR_DEVICE_H
#include <stdint.h>
#include <stdbool.h>

// Pin usages
///////////////////////////////////
// Baseline mode -21 digital, 3 analog
// GP0,1 is debug uart TX, RX (but rx not used)
// GP2-GP22 are digital inputs
// GP23 controls power supply modes and is not a board I/O
// GP24 is a power sense and not a board I/O
// GP25 controlls the LED and is not a board I/O
// GP26-28 are ADC. (Only 7 bit accuracy supported)
//////////////////////////////////
// Digital 26 Mode
// GP0-GP22 are digital inputs
// GP23 controls power supply modes and is not a board I/O
// GP24 is a power sense and not a board I/O
// GP25 controlls the LED and is not a board I/O
// GP26-28 are digital.
////////////////////////////
// Digital 32 Mode
// Use GPIO 0..31 for boards that export those pins
// No uart, no ADC, LED, power supply control or power sense etc.
/////////////////////////
// ADS1256 SPI Mode (PICO_MODE 3)
// GP0-GP15  : 16 digital inputs (D0-D15)
// GP16      : SPI0 MISO  (ADS1256 DOUT)
// GP17      : SPI0 CS    (ADS1256 /CS) - active low, GPIO-controlled
// GP18      : SPI0 SCLK  (ADS1256 SCLK)
// GP19      : SPI0 MOSI  (ADS1256 DIN)
// GP20      : DRDY input (ADS1256 /DRDY) - active low
// GP21-GP22 : reserved/unused
// GP23      : power supply mode control (not board I/O)
// GP24      : power sense (not board I/O)
// GP25      : LED (not board I/O)
// GP26-GP28 : not used (freed from onboard ADC)
// 8 single-ended analog channels AIN0-AIN7 vs AINCOM
/////////////////////////
// Note: In the wireless versions, GPIO23-25 control the wifi chip, 23 and 24
// aren't available in the PICO, and 25 controls the LED. So while the LED is lost,
// there is no change in available channels for sampling.
#define PICO_MODE 3 // 0 is baseline, 1 is digital 26, 2 is digital 32, 3 is ADS1256 SPI
// WARNING: USE PIN_TEST_MODE with extreme caution!!!!
// If set, treat the inputs (A&D) to be outputs so that the device can drive values for
// turn-on testing.  Enabling this allows all modes to be tested without having to drive
// test patterns on the chip.  But it turns what are normally inputs to outputs and thus
// can cause drive fights if any drivers are connected.
// #define PIN_TEST_MODE 1
#undef BASE_MODE
#undef DIG_26_MODE
#undef DIG_32_MODE
#undef ADS1256_MODE
#undef HAS_LED
#if PICO_MODE == 0 // Baseline
#define BASE_MODE 1
#define NUM_A_CHAN 3  // number of analog channels
#define NUM_D_CHAN 21 // number of digital channels
// Note: GPIO_D_MASK is relative to the pins of the chip, whereas the
// MEM_D_MASK is relative to the value written in memory, those may be different depending
// on how data is shifted from the GPIOs into memory.
#define GPIO_D_MASK 0x7FFFFC // Mask of bits for digital inputs
#define UART_EN 1
// Since this mode has all digital inputs contigous, the upper mask isn't needed.
#define MEM_D_MASK_L 0x007FFFFF // lower mask of bits for digital inputs
#define MEM_D_MASK_U 0x0        // upper mask of bits for digital inputs
#define PIN_TEST_MASK 0x1C7FFFFE
#define HAS_LED 1
#elif PICO_MODE == 1 // Digital 26
#define DIG_26_MODE 1
#define NUM_A_CHAN 0            // number of analog channels
#define NUM_D_CHAN 26           // number of digital channels
#define GPIO_D_MASK 0x1C7FFFFF  // Mask of bits for digital inputs
#define MEM_D_MASK_L 0x007FFFFF // lower mask of bits for digital inputs
#define MEM_D_MASK_U 0x1C000000 // upper mask of bits for digital inputs
#define UART_EN 0
#define PIN_TEST_MASK 0x1C7FFFFF
#define HAS_LED 1
// Note: The RP2040 only has GPIOs 0-29.
// The RP2350 has GPIOs 30 and above, but only in the QFN-80 packeage.
#elif PICO_MODE == 2 // Digital 32
// #define BASE_MODE 0
// #define DIG_26_MODE 0
#define DIG_32_MODE 1
#define NUM_A_CHAN 0            // number of analog channels
#define NUM_D_CHAN 32           // number of digital channels
#define GPIO_D_MASK 0xFFFFFFFF  // Mask of bits for digital inputs
#define MEM_D_MASK_L 0xFFFFFFFF // lower mask of bits for digital inputs
#define MEM_D_MASK_U 0x00000000 // upper mask of bits for digital inputs
#define UART_EN 0
#define PIN_TEST_MASK 0xFFFFFFFF
#elif PICO_MODE == 3 // ADS1256 SPI ADC mode
#define ADS1256_MODE 1
#define NUM_A_CHAN 8            // AIN0-AIN7 single-ended vs AINCOM
#define NUM_D_CHAN 16           // GP0-GP15
#define GPIO_D_MASK 0x0000FFFF  // Mask of bits for digital inputs (GP0-GP15)
#define MEM_D_MASK_L 0x0000FFFF // lower mask of bits for digital inputs
#define MEM_D_MASK_U 0x00000000 // upper mask of bits for digital inputs
#define UART_EN 0
#define PIN_TEST_MASK 0x0000FFFF
#define HAS_LED 1

// ---------- SPI0/1 / ADS1256 pin assignments ----------
#ifdef ADS1256_SPI1         // use SPI1
#define ADS1256_PIN_MISO 12 // SPI0 RX  -> ADS1256 DOUT
#define ADS1256_PIN_CS 13   // GPIO CS  -> ADS1256 /CS  (active-low, SW-controlled)
#define ADS1256_PIN_SCLK 14 // SPI0 SCK -> ADS1256 SCLK
#define ADS1256_PIN_MOSI 15 // SPI0 TX  -> ADS1256 DIN
#define ADS1256_PIN_DRDY 29 // GPIO in  -> ADS1256 /DRDY (active-low)
#else                       // #ifdef ADS1256_SPI0 use SPI0
#define ADS1256_PIN_MISO 0  // SPI0 RX  -> ADS1256 DOUT
#define ADS1256_PIN_CS 1    // GPIO CS  -> ADS1256 /CS  (active-low, SW-controlled)
#define ADS1256_PIN_SCLK 2  // SPI0 SCK -> ADS1256 SCLK
#define ADS1256_PIN_MOSI 3  // SPI0 TX  -> ADS1256 DIN
#define ADS1256_PIN_DRDY 4  // GPIO in  -> ADS1256 /DRDY (active-low)
#endif
// ---------- ADS1256 hardware configuration ----------
// These may be overridden at compile time via -DADS1256_PGA_GAIN=x etc.
// PGA gain: 1, 2, 4, 8, 16, 32, or 64
#ifndef ADS1256_PGA_GAIN
#define ADS1256_PGA_GAIN 1
#endif
// Input buffer: 1 = enabled, 0 = disabled
// When enabled, AIN must be held within AVDD-2V. Reduces noise significantly.
#ifndef ADS1256_BUF_ENABLE
#define ADS1256_BUF_ENABLE 1
#endif
// CLKIN frequency in Hz (crystal on ADS1256 module, typically 7.68 MHz)
#ifndef ADS1256_CLKIN_HZ
#define ADS1256_CLKIN_HZ 7680000
#endif
// DRATE register value. 0xA1 = 1000 SPS (see ADS1256 datasheet Table 13)
// At 1000 SPS: tSETTLE = 1ms, well within USB bandwidth.
// Other options: 0xF0=30000SPS, 0xE0=15000SPS, 0xD0=7500SPS, 0xC0=3750SPS,
//                0xB0=2000SPS, 0xA1=1000SPS, 0x92=500SPS, 0x82=100SPS,
//                0x72=60SPS,   0x63=50SPS,   0x53=30SPS,  0x43=25SPS,
//                0x33=15SPS,   0x23=10SPS,   0x13=5SPS,   0x03=2.5SPS
#ifndef ADS1256_DRATE_REG
#define ADS1256_DRATE_REG 0xA1
#endif

// ---------- Wire protocol: analog encoding ----------
// ADS1256 produces a 24-bit two's-complement result.
// For single-ended 0..VREF measurements only the positive half is used,
// giving an effective unsigned range of 0x000000..0x7FFFFF (23 useful bits).
// We right-shift by 3 to get 21 significant bits, then pack into
// ADS1256_A_BYTES=3 wire bytes of 7 bits each (matching the libsigrok
// raspberrypi-pico driver's multi-byte analog decoding with a_size=3).
#define ADS1256_A_BYTES 3                                       // wire bytes per analog sample (asize in ident string)
#define ADS1256_A_RSHIFT 3                                      // right-shift applied before encoding

// ---------- Wire protocol: scale and offset (integer microvolts) ----------
// The libsigrok raspberrypi-pico driver sends the 'a' command to retrieve
// scale (uV/count) and offset (uV) for each channel. Both are integers.
// Derivation (single-ended, VREF=2.5V):
//   Full-scale positive code  = 0x7FFFFF = 8388607
//   After right-shift by 3    = 8388607 >> 3 = 1048575  (2^20 - 1)
//   Voltage at full scale     = VREF / PGA_GAIN = 2500000 uV / ADS1256_PGA_GAIN
//   uV per count              = (2500000 / ADS1256_PGA_GAIN) / 1048575
//   For PGA=1: 2500000/1048575 ≈ 2  uV/count  (rounds to 2)
//   For PGA=2: 1250000/1048575 ≈ 1  uV/count
//   Offset: 0 uV (single-ended, AINCOM=AGND)
//
// Integer arithmetic: avoid floating point at compile time.
// SCALE_UV = (2500000 / ADS1256_PGA_GAIN) / ((1 << (24 - ADS1256_A_RSHIFT - 1)))
// denominator = 2^(24-3-1) = 2^20 = 1048576  (use 1048576 not 1048575 for integer math)
#define ADS1256_VREF_UV 2500000L                                // VREF in microvolts (2.5 V)
#define ADS1256_SCALE_DENOM (1L << (24 - ADS1256_A_RSHIFT - 1)) // 1048576
#define ADS1256_SCALE_UV ((ADS1256_VREF_UV / ADS1256_PGA_GAIN) / ADS1256_SCALE_DENOM)
// Offset: 0 for single-ended (AINCOM=AGND). Buffer state does not change
// the full-scale span when VREF is fixed externally.
#define ADS1256_OFFSET_UV 0L

// ---------- SPI clock ----------
// ADS1256 requires SCLK <= CLKIN/4. With CLKIN=7.68MHz: max SCLK=1.92MHz.
// Use 1.9 MHz to stay safely below the limit.
#define ADS1256_SPI_HZ 1900000

// ---------- Ring buffer for single-channel RDATAC mode ----------
// Sized as a multiple of ADS1256_A_BYTES so encoded samples are never split.
// Must fit in RP2040 SRAM alongside capture buffers; keep at 9000 encoded
// bytes = 3000 samples. Adjust down if link is saturated.
#define ADS1256_RING_BYTES 9000 // must be multiple of ADS1256_A_BYTES

#endif

// These two enable debug print outs of D4 generation, D4_DBG2 is higher verbosity
// #define D4_DBG 1
// #define D4_DBG2 2

// Storage size of the DMA buffer.  The buffer is split into two halves so that when the first
// buffer fills we can send the trace data serially while the other buffer is DMA'd into.
// In ADS1256 mode this buffer is NOT used for analog (ring buffer used instead),
// but IS still used for the digital capture PIO/DMA path.
#ifdef PICO_RP2350
#define DMA_BUF_SIZE 476000 // add the full 256KB increase
#else
#define DMA_BUF_SIZE 220000
#endif
// The size of the buffer sent to the CDC serial
// The TUD CDC buffer is only 256B so it doesn't help to have more than this.
#define TX_BUF_SIZE 260
// Setting to default value of Raspberry PI debug probe of 115200
#define UART_BAUD 115200 // 921600
// This sets the point which we will send data from the txbuf to the usb cdc.
#define TX_BUF_THRESH 20

typedef enum
{
  IDLE = 0,         // initial and ending condition, also cleanup variables used when not idle
  STARTED = 1,      // the host has sent a command to start sending samples
  SENDING = 2,      // the dma engines etc are configured and running
  DMA_DONE = 3,     // DMA engine has sent all expected loop and is disabled
  SAMPLES_SENT = 4, // all samples have been sent to the host
  ABORTED = 5       // an error , usually DMA buffer overflow, has occured
} dev_state;
typedef struct
{
  uint32_t sample_rate;
  uint32_t num_samples;
  uint32_t a_mask, d_mask;
  // number of samples for one of the 4 dma target arrays.  While this normally is related to the
  // half buffer sizes, in the optimized sending mode the value is temporarily overrided to adjust
  // the number of samples sent by the send_slice* functions.
  uint32_t samples_per_half;
  uint8_t a_chan_cnt; // count of enabled analog channels
  uint8_t d_chan_cnt; // count of enabled digital channels
  uint8_t d_tx_bps;   // Digital Transmit bytes per slice
  // Pins sampled by the PIO - 4,8,16 or 32
  uint8_t pin_count;
  uint8_t d_nps; // digital nibbles per slice from a PIO/DMA perspective.
  uint32_t scnt; // number of samples sent
  char cmdstrptr;
  char cmdstr[20];                                             // used for parsing input
  uint32_t d_size, a_size;                                     // size of each of the two data buffers for each of a & d
  uint32_t dbuf0_start, dbuf1_start, abuf0_start, abuf1_start; // starting memory pointers of adc buffers
  char rspstr[20];
  // mark key control variables voltatile since multiple cores might access them
  volatile dev_state state;
  volatile bool cont;
  // a "+" was received from the host telling us to stop
  volatile bool usb_plus;
} sr_device_t;

// Send to debug uart
int Dprintf(const char *fmt, ...);

// Process incoming character stream
int process_char(sr_device_t *d, char charin);

// reset as part of init, or on a completed send
void reset(sr_device_t *d);

// initial post reset state
void init(sr_device_t *d);

// Initialize the tx buffer
void tx_init(sr_device_t *d);

// Process incoming character stream
// Return 1 if the device rspstr has a response to send to host
// Be sure that rspstr does not have \n  or \r.
int process_char(sr_device_t *d, char charin);

#endif /* SR_DEVICE_H */
