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
// 8 differential analog channels via ADS1256 24-bit ADC over SPI
// GP16 = SPI0 MISO (DOUT on ADS1256)
// GP17 = SPI0 CS   (active-low chip select)
// GP18 = SPI0 SCK
// GP19 = SPI0 MOSI (DIN on ADS1256)
// GP20 = DRDY (active-low data-ready input from ADS1256)
// GP0,1 remain uart TX/RX for debug
// No PIO digital capture in this mode
/////////////////////////
// Note: In the wireless versions, GPIO23-25 control the wifi chip, 23 and 24
//aren't available in the PICO, and 25 controls the LED. So while the LED is lost,
//there is no change in available channels for sampling.
#define PICO_MODE 3 //0 is baseline, 1 is digital 26, 2 is digital 32, 3 is ADS1256 SPI
//WARNING: USE PIN_TEST_MODE with extreme caution!!!!
//If set, treat the inputs (A&D) to be outputs so that the device can drive values for
//turn-on testing.  Enabling this allows all modes to be tested without having to drive
//test patterns on the chip.  But it turns what are normally inputs to outputs and thus
//can cause drive fights if any drivers are connected.
//#define PIN_TEST_MODE 1
#undef BASE_MODE
#undef DIG_26_MODE
#undef DIG_32_MODE
#undef ADS1256_MODE
#undef HAS_LED
#if PICO_MODE == 0 //Baseline
  #define BASE_MODE 1
  #define NUM_A_CHAN 3 // number of analog channels
  #define NUM_D_CHAN 21 // number of digital channels
  //Note: GPIO_D_MASK is relative to the pins of the chip, whereas the
  //MEM_D_MASK is relative to the value written in memory, those may be different depending
  //on how data is shifted from the GPIOs into memory.
  #define GPIO_D_MASK 0x7FFFFC  //Mask of bits for digital inputs
  #define UART_EN 1
  //Since this mode has all digital inputs contigous, the upper mask isn't needed.
  #define MEM_D_MASK_L 0x007FFFFF  //lower mask of bits for digital inputs
  #define MEM_D_MASK_U 0x0  //upper mask of bits for digital inputs
  #define PIN_TEST_MASK 0x1C7FFFFE
  #define HAS_LED 1
#elif PICO_MODE == 1  //Digital 26
  #define DIG_26_MODE 1
  #define NUM_A_CHAN 0 // number of analog channels
  #define NUM_D_CHAN 26 // number of digital channels
  #define GPIO_D_MASK 0x1C7FFFFF  //Mask of bits for digital inputs
  #define MEM_D_MASK_L 0x007FFFFF  //lower mask of bits for digital inputs
  #define MEM_D_MASK_U 0x1C000000  //upper mask of bits for digital inputs
  #define UART_EN 0
  #define PIN_TEST_MASK 0x1C7FFFFF
  #define HAS_LED 1
//Note: The RP2040 only has GPIOs 0-29.
//The RP2350 has GPIOs 30 and above, but only in the QFN-80 packeage.
#elif PICO_MODE==2  //Digital 32
 // #define BASE_MODE 0
 // #define DIG_26_MODE 0
  #define DIG_32_MODE 1
  #define NUM_A_CHAN 0 // number of analog channels
  #define NUM_D_CHAN 32 // number of digital channels
  #define GPIO_D_MASK   0xFFFFFFFF  //Mask of bits for digital inputs
  #define MEM_D_MASK_L 0xFFFFFFFF  //lower mask of bits for digital inputs
  #define MEM_D_MASK_U 0x00000000  //upper mask of bits for digital inputs
  #define UART_EN 0
  #define PIN_TEST_MASK 0xFFFFFFFF
#elif PICO_MODE==3  //ADS1256 SPI 8-channel ADC
  #define ADS1256_MODE 1
  #define NUM_A_CHAN 8 // 8 single-ended (or 4 differential pair) channels
  #define NUM_D_CHAN 0 // no PIO digital capture in this mode
  #define GPIO_D_MASK 0x0  //no digital GPIO capture pins
  #define UART_EN 1
  #define HAS_LED 1
  // SPI0 pin assignments for ADS1256
  #define ADS1256_SPI_PORT spi0
  #define ADS1256_PIN_MISO 16
  #define ADS1256_PIN_CS   17
  #define ADS1256_PIN_SCK  18
  #define ADS1256_PIN_MOSI 19
  #define ADS1256_PIN_DRDY 20
  // ADS1256 SPI clock: max 1.92 MHz at 5V AVDD, use 1 MHz for margin
  #define ADS1256_SPI_BAUD 1000000
  // ADS1256 register addresses
  #define ADS1256_REG_STATUS 0x00
  #define ADS1256_REG_MUX    0x01
  #define ADS1256_REG_ADCON  0x02
  #define ADS1256_REG_DRATE  0x03
  // ADS1256 commands
  #define ADS1256_CMD_WAKEUP  0x00
  #define ADS1256_CMD_RDATA   0x01
  #define ADS1256_CMD_RDATAC  0x03
  #define ADS1256_CMD_SDATAC  0x0F
  #define ADS1256_CMD_RREG    0x10
  #define ADS1256_CMD_WREG    0x50
  #define ADS1256_CMD_SELFCAL 0xF0
  #define ADS1256_CMD_SYNC    0xFC
  #define ADS1256_CMD_STANDBY 0xFD
  #define ADS1256_CMD_RESET   0xFE
  // ADS1256 data rate register values (samples/sec)
  #define ADS1256_DRATE_30000SPS 0xF0
  #define ADS1256_DRATE_15000SPS 0xE0
  #define ADS1256_DRATE_7500SPS  0xD0
  #define ADS1256_DRATE_3750SPS  0xC0
  #define ADS1256_DRATE_2000SPS  0xB0
  #define ADS1256_DRATE_1000SPS  0xA1
  #define ADS1256_DRATE_500SPS   0x92
  #define ADS1256_DRATE_100SPS   0x82
  #define ADS1256_DRATE_60SPS    0x72
  #define ADS1256_DRATE_50SPS    0x63
  #define ADS1256_DRATE_30SPS    0x53
  #define ADS1256_DRATE_25SPS    0x43
  #define ADS1256_DRATE_15SPS    0x33
  #define ADS1256_DRATE_10SPS    0x23
  #define ADS1256_DRATE_5SPS     0x13
  #define ADS1256_DRATE_2_5SPS   0x03
  // Scale and offset for 'a' command response.
  // ADS1256 default gain=1, VREF=2.5V, so full-scale = +-2.5V = 5V span.
  // The wire protocol packs 8 ADC bits into 7 wire bits (>>1, |0x80).
  // libsigrok reconstructs: voltage = raw_7bit * scale_uV + offset_uV
  // With 7 wire bits (128 levels) across 5V: scale = 5000000/128 = 39063 uV/LSB
  // Offset = -2500000 uV so that mid-scale (64) decodes to 0V.
  #define ADS1256_SCALE_UV  39063
  #define ADS1256_OFFSET_UV -2500000
#endif
//These two enable debug print outs of D4 generation, D4_DBG2 is higher verbosity
//#define D4_DBG 1
//#define D4_DBG2 2

// Storage size of the DMA buffer.  The buffer is split into two halves so that when the first
// buffer fills we can send the trace data serially while the other buffer is DMA'd into
#ifdef PICO_RP2350
 #define DMA_BUF_SIZE 476000 //add the full 256KB increase
#else
 #define DMA_BUF_SIZE 220000
#endif
// The size of the buffer sent to the CDC serial
// The TUD CDC buffer is only 256B so it doesn't help to have more than this.
#define TX_BUF_SIZE 260
//Setting to default value of Raspberry PI debug probe of 115200
#define UART_BAUD 115200 //921600
// This sets the point which we will send data from the txbuf to the usb cdc.
// For the 5-21 channel RLE it must leave a spare ~83 entries to cover the case where
// a new long steady input comes after deciding to not send a sample.
//(Assuming 128KB samples per half, a max rle value of 1568 we can get
//   256*1024/2/1568=83 max length rles on a steady input).
// Other than that the value is not very specific because the usb tub code
// implement a 256 entry fifo that queues things up and sends max length 64B transactions
// 20 is arbitrarly picked to ensure that if we have even a little we send it so that
// at least something goes across the link.
#define TX_BUF_THRESH 20
typedef enum  {IDLE = 0, //initial and ending condition, also cleanup variables used when not idle
              STARTED = 1, //the host has sent a command to start sending samples
              SENDING = 2, //the dma engines etc are configured and running
              DMA_DONE = 3, //DMA engine has sent all expected loop and is disabled
              SAMPLES_SENT = 4, //all samples have been sent to the host
              ABORTED = 5 //an error , usually DMA buffer overflow, has occured
            } dev_state;
typedef struct
{
   uint32_t sample_rate;
   uint32_t num_samples;
   uint32_t a_mask, d_mask;
   // number of samples for one of the 4 dma target arrays.  While this normally is related to the
   //half buffer sizes, in the optimized sending mode the value is temporarily overrided to adjust
   //the number of samples sent by the send_slice* functions.
   uint32_t samples_per_half;
   uint8_t a_chan_cnt;        // count of enabled analog channels
   uint8_t d_chan_cnt;        // count of enabled digital channels
   uint8_t d_tx_bps;          // Digital Transmit bytes per slice
   // Pins sampled by the PIO - 4,8,16 or 32
   uint8_t pin_count;
   uint8_t d_nps; // digital nibbles per slice from a PIO/DMA perspective.
   uint32_t scnt; // number of samples sent
   char cmdstrptr;
   char cmdstr[20];                                             // used for parsing input
   uint32_t d_size, a_size;                                     // size of each of the two data buffers for each of a& d
   uint32_t dbuf0_start, dbuf1_start, abuf0_start, abuf1_start; // starting memory pointers of adc buffers
   char rspstr[20];
   // mark key control variables voltatile since multiple cores might access them
   volatile dev_state state;
   volatile bool cont;
   //a "+" was received from the host telling us to stop
   //this could be seen in continuous mode (host is stopping)
   //or in fixed mode (after an abort or because the host felt like it)
   //It is not a state machine state as it is somewhat asyncronous to the state
   //and could interfere with the normal orderly progression through the FSM.
   volatile bool usb_plus;
   /*Depracated trigger logic
   //If HW trigger enabled, uncomment all usages
   //volatile bool notfirst;  //Have we processed at least a first sample (so that lval is correct
   //  volatile bool triggered;
   //  uint32_t tlval; //last digital sample value - must keep it across multiple calls to slices for trigger
   //  uint32_t lvl0mask,lvl1mask,risemask,fallmask,chgmask;
   End depracated trigger logic*/
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
