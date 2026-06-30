"""
ADS1256 Boot & Channel Test Script for RP2040 Zero (MicroPython)
=================================================================
Wiring (adjust GP pins to match your build):
  RP2040 Zero  -->  ADS1256 Module
  3V3          -->  VCC (DVDD)
  GND          -->  GND / DGND
  GP2  (SCK)   -->  SCLK
  GP3  (MOSI)  -->  DIN
  GP4  (MISO)  -->  DOUT
  GP5  (CS)    -->  CS   (active LOW)
  GP6           -->  DRDY (active LOW, input)
  GP7           -->  RESET (active LOW, output) -- optional, tie HIGH if unused

Tests performed:
  1. Hardware reset (if RESET pin defined)
  2. Read STATUS register -> verify chip ID (upper nibble = 0x3)
  3. Read all register values and print them
  4. Single-ended scan of all 8 channels (AIN0-AIN7 vs AINCOM)
  5. Print raw 24-bit counts and voltage (assumes VREF = 2.5 V, PGA = 1)
"""

from machine import Pin, SPI
import time

# ── Pin assignments ────────────────────────────────────────────────────────────
SPI_ID   = 0          # SPI0 on RP2040 Zero
PIN_SCK  = 2
PIN_MOSI = 3
PIN_MISO = 0
PIN_CS   = 1
PIN_DRDY = 4
PIN_RST  = None          # Set to None if RESET is tied HIGH externally

# ── ADS1256 Constants ──────────────────────────────────────────────────────────
# Commands
CMD_WAKEUP  = 0x00
CMD_RDATA   = 0x01
CMD_RDATAC  = 0x03
CMD_SDATAC  = 0x0F
CMD_RREG    = 0x10
CMD_WREG    = 0x50
CMD_SELFCAL = 0xF0
CMD_RESET   = 0xFE
CMD_SYNC    = 0xFC

# Register addresses
REG_STATUS  = 0x00
REG_MUX     = 0x01
REG_ADCON   = 0x02
REG_DRATE   = 0x03

# Data rates (DRATE register value -> SPS label)
DRATE_30000 = 0xF0
DRATE_100   = 0x82

# PGA settings in ADCON (bits 2:0)
PGA_1  = 0b000
PGA_2  = 0b001
PGA_4  = 0b010
PGA_8  = 0b011
PGA_16 = 0b100
PGA_32 = 0b101
PGA_64 = 0b110

VREF = 2.5   # Volts — on-board reference

# ── Timing helpers ─────────────────────────────────────────────────────────────
# t6 delay: 50 * (1/7.68MHz) ≈ 6.51 µs  →  round up to 8 µs
T6_US = 8

def _delay_us(us):
    """Busy-wait microsecond delay."""
    t = time.ticks_us()
    while time.ticks_diff(time.ticks_us(), t) < us:
        pass

# ── SPI & GPIO setup ───────────────────────────────────────────────────────────
spi = SPI(SPI_ID,
          baudrate=1_920_000,      # ~1/4 of 7.68 MHz CLKIN on typical modules
          polarity=0,
          phase=1,                 # SPI Mode 1 (CPOL=0, CPHA=1)
          sck=Pin(PIN_SCK),
          mosi=Pin(PIN_MOSI),
          miso=Pin(PIN_MISO))

cs   = Pin(PIN_CS,   Pin.OUT, value=1)
drdy = Pin(PIN_DRDY, Pin.IN)
rst  = Pin(PIN_RST,  Pin.OUT, value=1) if PIN_RST is not None else None

# ── Low-level helpers ──────────────────────────────────────────────────────────
def wait_drdy(timeout_ms=2000):
    """Block until DRDY goes LOW (conversion ready). Raises on timeout."""
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while drdy.value() == 1:
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            raise RuntimeError("ADS1256: DRDY timeout — check wiring/power")

def cs_low():
    cs.value(0)

def cs_high():
    cs.value(1)

def send_cmd(cmd):
    cs_low()
    _delay_us(T6_US)
    spi.write(bytes([cmd]))
    _delay_us(T6_US)
    cs_high()

def read_register(reg):
    """Read one register and return its value."""
    cs_low()
    _delay_us(T6_US)
    spi.write(bytes([CMD_RREG | (reg & 0x0F), 0x00]))  # opcode1, opcode2 (read 1 reg)
    _delay_us(T6_US)
    result = spi.read(1)
    _delay_us(T6_US)
    cs_high()
    return result[0]

def write_register(reg, value):
    """Write one byte to a register."""
    wait_drdy()
    cs_low()
    _delay_us(T6_US)
    spi.write(bytes([CMD_WREG | (reg & 0x0F), 0x00, value]))
    _delay_us(T6_US)
    cs_high()

def read_data_raw():
    """
    Issue RDATA command and read back 3 bytes (24-bit signed result).
    Caller must ensure a conversion is ready (DRDY low) before calling.
    """
    cs_low()
    _delay_us(T6_US)
    spi.write(bytes([CMD_RDATA]))
    _delay_us(T6_US)
    buf = spi.read(3)
    _delay_us(T6_US)
    cs_high()

    raw = (buf[0] << 16) | (buf[1] << 8) | buf[2]
    # Sign-extend from 24 to 32 bits
    if raw & 0x800000:
        raw -= 0x1000000
    return raw

# ── High-level functions ───────────────────────────────────────────────────────
def hardware_reset():
    """Pulse RESET pin LOW for >4 CLKIN periods then wait for DRDY."""
    if rst is None:
        print("[RESET] No RESET pin defined — sending CMD_RESET over SPI.")
        send_cmd(CMD_RESET)
    else:
        rst.value(0)
        time.sleep_ms(1)
        rst.value(1)
    time.sleep_ms(100)   # datasheet: allow self-calibration after reset
    print("[RESET] Done.")

def stop_continuous():
    """Send SDATAC to ensure the chip is NOT in continuous read mode."""
    send_cmd(CMD_SDATAC)
    time.sleep_ms(1)

def self_calibrate():
    """Trigger offset + gain self-calibration and wait for completion."""
    print("[CAL] Starting self-calibration...")
    send_cmd(CMD_SELFCAL)
    wait_drdy()
    print("[CAL] Done.")

def check_status_register():
    """Read STATUS register and verify chip ID (upper nibble must be 0x3)."""
    status = read_register(REG_STATUS)
    chip_id = (status >> 4) & 0xF
    drdy_bit = (status >> 0) & 0x1   # bit 0: ORDER, bit 1: ACAL, bit 2: BUFEN
    print(f"[STATUS] Raw = 0x{status:02X}  |  Chip ID (upper nibble) = 0x{chip_id:X}", end="  ")
    if chip_id == 0x3:
        print("✓ ADS1256 identified correctly")
        return True
    else:
        print("✗ UNEXPECTED chip ID — check wiring, CS polarity, SPI mode")
        return False

def dump_registers():
    """Read and print all four user-accessible config registers."""
    names = {
        REG_STATUS: "STATUS",
        REG_MUX:    "MUX   ",
        REG_ADCON:  "ADCON ",
        REG_DRATE:  "DRATE ",
    }
    print("\n[REGS] Register dump:")
    for addr, name in names.items():
        val = read_register(addr)
        print(f"       0x{addr:02X}  {name}  = 0x{val:02X}  ({val:08b}b)")

def configure(pga=PGA_1, drate=DRATE_30000):
    """Set PGA and data rate. Buffer disabled (default)."""
    # ADCON: CLK1=0, CLK0=0 (no CLKOUT), SDCS=00 (sensor detect off), PGA
    adcon_val = pga & 0x07
    write_register(REG_ADCON, adcon_val)
    write_register(REG_DRATE, drate)

def read_channel(pos, neg=8):
    """
    Set MUX to (pos, neg) and return a single conversion.
    pos: 0-7 = AIN0-AIN7
    neg: 8   = AINCOM (single-ended)
    """
    mux_val = ((pos & 0x0F) << 4) | (neg & 0x0F)
    write_register(REG_MUX, mux_val)

    # SYNC + WAKEUP restarts conversion on new channel
    send_cmd(CMD_SYNC)
    _delay_us(T6_US)
    send_cmd(CMD_WAKEUP)

    wait_drdy()
    return read_data_raw()

def raw_to_volts(raw, vref=VREF, pga=1):
    """Convert a raw ADS1256 24-bit signed count to volts."""
    return raw * (2.0 * vref / pga) / (2 ** 23)

def read_channel_volts(pos, neg=8):
    return raw_to_volts(read_channel(pos, neg))
def scan_all_channels(vref=VREF, pga_gain=1):
    """
    Read all 8 single-ended channels (AIN0–AIN7 vs AINCOM).
    Returns list of (channel, raw_counts, voltage).
    """
    results = []
    lsb_voltage = (2.0 * vref) / (pga_gain * 0x7FFFFF)

    print(f"\n[SCAN] Scanning 8 channels  (VREF={vref}V, PGA={pga_gain}x)")
    print(f"       {'Ch':<4} {'Raw (counts)':>14}  {'Voltage (V)':>12}  {'Notes'}")
    print("       " + "-" * 52)

    for ch in range(8):
        raw = read_channel(ch, neg=8)  # 8 = AINCOM
        voltage = raw * lsb_voltage

        # Flag out-of-range (clipped at positive or negative full-scale)
        if raw >= 0x7FFFFF:
            note = "⚠ +FS clipped"
        elif raw <= -0x800000:
            note = "⚠ -FS clipped"
        else:
            note = ""

        print(f"       AIN{ch:<2} {raw:>14d}  {voltage:>12.6f} V  {note}")
        results.append((ch, raw, voltage))

    return results

# ── Main test sequence ─────────────────────────────────────────────────────────
def run_test():
    print("=" * 58)
    print("  ADS1256 Boot & Channel Test  —  RP2040 Zero / MicroPython")
    print("=" * 58)

    # 1. Reset
    hardware_reset()

    # 2. Stop any continuous read mode leftover from power-on
    stop_continuous()

    # 3. Verify chip identity
    ok = check_status_register()
    if not ok:
        print("\n[ABORT] Cannot continue without valid chip ID.")
        return

    # 4. Show register state fresh from reset
    dump_registers()

    # 5. Configure for test (PGA=1, 30 kSPS)
    configure(pga=PGA_1, drate=DRATE_30000)
    print("\n[CFG] Set PGA=1, DRATE=0xF0 (30 kSPS)")

    # 6. Self-calibrate
    self_calibrate()

    # 7. Scan all channels
    scan_all_channels(vref=VREF, pga_gain=1)

    print("\n[DONE] Test complete.")
    print("=" * 58)
