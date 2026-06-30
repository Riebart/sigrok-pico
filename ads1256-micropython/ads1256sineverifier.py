"""
ADS1256SineVerifier — MicroPython class for RP2040 Zero
========================================================
Captures samples from one or more ADS1256 channels and verifies that the
input signal matches an expected sine wave. Computes:

  • SNR   (signal-to-noise ratio, dB)
  • THD   (total harmonic distortion, dB — harmonics 2-5)
  • SINAD (signal + noise + distortion, dB)
  • ENOB  (effective number of bits) from SINAD
  • DC offset error (counts and µV)
  • Amplitude error vs expected (%)
  • Peak-to-peak noise floor on a DC input (ENOB_noise)

Depends on ads1256_test.py primitives (spi, cs, drdy, wait_drdy,
write_register, read_data_raw, etc.) — import them directly so this
class does not duplicate hardware setup.

Spectrum analysis uses a Goertzel algorithm instead of a full FFT.
This avoids importing ulab/numpy which may not be available, and is
very SRAM-friendly: it evaluates only the frequency bins we care about
(fundamental + harmonics 2-5 + DC) rather than computing all N bins.

Goertzel algorithm
------------------
For a block of N samples and a target bin k (where k = round(N * f_target / f_sample)):
  ω  = 2π k / N
  coeff = 2 cos(ω)
  s[n] = x[n] + coeff * s[n-1] - s[n-2],  s[-1]=s[-2]=0
  power = s[N-1]² + s[N-2]² - coeff * s[N-1] * s[N-2]
  magnitude = sqrt(power) / (N/2)   [normalised to amplitude in counts]

ENOB from SINAD (IEEE 1057, sine-wave test):
  ENOB = (SINAD_dB - 1.76) / 6.02

ENOB from noise floor (DC input, delta-sigma style):
  ENOB_noise = 24 - log2(std_dev)     [where 24 = ADS1256 bit depth]

Fixes applied vs v1:
  BUG-3  Coarse bin scan step was 4 — could skip the fundamental bin entirely.
         Fixed: step=1 scan up to N//2 (bounded to keep runtime acceptable).
  BUG-4  Sample rate estimated with ticks_ms() (1 ms resolution) → ~6% error.
         Fixed: use ticks_us().
  BUG-5  CRITICAL — at 30 kSPS with N=512, only ~1 cycle of a 61 Hz sine fits
         in the capture window. Goertzel needs >= 10 cycles for reliable ENOB.
         Fixed: verifier now warns and recommends a lower data rate or larger N.
         Runner updated to use DRATE_1000 (1000 SPS) for sine tests → 512
         samples captures 17 full cycles of 61 Hz. Much better.
  BUG-7  ADC_BITS=24 in ENOB_noise overcounts by 1 bit (ADS1256 is ±2^23).
         Fixed: use 23 in the noise ENOB formula.
  BUG-8  expected_amp_v was hardcoded as 0.45 * fsr/2, which assumed AVDD=5V.
         With 3.3V supply the amplitude is 0.45 * 0.5 * 3.3 V.
         Fixed: expected_amp_v is now a constructor parameter.

Spectrum analysis uses the Goertzel algorithm (no FFT / ulab required).
"""

import math
import array
import time

from ads1256 import (
    wait_drdy, read_data_raw, write_register, send_cmd,
    REG_MUX, CMD_SYNC, CMD_WAKEUP, _delay_us, T6_US
)

ADC_BITS = 23      # ADS1256: ±2^23 full-scale  (positive FS = 2^23 - 1 counts)
VREF     = 2.5     # Default on-board reference


class ADS1256SineVerifier:
    """
    Captures ADS1256 samples and evaluates sine-wave quality metrics.

    Parameters
    ----------
    n_samples        : int   — samples per capture (recommend >= 512)
    vref             : float — reference voltage in V (default 2.5)
    pga              : int   — PGA gain (1,2,4,8,16,32,64)
    n_harmonics      : int   — harmonics to include in THD (2 through n+1)
    avdd             : float — analog supply voltage (default 3.3 V)
                               Used to compute expected amplitude from PWM generator.
    sine_amplitude   : float — PWMSineGenerator amplitude fraction (default 0.45)
    sine_dc_offset   : float — PWMSineGenerator dc_offset fraction (default 0.5)
    """

    def __init__(self, n_samples=512, vref=VREF, pga=1, n_harmonics=5,
                 avdd=3.3, sine_amplitude=0.45, sine_dc_offset=0.5):
        if n_samples < 64:
            raise ValueError("n_samples must be >= 64")
        self.n_samples      = n_samples
        self.vref           = vref
        self.pga            = pga
        self.n_harmonics    = n_harmonics
        self.avdd           = avdd
        self.sine_amplitude = sine_amplitude
        self.sine_dc_offset = sine_dc_offset

        self.fsr   = (2.0 * vref) / pga           # ADC full-scale range in V
        self.lsb_v = self.fsr / (2 ** ADC_BITS)   # LSB voltage

        # Expected amplitude from PWM generator (peak, in ADC voltage space)
        # PWM output: amplitude_fraction * dc_offset_fraction * avdd  (peak swing)
        # But this is a single-supply signal biased at avdd/2.
        # Peak amplitude of the sine around mid-rail:
        self.expected_amp_v = sine_amplitude * sine_dc_offset * avdd

        self._buf = array.array('l', [0] * n_samples)

    # ── Internal: channel select ──────────────────────────────────────────────

    def _select_channel(self, pos, neg=8):
        mux = ((pos & 0x0F) << 4) | (neg & 0x0F)
        write_register(REG_MUX, mux)
        send_cmd(CMD_SYNC)
        _delay_us(T6_US)
        send_cmd(CMD_WAKEUP)
        wait_drdy()

    # ── Capture ───────────────────────────────────────────────────────────────

    def capture(self, channel_pos, channel_neg=8):
        """
        Fill internal buffer with n_samples from the given channel.
        Returns elapsed time in microseconds (ticks_us for accuracy).
        """
        self._select_channel(channel_pos, channel_neg)
        buf = self._buf
        n   = self.n_samples
        t0  = time.ticks_us()
        for i in range(n):
            wait_drdy()
            buf[i] = read_data_raw()
        return time.ticks_diff(time.ticks_us(), t0)

    # ── DC analysis ───────────────────────────────────────────────────────────

    def analyse_dc(self, channel_pos, channel_neg=8):
        elapsed_us = self.capture(channel_pos, channel_neg)
        buf = self._buf
        n   = self.n_samples

        mean     = sum(buf) / n
        variance = sum((x - mean) ** 2 for x in buf) / n
        std      = math.sqrt(variance) if variance > 0 else 1e-9
        pk_pk    = max(buf) - min(buf)

        # ENOB from noise floor — using ADC_BITS = 23 (FIX BUG-7)
        enob_noise = ADC_BITS - math.log(max(std, 0.5), 2)

        mean_v = mean  * self.lsb_v
        std_v  = std   * self.lsb_v
        pp_v   = pk_pk * self.lsb_v

        return {
            "n_samples":   n,
            "elapsed_ms":  elapsed_us / 1000.0,
            "mean_counts": mean,
            "mean_v":      mean_v,
            "std_counts":  std,
            "std_v":       std_v,
            "std_uv":      std_v * 1e6,
            "peak_peak_v": pp_v,
            "enob_noise":  enob_noise,
            "offset_uv":   mean_v * 1e6,
        }

    # ── Goertzel ──────────────────────────────────────────────────────────────

    def _goertzel(self, buf, n, k):
        """Return unnormalised power at bin k."""
        omega = 2.0 * math.pi * k / n
        coeff = 2.0 * math.cos(omega)
        s1 = s2 = 0.0
        for x in buf:
            s0 = x + coeff * s1 - s2
            s2, s1 = s1, s0
        return s1 * s1 + s2 * s2 - coeff * s1 * s2

    def _find_fundamental_bin(self, buf, n, hint_bin=None, search_width=5):
        """
        Find the bin with the peak spectral energy.
        FIX BUG-3: scan step = 1 (was 4, could skip low bins entirely).
        Search is bounded to n//2 to avoid negative-frequency mirror.
        """
        if hint_bin is not None:
            lo = max(1, hint_bin - search_width)
            hi = min(n // 2 - 1, hint_bin + search_width)
        else:
            lo, hi = 1, n // 2 - 1

        best_bin, best_pwr = lo, 0.0
        for k in range(lo, hi + 1):
            p = self._goertzel(buf, n, k)
            if p > best_pwr:
                best_pwr, best_bin = p, k
        return best_bin, best_pwr

    # ── Sine analysis ─────────────────────────────────────────────────────────

    def analyse_sine(self, channel_pos, channel_neg=8,
                     expected_freq_hz=None, sample_rate_hz=None):
        """
        Capture N samples and compute SNR, THD, SINAD, ENOB.

        Warns if fewer than 10 sine cycles are captured — results will be
        unreliable. Lower the ADS1256 data rate or increase n_samples.
        """
        elapsed_us = self.capture(channel_pos, channel_neg)
        buf = self._buf
        n   = self.n_samples

        # FIX BUG-4: use ticks_us elapsed for sample rate estimation
        elapsed_s = elapsed_us / 1_000_000.0
        if sample_rate_hz is None:
            sample_rate_hz = n / elapsed_s if elapsed_s > 0 else 1.0

        # Cycle count check — warn before computing (FIX BUG-5)
        if expected_freq_hz is not None:
            n_cycles = expected_freq_hz * (self.n_samples / sample_rate_hz)
            if n_cycles < 10:
                print(f"  ⚠ WARNING: only {n_cycles:.1f} cycles of {expected_freq_hz:.1f} Hz"
                      f" captured at {sample_rate_hz:.0f} SPS.")
                print(f"    Reduce data rate to <= "
                      f"{int(n * expected_freq_hz / 10)} SPS, "
                      f"or increase n_samples to >= "
                      f"{int(10 * sample_rate_hz / expected_freq_hz)}.")

        # DC offset removal
        mean    = sum(buf) / n
        dc_offset_v = mean * self.lsb_v
        ac_buf  = array.array('l', (int(x - mean) for x in buf))

        # Fundamental bin
        hint = None
        if expected_freq_hz is not None:
            hint = max(1, round(n * expected_freq_hz / sample_rate_hz))

        fund_bin, fund_pwr = self._find_fundamental_bin(ac_buf, n, hint_bin=hint)
        freq_hz = fund_bin * sample_rate_hz / n

        # Amplitude: sqrt(power) / (N/2) gives peak in counts
        fund_amp_counts = math.sqrt(max(fund_pwr, 0.0)) / (n / 2.0)
        amplitude_v     = fund_amp_counts * self.lsb_v

        # Harmonic powers
        harm_pwrs = []
        for h in range(2, self.n_harmonics + 2):
            hbin = fund_bin * h
            if hbin >= n // 2:
                break
            harm_pwrs.append(self._goertzel(ac_buf, n, hbin))

        # Power budget
        total_pwr    = sum(float(x) * float(x) for x in ac_buf)
        harm_total   = sum(harm_pwrs)
        noise_pwr    = max(total_pwr - fund_pwr - harm_total, 1.0)

        snr_db   = 10.0 * math.log10(fund_pwr / noise_pwr) if noise_pwr > 0 else -999.0
        thd_lin  = math.sqrt(harm_total / max(fund_pwr, 1.0))
        thd_db   = 20.0 * math.log10(thd_lin) if thd_lin > 0 else -999.0
        sinad_db = 10.0 * math.log10(fund_pwr / max(noise_pwr + harm_total, 1.0))
        enob     = (sinad_db - 1.76) / 6.02

        # Amplitude error vs expected (FIX BUG-8: use measured avdd-based expectation)
        amp_err_pct = None
        if self.expected_amp_v > 0:
            amp_err_pct = ((amplitude_v - self.expected_amp_v) / self.expected_amp_v) * 100.0

        return {
            "channel":             f"AIN{channel_pos}" + ("" if channel_neg == 8
                                   else f"-AIN{channel_neg}"),
            "n_samples":           n,
            "elapsed_ms":          elapsed_us / 1000.0,
            "sample_rate_hz":      sample_rate_hz,
            "freq_hz":             freq_hz,
            "fund_bin":            fund_bin,
            "n_cycles":            freq_hz * elapsed_s,
            "amplitude_v":         amplitude_v,
            "amplitude_error_pct": amp_err_pct,
            "dc_offset_v":         dc_offset_v,
            "snr_db":              snr_db,
            "thd_db":              thd_db,
            "sinad_db":            sinad_db,
            "enob":                enob,
        }

    def scan_channels(self, channels, expected_freq_hz=None, sample_rate_hz=None):
        results = []
        for spec in channels:
            pos = spec[0]
            neg = spec[1] if len(spec) > 1 else 8
            results.append(self.analyse_sine(pos, neg, expected_freq_hz, sample_rate_hz))
        return results

    # ── Print helpers ─────────────────────────────────────────────────────────

    @staticmethod
    def print_sine_result(r, expected_enob=16.0):
        status = "✓ PASS" if r["enob"] >= expected_enob else "✗ FAIL"
        print(f"\n{'─'*56}")
        print(f"  Channel    : {r['channel']}")
        print(f"  Samples    : {r['n_samples']}  ({r['elapsed_ms']:.1f} ms,  "
              f"{r['sample_rate_hz']:.0f} SPS est.)")
        print(f"  Freq det.  : {r['freq_hz']:.2f} Hz  (bin {r['fund_bin']},  "
              f"{r['n_cycles']:.1f} cycles)")
        print(f"  Amplitude  : {r['amplitude_v']*1000:.3f} mV peak", end="")
        if r["amplitude_error_pct"] is not None:
            print(f"  (error {r['amplitude_error_pct']:+.1f}%)")
        else:
            print()
        print(f"  DC offset  : {r['dc_offset_v']*1000:.3f} mV")
        print(f"  SNR        : {r['snr_db']:.2f} dB")
        print(f"  THD        : {r['thd_db']:.2f} dB")
        print(f"  SINAD      : {r['sinad_db']:.2f} dB")
        print(f"  ENOB       : {r['enob']:.2f} bits  {status}")
        print(f"{'─'*56}")

    @staticmethod
    def print_dc_result(r, channel_label=""):
        print(f"\n{'─'*56}")
        print(f"  Channel    : {channel_label or 'DC test'}")
        print(f"  Samples    : {r['n_samples']}  ({r['elapsed_ms']:.1f} ms)")
        print(f"  Mean       : {r['mean_v']*1000:.6f} mV  "
              f"({r['offset_uv']:.2f} µV offset)")
        print(f"  Noise RMS  : {r['std_uv']:.3f} µV")
        print(f"  Pk-Pk      : {r['peak_peak_v']*1e6:.1f} µV")
        print(f"  ENOB noise : {r['enob_noise']:.2f} bits")
        print(f"{'─'*56}")

    @staticmethod
    def print_channel_comparison(results, expected_enob=16.0):
        print(f"\n{'Ch':<10} {'Cycles':>7} {'Freq Hz':>8} {'Amp mV':>8} "
              f"{'DC mV':>8} {'SNR dB':>8} {'THD dB':>8} {'ENOB':>6}  Status")
        print("─" * 76)
        for r in results:
            ok  = "✓" if r["enob"] >= expected_enob else "✗"
            print(f"{r['channel']:<10} {r['n_cycles']:>7.1f} {r['freq_hz']:>8.2f} "
                  f"{r['amplitude_v']*1000:>8.3f} {r['dc_offset_v']*1000:>8.3f} "
                  f"{r['snr_db']:>8.2f} {r['thd_db']:>8.2f} {r['enob']:>6.2f}  {ok}")
        print()

# ─────────────────────────────────────────────────────────────────────────────
# ADD THIS METHOD to the ADS1256SineVerifier class body.
# Place it after the existing scan_channels() method and before the print helpers.
#
# Also add this import at the top of ads1256_verifier.py (if not already present):
#   from machine import ADC, Pin
# Changes vs v1:
#   - Computes carrier amplitude via Goertzel at the expected carrier bin
#   - Also searches ±search_width bins around the carrier bin for the actual peak
#   - Reports carrier_amplitude_v, carrier_freq_hz, carrier_snr_db
#   - Adds carrier lines to the printed summary
#
# Constructor change: add `carrier_hz=62_500` parameter to __init__ and store
# as self.carrier_hz so this method can find the correct carrier bin.
# If you don't want to change __init__, pass carrier_hz as a method parameter
# (see signature below).
# ─────────────────────────────────────────────────────────────────────────────
    def debug_confirm2(self, gpio_pin=26, n_samples=2048,
                        sample_rate_hz=50_000, expected_freq_hz=None,
                        carrier_hz=62_500):
        """
        Confirm PWM sine signal presence using the RP2040's internal ADC.

        Parameters
        ----------
        gpio_pin         : int   — RP2040 ADC pin (GP26=ADC0, GP27=ADC1, GP28=ADC2)
        n_samples        : int   — samples to capture (default 2048)
        sample_rate_hz   : int   — target sample rate in SPS (default 50_000)
        expected_freq_hz : float — expected sine fundamental in Hz (optional)
        carrier_hz       : int   — PWM carrier frequency in Hz (default 62_500)
                                   Used to locate the carrier bin in the spectrum.
                                   The carrier aliases to:
                                     alias = carrier_hz % sample_rate_hz
                                   This aliased frequency is what Goertzel finds.

        Returns
        -------
        dict — same keys as v1, plus:
          carrier_amplitude_v  : float — detected carrier amplitude in volts
          carrier_freq_hz      : float — detected carrier frequency (aliased)
          carrier_alias_hz     : float — expected alias of carrier_hz at sample_rate_hz
          carrier_snr_db       : float — carrier power vs noise floor (dB)
          fund_to_carrier_db   : float — fundamental amplitude vs carrier (dB)
                                         ideally very negative (carrier dominates
                                         without RC filter; fundamental dominates after)
        """
        from machine import ADC, Pin

        _adc_pins = {26: 0, 27: 1, 28: 2}
        if gpio_pin not in _adc_pins:
            raise ValueError(f"GP{gpio_pin} is not an ADC pin. Use GP26, GP27, or GP28.")

        adc = ADC(Pin(gpio_pin))

        # ── Capture ───────────────────────────────────────────────────────────
        interval_us = int(1_000_000 / sample_rate_hz)
        buf = array.array('H', [0] * n_samples)

        t0 = time.ticks_us()
        next_tick = t0
        for i in range(n_samples):
            while time.ticks_diff(time.ticks_us(), next_tick) < 0:
                pass
            buf[i] = adc.read_u16()
            next_tick = time.ticks_add(next_tick, interval_us)
        t_elapsed_us = time.ticks_diff(time.ticks_us(), t0)

        actual_rate_hz = n_samples / (t_elapsed_us / 1_000_000.0)
        elapsed_s      = t_elapsed_us / 1_000_000.0

        PICO_ADC_SCALE = 3.3 / 65535.0

        # ── DC removal ────────────────────────────────────────────────────────
        mean_raw    = sum(buf) / n_samples
        dc_offset_v = mean_raw * PICO_ADC_SCALE
        ac_buf      = array.array('l', (int(x - mean_raw) for x in buf))

        pk_pk_raw         = max(ac_buf) - min(ac_buf)
        noise_threshold   = 200
        signal_present    = pk_pk_raw > noise_threshold

        # ── Fundamental Goertzel ──────────────────────────────────────────────
        hint_bin = None
        if expected_freq_hz is not None:
            hint_bin = max(1, round(n_samples * expected_freq_hz / actual_rate_hz))

        fund_bin, fund_pwr = self._find_fundamental_bin(
            ac_buf, n_samples, hint_bin=hint_bin, search_width=8
        )
        freq_hz        = fund_bin * actual_rate_hz / n_samples
        n_cycles       = freq_hz * elapsed_s
        fund_amp_raw   = math.sqrt(max(fund_pwr, 0.0)) / (n_samples / 2.0)
        amplitude_v    = fund_amp_raw * PICO_ADC_SCALE

        # ── Carrier Goertzel ──────────────────────────────────────────────────
        # The carrier aliases: alias_hz = carrier_hz % actual_rate_hz
        # Use the fractional remainder to find the correct bin.
        carrier_alias_hz  = carrier_hz % actual_rate_hz
        # Mirror aliases above Nyquist back down
        nyquist = actual_rate_hz / 2.0
        if carrier_alias_hz > nyquist:
            carrier_alias_hz = actual_rate_hz - carrier_alias_hz

        carrier_hint_bin  = max(1, round(n_samples * carrier_alias_hz / actual_rate_hz))
        carrier_bin, carrier_pwr = self._find_fundamental_bin(
            ac_buf, n_samples,
            hint_bin=carrier_hint_bin,
            search_width=8
        )
        carrier_freq_hz   = carrier_bin * actual_rate_hz / n_samples
        carrier_amp_raw   = math.sqrt(max(carrier_pwr, 0.0)) / (n_samples / 2.0)
        carrier_amp_v     = carrier_amp_raw * PICO_ADC_SCALE

        # ── Power budget ──────────────────────────────────────────────────────
        total_pwr    = sum(float(x) * float(x) for x in ac_buf)
        noise_pwr    = max(total_pwr - fund_pwr - carrier_pwr, 1.0)
        snr_db       = 10.0 * math.log10(fund_pwr    / noise_pwr) if noise_pwr > 0 else -999.0
        carrier_snr  = 10.0 * math.log10(carrier_pwr / noise_pwr) if noise_pwr > 0 else -999.0

        # Fundamental vs carrier ratio — negative means carrier dominates (no RC filter)
        fund_to_carrier_db = (20.0 * math.log10(fund_amp_raw / carrier_amp_raw)
                              if carrier_amp_raw > 0 else 0.0)

        freq_error_hz = None
        if expected_freq_hz is not None:
            freq_error_hz = freq_hz - expected_freq_hz

        # ── Print ─────────────────────────────────────────────────────────────
        filter_note = ("carrier >> fundamental — RC filter absent or ineffective"
                       if fund_to_carrier_db < -20
                       else "fundamental dominates — RC filter working" if fund_to_carrier_db > 0
                       else "carrier and fundamental comparable")
        status = "✓ SIGNAL PRESENT" if signal_present else "✗ NO SIGNAL DETECTED"

        print(f"\n{'═'*58}")
        print(f"  DEBUG CONFIRM — RP2040 ADC on GP{gpio_pin}")
        print(f"{'═'*58}")
        print(f"  Samples      : {n_samples}  ({elapsed_s*1000:.1f} ms)")
        print(f"  Actual rate  : {actual_rate_hz:.0f} SPS  (target {sample_rate_hz} SPS)")
        print(f"  Pk-Pk raw    : {pk_pk_raw} counts  (threshold >{noise_threshold})")
        print(f"  DC offset    : {dc_offset_v*1000:.1f} mV  "
              f"(expect ~{self.avdd * self.sine_dc_offset * 1000:.0f} mV mid-rail)")
        print(f"  ── Fundamental ──────────────────────────────────────")
        print(f"  Freq det.    : {freq_hz:.2f} Hz  (bin {fund_bin},  {n_cycles:.1f} cycles)", end="")
        if freq_error_hz is not None:
            print(f"  error {freq_error_hz:+.2f} Hz")
        else:
            print()
        print(f"  Amplitude    : {amplitude_v*1000:.1f} mV peak  "
              f"(expect ~{self.expected_amp_v*1000:.0f} mV)")
        print(f"  SNR (fund)   : {snr_db:.1f} dB")
        print(f"  ── Carrier ──────────────────────────────────────────")
        print(f"  Alias freq   : {carrier_alias_hz:.1f} Hz  "
              f"(carrier {carrier_hz} Hz @ {actual_rate_hz:.0f} SPS)")
        print(f"  Carrier det. : {carrier_freq_hz:.2f} Hz  (bin {carrier_bin})")
        print(f"  Carrier amp  : {carrier_amp_v*1000:.1f} mV peak")
        print(f"  Carrier SNR  : {carrier_snr:.1f} dB")
        print(f"  Fund/carrier : {fund_to_carrier_db:+.1f} dB  ({filter_note})")
        print(f"  ── Result ───────────────────────────────────────────")
        print(f"  Status       : {status}")
        print(f"{'═'*58}\n")

        return {
            "signal_present":       signal_present,
            "amplitude_v":          amplitude_v,
            "dc_offset_v":          dc_offset_v,
            "freq_hz":              freq_hz,
            "snr_db":               snr_db,
            "freq_error_hz":        freq_error_hz,
            "n_cycles":             n_cycles,
            "actual_rate_hz":       actual_rate_hz,
            "carrier_amplitude_v":  carrier_amp_v,
            "carrier_freq_hz":      carrier_freq_hz,
            "carrier_alias_hz":     carrier_alias_hz,
            "carrier_snr_db":       carrier_snr,
            "fund_to_carrier_db":   fund_to_carrier_db,
        }


    def debug_confirm(self, gpio_pin=26, n_samples=2048,
                        sample_rate_hz=50_000, expected_freq_hz=None):
        """
        Confirm that the PWM sine signal is present on a GPIO pin using the
        RP2040's own 12-bit ADC — independent of the ADS1256.

        Use this when all ADS1256 channels are failing to isolate whether the
        problem is with the signal generator or with the ADS1256 / SPI path.

        The RP2040 ADC has 12-bit resolution (read_u16 returns 0–65535, mapped
        from 12 bits, so effective resolution is ~8.7 ENOB due to its known
        noise floor). It is only used here as a go/no-go presence detector,
        not for precision measurement.

        Parameters
        ----------
        gpio_pin        : int   — GPIO number for RP2040 ADC input.
                                  ADC0=GP26, ADC1=GP27, ADC2=GP28.
                                  Connect this pin to the RC filter output
                                  (same node as the ADS1256 AINx input).
        n_samples       : int   — number of samples to capture (default 2048).
        sample_rate_hz  : int   — target sample rate in SPS (default 50_000).
                                  The RP2040 ADC maxes out at ~500 kSPS in
                                  MicroPython; 50 kSPS is conservative and
                                  reliable without DMA.
                                  At 50 kSPS, 2048 samples → 40.96 ms window.
                                  A 61 Hz sine produces ~2.5 cycles — enough
                                  for presence detection and rough amplitude
                                  check. Increase n_samples for spectral use.
        expected_freq_hz: float — expected sine frequency. If provided, the
                                  detected frequency bin is compared against it.

        Returns
        -------
        dict with keys:
          signal_present  : bool   — True if amplitude exceeds noise threshold
          amplitude_v     : float  — detected peak amplitude in volts
          dc_offset_v     : float  — DC mid-rail in volts
          freq_hz         : float  — detected fundamental frequency (Hz)
          snr_db          : float  — rough SNR from Goertzel (dB)
          freq_error_hz   : float  — deviation from expected_freq_hz (or None)
          n_cycles        : float  — sine cycles captured in window
          actual_rate_hz  : float  — measured sample rate (from ticks_us)

        Prints a human-readable summary automatically.

        Wiring note
        -----------
        The RP2040 Zero ADC pins are GP26/27/28. They must NOT be connected to
        voltages above 3.3V. The PWM sine output is biased at AVDD/2 = 1.65V
        with ±0.45×0.5×3.3 = ±0.7425V swing — safely within range.
        """
        from machine import ADC, Pin

        # ── Validate pin ──────────────────────────────────────────────────────
        _adc_pins = {26: 0, 27: 1, 28: 2}
        if gpio_pin not in _adc_pins:
            raise ValueError(f"GP{gpio_pin} is not an ADC pin. Use GP26, GP27, or GP28.")

        adc = ADC(Pin(gpio_pin))

        # ── Collect samples at target rate using ticks_us busy-wait ──────────
        interval_us = int(1_000_000 / sample_rate_hz)
        buf = array.array('H', [0] * n_samples)   # uint16, raw read_u16 values

        t0 = time.ticks_us()
        next_tick = t0
        for i in range(n_samples):
            while time.ticks_diff(time.ticks_us(), next_tick) < 0:
                pass
            buf[i] = adc.read_u16()
            next_tick = time.ticks_add(next_tick, interval_us)
        t_elapsed_us = time.ticks_diff(time.ticks_us(), t0)

        actual_rate_hz = n_samples / (t_elapsed_us / 1_000_000.0)
        elapsed_s      = t_elapsed_us / 1_000_000.0

        # ── DC removal and basic stats ────────────────────────────────────────
        # read_u16 returns 0–65535 (12-bit value left-shifted to 16-bit)
        # Convert to volts: v = raw * 3.3 / 65535
        PICO_ADC_SCALE = 3.3 / 65535.0

        mean_raw  = sum(buf) / n_samples
        dc_offset_v = mean_raw * PICO_ADC_SCALE

        # AC-coupled buffer in raw counts (integer, centred on zero)
        ac_buf = array.array('l', (int(x - mean_raw) for x in buf))

        pk_pk_raw = max(ac_buf) - min(ac_buf)
        noise_threshold_raw = 200   # ~10 mV at 3.3V/65535 ≈ counts — below this = no signal

        # ── Goertzel for fundamental ──────────────────────────────────────────
        hint_bin = None
        if expected_freq_hz is not None:
            hint_bin = max(1, round(n_samples * expected_freq_hz / actual_rate_hz))

        # Reuse existing _find_fundamental_bin — it operates on any array('l')
        fund_bin, fund_pwr = self._find_fundamental_bin(
            ac_buf, n_samples, hint_bin=hint_bin, search_width=8
        )
        freq_hz   = fund_bin * actual_rate_hz / n_samples
        n_cycles  = freq_hz * elapsed_s

        # Amplitude from Goertzel (peak, in raw ADC counts)
        fund_amp_raw = math.sqrt(max(fund_pwr, 0.0)) / (n_samples / 2.0)
        amplitude_v  = fund_amp_raw * PICO_ADC_SCALE

        # Total AC power and noise
        total_pwr = sum(float(x) * float(x) for x in ac_buf)
        noise_pwr = max(total_pwr - fund_pwr, 1.0)
        snr_db    = 10.0 * math.log10(fund_pwr / noise_pwr) if noise_pwr > 0 else -999.0

        # Signal present if peak-to-peak exceeds threshold
        signal_present = pk_pk_raw > noise_threshold_raw

        # Frequency error
        freq_error_hz = None
        if expected_freq_hz is not None:
            freq_error_hz = freq_hz - expected_freq_hz

        # ── Print summary ─────────────────────────────────────────────────────
        status = "✓ SIGNAL PRESENT" if signal_present else "✗ NO SIGNAL DETECTED"
        print(f"\n{'═'*56}")
        print(f"  DEBUG CONFIRM — RP2040 ADC0 on GP{gpio_pin}")
        print(f"{'═'*56}")
        print(f"  Samples      : {n_samples}  ({elapsed_s*1000:.1f} ms)")
        print(f"  Actual rate  : {actual_rate_hz:.0f} SPS  "
              f"(target {sample_rate_hz} SPS)")
        print(f"  Cycles cap.  : {n_cycles:.1f}")
        print(f"  DC offset    : {dc_offset_v*1000:.1f} mV  "
              f"(expect ~{self.avdd*self.sine_dc_offset*1000:.0f} mV mid-rail)")
        print(f"  Amplitude    : {amplitude_v*1000:.1f} mV peak  "
              f"(expect ~{self.expected_amp_v*1000:.0f} mV)")
        print(f"  Pk-Pk raw    : {pk_pk_raw} counts  "
              f"(threshold >{noise_threshold_raw})")
        print(f"  Freq det.    : {freq_hz:.2f} Hz  (bin {fund_bin})", end="")
        if freq_error_hz is not None:
            print(f"  error {freq_error_hz:+.2f} Hz")
        else:
            print()
        print(f"  SNR (rough)  : {snr_db:.1f} dB")
        print(f"  Status       : {status}")
        print(f"{'═'*56}\n")

        return {
            "signal_present":  signal_present,
            "amplitude_v":     amplitude_v,
            "dc_offset_v":     dc_offset_v,
            "freq_hz":         freq_hz,
            "snr_db":          snr_db,
            "freq_error_hz":   freq_error_hz,
            "n_cycles":        n_cycles,
            "actual_rate_hz":  actual_rate_hz,
        }
