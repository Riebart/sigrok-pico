"""
sine_test_runner.py — End-to-end ADS1256 sine test
====================================================
Wires together PWMSineGenerator and ADS1256SineVerifier to:
  1. Generate a PWM sine on a GPIO pin via RC filter
  2. Capture and analyse it on one or more ADS1256 channels
  3. Print ENOB, SNR, THD, SINAD and channel mismatch table

Wiring for this test
--------------------
  GP0 → 1kΩ → node A → AIN0 (and optionally daisy-chained to AIN1-AIN7)
                      → 10nF → GND      (single-pole RC, fc ≈ 15.9 kHz)

  ADS1256 SPI lines must already be wired per ads1256_test.py defaults.

Adjust SIGNAL_PIN, ADS_CHANNELS, and ADS_RATE_HZ to match your setup.

Key fix from v1: the sine test now uses DRATE_1000 (1000 SPS) instead of
DRATE_30000. At 30 kSPS only ~1 cycle of a 61 Hz sine fits in 512 samples,
making Goertzel/ENOB analysis meaningless. At 1000 SPS, 512 samples captures
~31 full cycles — plenty for reliable spectral analysis.
"""

import time
from pwmsine         import PWMSineGenerator
from ads1256sineverifier import ADS1256SineVerifier
import ads1256   as ads

# ── Configuration ─────────────────────────────────────────────────────────────
SIGNAL_PIN    = 29            # GPIO driving RC filter
N_LUT         = 256          # PWM sine LUT size
TICK_HZ       = 15_625       # Timer ISR rate → fundamental = 15625/256 ≈ 61 Hz
CARRIER_HZ    = 62_500       # PWM carrier frequency
HARMONIC      = 1            # Sine harmonic (1 = fundamental)

# ADS1256 channels to test
#ADS_CHANNELS  = [(0, 8), (1, 8), (2, 8), (3, 8),
#                 (4, 8), (5, 8), (6, 8), (7, 8)]

ADS_CHANNELS  = [(0, 8)]

ADS_N_SAMPLES = 512          # Samples per channel

# FIX BUG-5: Use 1000 SPS for sine test.
# 512 samples @ 1000 SPS = 512 ms window = 31.2 cycles of 61 Hz. ✓
# Change to DRATE_100 for even cleaner results (slower).
SINE_DRATE    = 0xA1         # 1000 SPS
NOISE_DRATE   = 0x82         # 100 SPS  (for DC / noise floor test)

AVDD          = 3.3          # Supply voltage on your board
EXPECTED_ENOB = 16.0         # Conservative minimum (improves at lower data rate)
SETTLE_MS     = 500          # RC filter settle time after generator starts

_fundamental_hz = TICK_HZ / N_LUT   # ≈ 61.04 Hz


def _make_verifier():
    return ADS1256SineVerifier(
        n_samples=ADS_N_SAMPLES,
        avdd=AVDD,
        sine_amplitude=0.45,
        sine_dc_offset=0.5,
    )


def run_noise_floor_test(channel_pos=0):
    print("\n" + "=" * 56)
    print("  STEP 1: DC Noise Floor  (input shorted to AGND)")
    print("=" * 56)
    print(f"  Short AIN{channel_pos} pin directly to AGND, then press Enter...")
    input()

    ads.write_register(ads.REG_DRATE, NOISE_DRATE)
    ads.self_calibrate()

    v = _make_verifier()
    r = v.analyse_dc(channel_pos)
    ADS1256SineVerifier.print_dc_result(r, f"AIN{channel_pos} → AGND")


def run_full_scale_test(channel_pos=0):
    print("\n" + "=" * 56)
    print("  STEP 2: Full-Scale Accuracy  (input tied to 2.5V VREF)")
    print("=" * 56)
    print(f"  Connect AIN{channel_pos} to the V3 VREF test pad, then press Enter...")
    input()

    ads.write_register(ads.REG_DRATE, NOISE_DRATE)
    ads.self_calibrate()

    v = _make_verifier()
    r = v.analyse_dc(channel_pos)
    gain_err_v   = r["mean_v"] - 2.5
    gain_err_ppm = (gain_err_v / 2.5) * 1e6
    ADS1256SineVerifier.print_dc_result(r, f"AIN{channel_pos} → VREF")
    print(f"  Gain error : {gain_err_v*1000:.4f} mV  ({gain_err_ppm:.1f} ppm)")


def run_sine_test():
    print("\n" + "=" * 56)
    print("  STEP 3: Sine Wave ENOB / SNR / THD")
    print(f"  GP{SIGNAL_PIN} → RC filter → AIN0 (daisy to AIN1-7 optional)")
    print("=" * 56)

    n_cycles_expected = _fundamental_hz * (ADS_N_SAMPLES / 1000.0)
    print(f"  Fundamental  : {_fundamental_hz:.2f} Hz  (harmonic {HARMONIC})")
    print(f"  Data rate    : 1000 SPS  →  {n_cycles_expected:.1f} cycles in {ADS_N_SAMPLES} samples")
    print("  Wire signal and press Enter...")
    input()

    ads.write_register(ads.REG_DRATE, SINE_DRATE)
    ads.self_calibrate()

    gen = PWMSineGenerator(
        n_samples=N_LUT,
        tick_hz=TICK_HZ,
        carrier_hz=CARRIER_HZ,
        dc_offset=0.5,
        amplitude=0.45,
    )
    gen.add_channel(pin=SIGNAL_PIN, harmonic=HARMONIC)
    gen.start()
    print(f"  Generator running — settling {SETTLE_MS} ms...")
    time.sleep_ms(SETTLE_MS)

    v       = _make_verifier()
    results = []
    for pos, neg in ADS_CHANNELS:
        r = v.analyse_sine(pos, neg,
                           expected_freq_hz=_fundamental_hz * HARMONIC,
                           sample_rate_hz=1000.0)   # pass known rate, not estimated
        ADS1256SineVerifier.print_sine_result(r, expected_enob=EXPECTED_ENOB)
        results.append(r)

    gen.deinit()

    if len(results) > 1:
        print("\n  Channel-to-Channel Comparison:")
        ADS1256SineVerifier.print_channel_comparison(results, expected_enob=EXPECTED_ENOB)

        enobs    = [r["enob"]        for r in results]
        amps_mv  = [r["amplitude_v"] * 1000 for r in results]
        dcs_mv   = [r["dc_offset_v"] * 1000 for r in results]
        print(f"  ENOB range   : {min(enobs):.2f} – {max(enobs):.2f} bits")
        print(f"  Amp spread   : {max(amps_mv)-min(amps_mv):.3f} mV pk-pk across channels")
        print(f"  DC spread    : {max(dcs_mv)-min(dcs_mv):.3f} mV  (offset mismatch)")


def run_all():
    print("\n╔" + "═" * 54 + "╗")
    print("║    ADS1256 Full Performance Verification  (v2)    ║")
    print("╚" + "═" * 54 + "╝")
    ads.hardware_reset()
    ads.stop_continuous()
    ads.check_status_register()
    run_noise_floor_test(channel_pos=0)
    run_full_scale_test(channel_pos=0)
    run_sine_test()
    print("\n[DONE] All tests complete.")


if __name__ == "__main__":
    run_all()
