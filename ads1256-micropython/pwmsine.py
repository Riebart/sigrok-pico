"""
PWMSineGenerator — MicroPython class for RP2040 Zero
=====================================================
Generates sine waves on up to 8 GPIO pins simultaneously using hardware PWM
and a Timer interrupt to step through a shared lookup table.

Architecture
------------
  • One shared sine LUT (stored as a bytearray to minimise SRAM usage)
  • One hardware Timer interrupt drives ALL active channels at the same tick rate
  • Each channel has an independent phase accumulator and frequency multiplier
    so channels can run at different harmonics of the fundamental
  • RP2040 has 8 PWM slices (GP0/1, GP2/3, GP4/5, GP6/7, GP8/9, GP10/11,
    GP12/13, GP14/15) — each pin maps to a fixed slice, so pin choices matter

PWM carrier frequency
---------------------
  carrier_freq = sys_clk / wrap_count
  wrap_count   = 65535 (full 16-bit range used for best duty resolution)
  sys_clk      = 125 MHz
  → carrier    ≈ 1907 Hz  ... but that is too low.

  For a >60 kHz carrier we need a smaller wrap. We use wrap=2000:
  carrier = 125_000_000 / 2000 = 62_500 Hz  ✓

  duty_u16 values 0–65535 are automatically scaled to 0–wrap by MicroPython
  when using duty_u16(), so we can keep the LUT in 0–65535 space.

Sine LUT memory estimate
------------------------
  N=256  samples × 2 bytes (uint16) = 512 bytes  — very comfortable
  N=512  samples × 2 bytes          = 1024 bytes
  Default: 256 samples. Changeable at construction.

Fundamental frequency
---------------------
  f_fundamental = timer_tick_rate / N_samples
  timer_tick_rate is set at construction (default 15_625 Hz → fund ≈ 61 Hz at N=256)
  A channel at harmonic k runs at k × f_fundamental.

Usage example
-------------
  from pwm_sine import PWMSineGenerator
  gen = PWMSineGenerator(n_samples=256, tick_hz=15_625)
  gen.add_channel(pin=0, harmonic=1)   # ~61 Hz on GP0
  gen.add_channel(pin=2, harmonic=2)   # ~122 Hz on GP2
  gen.add_channel(pin=4, harmonic=4)   # ~244 Hz on GP4
  gen.start()
  # ... later ...
  gen.stop()
"""

import math
import array
from machine import Pin, PWM, Timer


class PWMSineGenerator:
    """
    Multi-channel PWM sine wave generator for RP2040.

    Parameters
    ----------
    n_samples   : int   — LUT size (must be power of 2; default 256)
    tick_hz     : int   — Timer interrupt rate in Hz (default 15_625)
                          fundamental = tick_hz / n_samples
    carrier_hz  : int   — PWM carrier frequency in Hz (default 62_500)
    dc_offset   : float — Fraction of full-scale added to sine (0.5 = mid-rail)
                          Set 0.5 for single-supply operation (0 → 3.3V swing)
    amplitude   : float — Fraction of full-scale for sine amplitude (default 0.45)
                          amplitude + dc_offset must be ≤ 1.0
    """

    MAX_CHANNELS = 8

    def __init__(self,
                 n_samples=256,
                 tick_hz=15_625,
                 carrier_hz=62_500,
                 dc_offset=0.5,
                 amplitude=0.45):

        # ── Validate ──────────────────────────────────────────────────────────
        if n_samples < 16 or (n_samples & (n_samples - 1)):
            raise ValueError("n_samples must be a power of 2, minimum 16")
        if amplitude + dc_offset > 1.0:
            raise ValueError("amplitude + dc_offset exceeds 1.0 (clipping)")

        self.n_samples  = n_samples
        self.tick_hz    = tick_hz
        self.carrier_hz = carrier_hz
        self.dc_offset  = dc_offset
        self.amplitude  = amplitude

        self._fundamental_hz = tick_hz / n_samples

        # ── Build LUT as array of unsigned 16-bit ints ────────────────────────
        # Values span 0–65535. DC offset shifts centre, amplitude scales swing.
        # Using array('H', ...) = uint16 → 2 bytes/sample
        half   = 32767.5
        amp_c  = amplitude * half
        off_c  = dc_offset * 65535
        self._lut = array.array('H', (
            int(off_c + amp_c * math.sin(2 * math.pi * i / n_samples))
            for i in range(n_samples)
        ))

        # ── Channel state ─────────────────────────────────────────────────────
        # Parallel arrays for speed in the ISR — avoids attribute lookup on objects
        self._pwm       = [None] * self.MAX_CHANNELS   # machine.PWM instances
        self._phase     = array.array('H', [0] * self.MAX_CHANNELS)  # current LUT index
        self._step      = array.array('H', [0] * self.MAX_CHANNELS)  # phase increment
        self._active    = array.array('b', [0] * self.MAX_CHANNELS)  # 1 if slot in use
        self._n_active  = 0

        # ── Timer (not started yet) ───────────────────────────────────────────
        self._timer    = Timer()
        self._running  = False

        print(f"[PWMSine] LUT: {n_samples} samples, "
              f"{n_samples * 2} bytes, "
              f"fundamental = {self._fundamental_hz:.2f} Hz")
        print(f"[PWMSine] Carrier: {carrier_hz} Hz  |  "
              f"Tick: {tick_hz} Hz  |  "
              f"DC offset: {dc_offset:.0%}  Amplitude: {amplitude:.0%}")

    # ── Public API ─────────────────────────────────────────────────────────────

    @property
    def fundamental_hz(self):
        """Fundamental frequency in Hz (read-only)."""
        return self._fundamental_hz

    def add_channel(self, pin, harmonic=1, phase_offset=0):
        """
        Configure a GPIO pin as a PWM sine output.

        Parameters
        ----------
        pin          : int   — GPIO number (0–28)
        harmonic     : int   — Frequency multiplier of fundamental (≥1)
        phase_offset : float — Initial phase offset in degrees (0–360)

        Returns
        -------
        int — channel slot index (needed for remove_channel)
        """
        if harmonic < 1:
            raise ValueError("harmonic must be >= 1")
        if self._n_active >= self.MAX_CHANNELS:
            raise RuntimeError("Maximum 8 channels already registered")

        # Find first free slot
        slot = next(i for i in range(self.MAX_CHANNELS) if not self._active[i])

        # Initialise PWM on this pin at the carrier frequency
        pwm = PWM(Pin(pin))
        pwm.freq(self.carrier_hz)
        pwm.duty_u16(self._lut[0])
        self._pwm[slot]    = pwm

        # Phase step: each timer tick advances by `harmonic` LUT indices
        self._step[slot]   = harmonic % self.n_samples

        # Phase offset — convert degrees to LUT index
        self._phase[slot]  = int((phase_offset / 360.0) * self.n_samples) % self.n_samples

        self._active[slot] = 1
        self._n_active    += 1

        freq = self._fundamental_hz * harmonic
        print(f"[PWMSine] Channel {slot}: GP{pin}, harmonic {harmonic} "
              f"(≈{freq:.2f} Hz), phase offset {phase_offset}°")
        return slot

    def remove_channel(self, slot):
        """Stop and release a channel by slot index."""
        if self._active[slot]:
            self._pwm[slot].deinit()
            self._pwm[slot]    = None
            self._active[slot] = 0
            self._step[slot]   = 0
            self._phase[slot]  = 0
            self._n_active    -= 1
            print(f"[PWMSine] Channel {slot} removed.")

    def set_harmonic(self, slot, harmonic):
        """
        Change the harmonic (frequency multiplier) of a running channel.
        Safe to call while the generator is running.
        """
        if not self._active[slot]:
            raise ValueError(f"Slot {slot} is not active")
        self._step[slot] = harmonic % self.n_samples
        freq = self._fundamental_hz * harmonic
        print(f"[PWMSine] Channel {slot} → harmonic {harmonic} (≈{freq:.2f} Hz)")

    def set_phase_offset(self, slot, degrees):
        """Set the phase offset of a channel in degrees without stopping it."""
        if not self._active[slot]:
            raise ValueError(f"Slot {slot} is not active")
        self._phase[slot] = int((degrees / 360.0) * self.n_samples) % self.n_samples

    def start(self):
        """Start the timer interrupt and begin generating waveforms."""
        if self._running:
            return
        if self._n_active == 0:
            raise RuntimeError("No channels configured — call add_channel() first")
        self._running = True
        self._timer.init(freq=self.tick_hz,
                         mode=Timer.PERIODIC,
                         callback=self._tick_isr)
        print(f"[PWMSine] Started — {self._n_active} channel(s) active.")

    def stop(self):
        """Stop the timer interrupt and silence all outputs."""
        if not self._running:
            return
        self._timer.deinit()
        self._running = False
        # Drive all active channels to mid-scale silence
        mid = self._lut[0]  # first LUT entry = dc_offset at phase 0 ≈ mid-scale
        for i in range(self.MAX_CHANNELS):
            if self._active[i]:
                self._pwm[i].duty_u16(32768)  # exact mid-rail
        print("[PWMSine] Stopped.")

    def deinit(self):
        """Stop the generator and release all PWM peripherals."""
        self.stop()
        for i in range(self.MAX_CHANNELS):
            if self._active[i]:
                self.remove_channel(i)
        print("[PWMSine] Deinitialized.")

    def status(self):
        """Print current channel status table."""
        print(f"\n{'Slot':<6} {'Pin':<6} {'Harmonic':<10} {'Freq (Hz)':<12} {'Phase idx':<10}")
        print("-" * 48)
        for i in range(self.MAX_CHANNELS):
            if self._active[i]:
                freq = self._fundamental_hz * self._step[i]
                # Recover pin number from PWM object repr (best effort)
                try:
                    pin_num = self._pwm[i].pin()
                except Exception:
                    pin_num = "?"
                print(f"{i:<6} {str(pin_num):<6} {self._step[i]:<10} "
                      f"{freq:<12.2f} {self._phase[i]:<10}")
        print()

    # ── ISR — called at tick_hz rate ──────────────────────────────────────────
    # Keep this as tight as possible; no allocations, no prints.

    def _tick_isr(self, t):
        lut    = self._lut
        phase  = self._phase
        step   = self._step
        active = self._active
        pwm    = self._pwm
        n      = self.n_samples

        for i in range(8):
            if active[i]:
                p = (phase[i] + step[i]) % n
                phase[i] = p
                pwm[i].duty_u16(lut[p])
