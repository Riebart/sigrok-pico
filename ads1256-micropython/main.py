from ads1256sineverifier import ADS1256SineVerifier
from pwmsine import PWMSineGenerator

from ads1256 import *

import time

write_register(REG_DRATE, 0x23)   # 10 SPS
run_test()
print(read_channel_volts(0))

gen = PWMSineGenerator(
    n_samples=256,
    tick_hz=64,          # 256 / 256 = 1 Hz fundamental
    carrier_hz=100_000,   # 100 kHz PWM carrier
    dc_offset=0.5,
    amplitude=0.45,
)

gen.add_channel(pin=28, harmonic=1)
gen.start()

for i in range(20):
    print(read_channel_volts(0))
    s = 0
    for i in range(40):
        s += read_channel_volts(0)
        time.sleep_ms(10)
    print(s/40.0)

v = ADS1256SineVerifier(n_samples=512, avdd=5.0)
r = v.analyse_sine(
    0,
    expected_freq_hz=0.25,
    sample_rate_hz=10.0    # 10 SPS → 512 samples = 51.2 seconds = 12.8 cycles
)
print(r)
#
# gen = PWMSineGenerator(n_samples=256, tick_hz=15_625)
# gen.add_channel(pin=28, harmonic=1)
# gen.start()
#
# v = ADS1256SineVerifier(n_samples=512, avdd=3.3)
#
# v.debug_confirm(
#     gpio_pin=26,
#     n_samples=16384,
#     sample_rate_hz=5000,        # << well below carrier alias
#     expected_freq_hz=61.0
# )
#
# v.debug_confirm2(
#     gpio_pin=26,
#     n_samples=16384,
#     sample_rate_hz=5000,        # << well below carrier alias
#     expected_freq_hz=61.0
# )
#
# #r = v.analyse_sine(0, expected_freq_hz=61.0, sample_rate_hz=1000.0)
#
# #import sine_verifier
# #sine_verifier.run_all()
#
