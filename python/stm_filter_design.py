# -*- coding: utf-8 -*-
"""
Created on Fri May 29 14:20:56 2026

@author: vijit
"""

import numpy as np
from scipy import signal

# -----------------------------
# User settings
# -----------------------------
FS_HZ = 5000          # Change to 20000 if using 20 kHz sampling
LOWCUT_HZ = 50
HIGHCUT_HZ = 100
FILTER_ORDER = 4      # 4th-order Butterworth = 2 SOS biquad sections

# -----------------------------
# Generate filter
# -----------------------------
sos = signal.butter(
    N=FILTER_ORDER,
    Wn=[LOWCUT_HZ, HIGHCUT_HZ],
    btype="bandpass",
    fs=FS_HZ,
    output="sos"
)

print("SOS coefficients:")
print(sos)

# -----------------------------
# Print C-friendly coefficients
# Each SOS row:
# [b0, b1, b2, a0, a1, a2]
#
# scipy gives a0 = 1 usually.
# Embedded biquad usually uses:
# y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
#        - a1*y[n-1] - a2*y[n-2]
# -----------------------------

print("\nC coefficients for manual biquad implementation:")
print("typedef struct {")
print("    float b0, b1, b2;")
print("    float a1, a2;")
print("    float x1, x2;")
print("    float y1, y2;")
print("} biquad_t;\n")

print(f"#define IED_BANDPASS_NUM_SECTIONS {sos.shape[0]}\n")

print("static biquad_t ied_bandpass_50_85[IED_BANDPASS_NUM_SECTIONS] = {")
for section in sos:
    b0, b1, b2, a0, a1, a2 = section

    # Normalize just in case a0 is not exactly 1
    b0 /= a0
    b1 /= a0
    b2 /= a0
    a1 /= a0
    a2 /= a0

    print(
        f"    {{ "
        f"{b0:.9e}f, {b1:.9e}f, {b2:.9e}f, "
        f"{a1:.9e}f, {a2:.9e}f, "
        f"0.0f, 0.0f, 0.0f, 0.0f "
        f"}},"
    )
print("};")

# -----------------------------
# Optional: inspect response
# -----------------------------
w, h = signal.sosfreqz(sos, worN=4096, fs=FS_HZ)

# Print basic sanity check
passband = (w >= LOWCUT_HZ) & (w <= HIGHCUT_HZ)
peak_gain_db = 20 * np.log10(np.max(np.abs(h)))
mean_passband_gain_db = 20 * np.log10(np.mean(np.abs(h[passband])))

print("\nSanity check:")
print(f"Sampling rate: {FS_HZ} Hz")
print(f"Bandpass: {LOWCUT_HZ}-{HIGHCUT_HZ} Hz")
print(f"Filter order: {FILTER_ORDER}")
print(f"Number of SOS sections: {sos.shape[0]}")
print(f"Peak gain: {peak_gain_db:.3f} dB")
print(f"Mean passband gain: {mean_passband_gain_db:.3f} dB")