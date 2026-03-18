import os
import time
import serial
import matplotlib.pyplot as plt
import pyabf
import numpy as np

# ----------------------------
# Serial Port Initialization
# ----------------------------
ser = serial.Serial("COM4", 921600, timeout=0.1)
time.sleep(2)
ser.reset_input_buffer()
ser.reset_output_buffer()

# ----------------------------
# ABF path
# ----------------------------
current_dir = os.getcwd()
abf_dir = os.path.join(current_dir, "seizureData")

# ----------------------------
# Settings
# ----------------------------
GAIN = 1000
BLOCK_SIZE = 1000

DETECTED_TIMES = []

def line_parse(line):
    segments = line.split(',')
    if len(segments) < 2:
        return None

    try:
        time_us = int(segments[1].strip())
    except ValueError:
        return None

    return time_us / 1e6

def convert_sample_to_24bit_bytes(sample_float):
    """
    Convert a float sample to signed 24-bit little-endian bytes.
    """
    sample_i = int(round(sample_float * GAIN))

    # Clip into signed 24-bit range
    if sample_i > 8388607:
        sample_i = 8388607
    elif sample_i < -8388608:
        sample_i = -8388608

    # Convert negative values to 24-bit two's complement
    if sample_i < 0:
        sample_i = (1 << 24) + sample_i

    b0 = sample_i & 0xFF
    b1 = (sample_i >> 8) & 0xFF
    b2 = (sample_i >> 16) & 0xFF

    return b0, b1, b2

# ----------------------------
# Find one ABF and use it
# ----------------------------
abf_files = []
for dirpath, dirnames, filenames in os.walk(abf_dir):
    for filename in filenames:
        if filename.endswith(".abf"):
            abf_files.append(os.path.join(dirpath, filename))

if not abf_files:
    raise FileNotFoundError("No ABF files found in seizureData")

abf_path = abf_files[0]
print("Using file:", abf_path)

abf = pyabf.ABF(abf_path)
abf.setSweep(0)

dataY = abf.sweepY.copy()
dataX = abf.sweepX.copy()
fs = abf.dataRate

print("adcNames:", getattr(abf, "adcNames", None))
print("adcUnits:", getattr(abf, "adcUnits", None))
print("Sampling rate:", fs)
print("Total samples:", len(dataY))

# ----------------------------
# Plot original waveform
# ----------------------------
plt.figure(figsize=(12, 4))
plt.plot(dataX, dataY, linewidth=0.8)
plt.title(os.path.basename(abf_path))
plt.xlabel("Time (s)")
plt.ylabel("Signal")
plt.tight_layout()
plt.show(block=False)

# ----------------------------
# Stream data block by block
# ----------------------------
block_duration = BLOCK_SIZE / fs

try:
    for start in range(0, len(dataY), BLOCK_SIZE):
        stop = min(start + BLOCK_SIZE, len(dataY))
        block = dataY[start:stop]

        payload = bytearray()
        for sample in block:
            b0, b1, b2 = convert_sample_to_24bit_bytes(sample)
            payload.extend([b0, b1, b2])

        print(f"Sending block {start}:{stop}")
        ser.write(payload)
        ser.flush()

        # Read back anything STM printed
        while ser.in_waiting > 0:
            raw_line = ser.readline()
            if not raw_line:
                break

            line = raw_line.decode(errors="ignore").strip()
            print("STM says:", line)

            detect_time = line_parse(line)
            if detect_time is not None:
                DETECTED_TIMES.append(detect_time)

        # Simulate real-time playback
        time.sleep(block_duration)

finally:
    ser.close()

# ----------------------------
# Plot waveform with vertical detection lines
# ----------------------------
print("Detected times (s):", DETECTED_TIMES)

fig, ax = plt.subplots(figsize=(14, 5))
ax.plot(dataX, dataY, linewidth=0.8, label="Waveform")

# only draw lines that fall within the ABF time range
valid_detect_times = [t for t in DETECTED_TIMES if dataX[0] <= t <= dataX[-1]]

for i, t in enumerate(valid_detect_times):
    if i == 0:
        ax.axvline(x=t, linestyle="--", linewidth=1.2, label="STM detection", c="red")
    else:
        ax.axvline(x=t, linestyle="--", linewidth=1.2, c="red")

ax.set_title(f"Waveform with STM32 seizure detections: {os.path.basename(abf_path)}")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Signal")
ax.legend()

fig.tight_layout() 
plt.show()