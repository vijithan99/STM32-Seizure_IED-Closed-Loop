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
GAIN = 1000       # scale float ABF values into int range
BLOCK_SIZE = 500    # match your MCU test for now

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
# Plot
# ----------------------------
plt.figure()
plt.plot(dataX, dataY)
plt.title(os.path.basename(abf_path))
plt.xlabel("Time (s)")
plt.ylabel("Signal")
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
            line = ser.readline()
            if not line:
                break
            print("STM says:", line.decode(errors="ignore").strip())

        # Simulate real-time playback
        time.sleep(block_duration)

finally:
    ser.close()