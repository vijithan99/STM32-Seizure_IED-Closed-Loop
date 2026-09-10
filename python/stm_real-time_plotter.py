# -*- coding: utf-8 -*-
"""
Created on Tue Aug 18 12:42:50 2026

@author: vijit
"""

import struct
import re

import serial
from serial.tools import list_ports

import csv
from pathlib import Path

import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

PORT = "COM4"
BAUD = 921600

packet_count = 0
last_sequence = None
report_time = time.monotonic()

# 5000-Hz acquisition divided by UART_STREAM_DECIMATION=5.
STREAM_FS = 5000
DISPLAY_SECONDS = 10

SYNC_VALUE = 0xA55A
SYNC_BYTES = b"\x5A\xA5"       # STM32 little-endian representation
FRAME = struct.Struct("<HHhh")  # sync, sequence, chA, chB

CSV_PATH = Path(
    f"rhs_capture_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)

csv_file = CSV_PATH.open("w", newline="")
csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "received_index",
    "stream_time_s",
    "host_elapsed_s",
    "sequence",
    "missing_packets_before",
    "channel_a_centered_counts",
    "channel_b_centered_counts",
    "channel_a_raw_u16",
    "channel_b_raw_u16",
    "channel_a_uV",
    "channel_b_uV",
])

received_index = 0
stream_index = 0
total_missing = 0
capture_start = time.monotonic()

serial_port = serial.Serial(PORT, BAUD, timeout=0)
serial_port.reset_input_buffer()

print(f"Saving data to: {CSV_PATH.resolve()}")

# serial_port = serial.Serial(PORT, BAUD, timeout=0)

number_displayed = STREAM_FS * DISPLAY_SECONDS
time_axis = np.arange(-number_displayed, 0) / STREAM_FS

channel_a = np.zeros(number_displayed, dtype=np.float32)
channel_b = np.zeros(number_displayed, dtype=np.float32)

receive_buffer = bytearray()

# -------------------------------------------------------------------------
# RHS2116 ROM-test parsing
# -------------------------------------------------------------------------

ROM_REGISTER_RE = re.compile(
    r"Register\s+(251|252|253|254|255):\s*0x([0-9A-Fa-f]+)"
)

ROM_ATTEMPT_RE = re.compile(
    r"ROM test attempt\s+(\d+)"
)

ROM_RESULT_RE = re.compile(
    r"ROM test:\s*(PASS|FAIL)"
)

rom_attempt = None
rom_registers = {}
rom_firmware_pass = None

app = pg.mkQApp("RHS2116 Real-Time AC Viewer")

plot = pg.plot(title="RHS2116 AC amplifier data")

# Keep units fixed rather than allowing PyQtGraph to
# automatically change s -> ms or µV -> mµV/kµV.
plot.setLabel("bottom", "Time", units="s")
plot.setLabel("left", "Electrode voltage", units="µV")

plot.getAxis("bottom").enableAutoSIPrefix(False)
plot.getAxis("left").enableAutoSIPrefix(False)

plot.showGrid(x=True, y=True)
plot.addLegend()

curve_a = plot.plot(time_axis, channel_a, pen="y", name="Channel 0")
curve_b = plot.plot(time_axis, channel_b, pen="c", name="Channel 1")

def show_serial_ports() -> None:
    """Print the serial ports currently visible to the computer."""
    ports = list(list_ports.comports())

    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")

    for port in ports:
        print(
            f"  {port.device}: {port.description} "
            f"[VID={port.vid}, PID={port.pid}]"
        )


def append_samples(destination, new_values):
    count = min(len(new_values), len(destination))

    if count == 0:
        return

    destination[:-count] = destination[count:]
    destination[-count:] = np.asarray(new_values[-count:], dtype=np.float32)
    
    
def check_rom_registers():
    """
    Independently verify the RHS2116 ROM response on the PC side.

    Returns
    -------
    True
        Complete ROM response is present and valid.
    False
        Complete ROM response is present but invalid.
    None
        Not all five ROM registers have been received yet.
    """

    required = {251, 252, 253, 254, 255}

    if not required.issubset(rom_registers):
        return None

    return (
        rom_registers[255] == 0x0020
        and (rom_registers[254] & 0x00FF) == 0x0010
        and rom_registers[253] == 0x4E00
        and rom_registers[252] == 0x5441
        and rom_registers[251] == 0x494E
    )


def print_rom_summary():
    """Decode and print the RHS2116 ROM information."""

    if not {251, 252, 253, 254, 255}.issubset(rom_registers):
        return

    chip_id = rom_registers[255]

    revision_channels = rom_registers[254]
    die_revision = (revision_channels >> 8) & 0xFF
    number_channels = revision_channels & 0xFF

    company_bytes = []

    for register in (251, 252, 253):
        value = rom_registers[register]

        company_bytes.append((value >> 8) & 0xFF)
        company_bytes.append(value & 0xFF)

    company = (
        bytes(company_bytes)
        .rstrip(b"\x00")
        .decode("ascii", errors="replace")
    )

    print()
    print("========== RHS2116 ROM SUMMARY ==========")
    print(f"Chip ID:        0x{chip_id:04X}")
    print(f"Channels:       {number_channels}")
    print(f"Die revision:   {die_revision}")
    print(f"Manufacturer:   {company}")
    print("=========================================")
    print()


def handle_mcu_text_line(line):
    """
    Handle human-readable UART output from the STM32.

    This includes:
        Starting RHS2116 SPI ROM test...
        ROM test attempt 1...
        Register 255: 0x0020
        ...
        ROM test: PASS
    """

    global rom_attempt
    global rom_registers
    global rom_firmware_pass

    line = line.strip()

    if not line:
        return

    print(f"[MCU] {line}")

    # New ROM-test attempt.
    match = ROM_ATTEMPT_RE.search(line)

    if match:
        rom_attempt = int(match.group(1))
        rom_registers = {}

        print(f"[ROM] Beginning attempt {rom_attempt}")
        return

    # ROM register result.
    match = ROM_REGISTER_RE.search(line)

    if match:
        register = int(match.group(1))
        value = int(match.group(2), 16)

        rom_registers[register] = value
        return

    # Final MCU ROM result.
    match = ROM_RESULT_RE.search(line)

    if match:
        rom_firmware_pass = match.group(1) == "PASS"

        host_check = check_rom_registers()

        print_rom_summary()

        print(
            f"[ROM] MCU result: "
            f"{'PASS' if rom_firmware_pass else 'FAIL'}"
        )

        if host_check is None:
            print(
                "[ROM] PC verification unavailable "
                "(not all ROM register lines were received)."
            )

        elif host_check:
            print("[ROM] PC verification: PASS")

        else:
            print("[ROM] PC verification: FAIL")

        # Update the graph title so ROM status is always visible.
        if rom_firmware_pass and host_check is not False:
            plot.setTitle(
                "RHS2116 AC amplifier data — ROM TEST PASS"
            )
        else:
            plot.setTitle(
                "RHS2116 AC amplifier data — ROM TEST FAIL"
            )


def process_text_messages():
    """
    Consume complete ASCII diagnostic lines that appear before the next
    binary RHS sample frame.

    Binary sample frames always begin with 0xA55A.
    """

    while True:

        sync_index = receive_buffer.find(SYNC_BYTES)
        newline_index = receive_buffer.find(b"\n")

        if newline_index < 0:
            return

        # If a binary frame begins before the newline, leave it for the
        # binary frame parser.
        if sync_index >= 0 and sync_index < newline_index:
            return

        raw_line = bytes(
            receive_buffer[:newline_index + 1]
        )

        del receive_buffer[:newline_index + 1]

        payload = raw_line.strip(b"\r\n")

        if not payload:
            continue

        # Only interpret the data as text when it is predominantly
        # printable ASCII. This prevents arbitrary ADC bytes from being
        # interpreted as MCU messages.
        printable = sum(
            32 <= byte <= 126 or byte == 9
            for byte in payload
        )

        if printable / len(payload) > 0.90:
            line = payload.decode(
                "ascii",
                errors="replace"
            )

            handle_mcu_text_line(line)


def update_plot():
    global packet_count
    global last_sequence
    global report_time
    global received_index
    global stream_index
    global total_missing

    available = serial_port.in_waiting

    if available:
        receive_buffer.extend(serial_port.read(available))

    samples_a = []
    samples_b = []
    csv_rows = []

    while receive_buffer:

        # First see whether the MCU sent us a complete text diagnostic
        # message such as a ROM-test result.
        process_text_messages()
    
        if len(receive_buffer) < FRAME.size:
            break
    
        sync_index = receive_buffer.find(SYNC_BYTES)
    
        if sync_index < 0:
    
            # There may be a partial ASCII diagnostic message waiting for its
            # newline, so don't immediately throw the whole buffer away.
            #
            # Only protect against pathological unlimited buffer growth.
            if len(receive_buffer) > 65536:
                del receive_buffer[:-1]
    
            break
    
        if sync_index > 0:
            # Anything preceding a binary sync word that was not recognized
            # as a diagnostic message is discarded.
            del receive_buffer[:sync_index]
    
        if len(receive_buffer) < FRAME.size:
            break
    
        sync, sequence, ac_a, ac_b = FRAME.unpack_from(receive_buffer)
    
        if sync != SYNC_VALUE:
            del receive_buffer[0]
            continue
    
        del receive_buffer[:FRAME.size]

        missing_packets = 0

        if last_sequence is not None:
            sequence_delta = (sequence - last_sequence) & 0xFFFF

            if 0 < sequence_delta < 0x8000:
                missing_packets = sequence_delta - 1
                stream_index += sequence_delta
                total_missing += missing_packets
            else:
                # Duplicate, out-of-order packet, or MCU sequence reset.
                missing_packets = -1
                stream_index += 1

        # Recover the unsigned RHS ADC result from the centred int16.
        raw_a = (ac_a + 32768) & 0xFFFF
        raw_b = (ac_b + 32768) & 0xFFFF

        voltage_a_uV = ac_a * 0.195
        voltage_b_uV = ac_b * 0.195

        samples_a.append(voltage_a_uV)
        samples_b.append(voltage_b_uV)

        csv_rows.append([
            received_index,
            stream_index / STREAM_FS,
            time.monotonic() - capture_start,
            sequence,
            missing_packets,
            ac_a,
            ac_b,
            raw_a,
            raw_b,
            voltage_a_uV,
            voltage_b_uV,
        ])

        received_index += 1
        packet_count += 1
        last_sequence = sequence

    if csv_rows:
        # Write one batch rather than writing each sample separately.
        csv_writer.writerows(csv_rows)

    if samples_a:
        append_samples(channel_a, samples_a)
        append_samples(channel_b, samples_b)

        curve_a.setData(time_axis, channel_a)
        curve_b.setData(time_axis, channel_b)

    now = time.monotonic()

    if now - report_time >= 1.0:
        csv_file.flush()

        print(
            f"Packets/s: {packet_count}, "
            f"last sequence: {last_sequence}, "
            f"total missing: {total_missing}, "
            f"buffered bytes: {len(receive_buffer)}"
        )

        packet_count = 0
        report_time = now

show_serial_ports()

timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(20)

try:
    pg.exec()
finally:
    csv_file.flush()
    csv_file.close()
    serial_port.close()

    print(f"Capture saved to: {CSV_PATH.resolve()}")