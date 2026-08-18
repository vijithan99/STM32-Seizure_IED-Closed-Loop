# -*- coding: utf-8 -*-
"""
Created on Tue Aug 18 12:42:50 2026

@author: vijit
"""

import struct

import serial
from serial.tools import list_ports

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

PORT = "COM4"
BAUD = 921600

# 5000-Hz acquisition divided by UART_STREAM_DECIMATION=5.
STREAM_FS = 1000
DISPLAY_SECONDS = 3

SYNC_VALUE = 0xA55A
SYNC_BYTES = b"\x5A\xA5"       # STM32 little-endian representation
FRAME = struct.Struct("<HHhh")  # sync, sequence, chA, chB

serial_port = serial.Serial(PORT, BAUD, timeout=0)

number_displayed = STREAM_FS * DISPLAY_SECONDS
time_axis = np.arange(-number_displayed, 0) / STREAM_FS

channel_a = np.zeros(number_displayed, dtype=np.float32)
channel_b = np.zeros(number_displayed, dtype=np.float32)

receive_buffer = bytearray()

app = pg.mkQApp("RHS2116 Real-Time AC Viewer")

plot = pg.plot(title="RHS2116 AC amplifier data")
plot.setLabel("bottom", "Time", units="s")
plot.setLabel("left", "Electrode voltage", units="µV")
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


def update_plot():
    available = serial_port.in_waiting

    if available:
        receive_buffer.extend(serial_port.read(available))

    samples_a = []
    samples_b = []

    while len(receive_buffer) >= FRAME.size:
        sync_index = receive_buffer.find(SYNC_BYTES)

        if sync_index < 0:
            # Preserve one byte in case it is the start of a split sync word.
            del receive_buffer[:-1]
            break

        if sync_index > 0:
            del receive_buffer[:sync_index]

        if len(receive_buffer) < FRAME.size:
            break

        sync, sequence, ac_a, ac_b = FRAME.unpack_from(receive_buffer)

        if sync != SYNC_VALUE:
            del receive_buffer[0]
            continue

        del receive_buffer[:FRAME.size]

        # Convert centered ADC counts into microvolts.
        samples_a.append(ac_a * 0.195)
        samples_b.append(ac_b * 0.195)

    if samples_a:
        append_samples(channel_a, samples_a)
        append_samples(channel_b, samples_b)

        curve_a.setData(time_axis, channel_a)
        curve_b.setData(time_axis, channel_b)

show_serial_ports()

timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(20)

try:
    pg.exec()
finally:
    serial_port.close()