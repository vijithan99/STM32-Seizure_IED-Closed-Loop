# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 11:44:34 2026

@author: vijit
"""

from pathlib import Path
from typing import Optional

import serial
from serial import SerialException
from serial.tools import list_ports


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


def view_stm32_printf(
    port: str,
    baudrate: int = 921600,
    log_path: Optional[str] = None,
) -> None:
    """
    Display newline-terminated printf output received from an STM32.

    Press Ctrl+C to stop.

    Parameters
    ----------
    port:
        Windows serial-port name, such as "COM5".
    baudrate:
        Must match the STM32 UART baud rate.
    log_path:
        Optional text-file path where received messages will also be saved.
    """
    log_file = None

    try:
        if log_path is not None:
            path = Path(log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            log_file = path.open("a", encoding="utf-8", buffering=1)

        with serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.25,
        ) as connection:
            connection.reset_input_buffer()

            print(f"Connected to {port} at {baudrate} baud.")
            print("Reset the STM32 to capture its startup messages.")
            print("Press Ctrl+C to stop.\n")

            while True:
                raw_line = connection.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(
                    "utf-8",
                    errors="replace",
                ).rstrip("\r\n")

                print(line, flush=True)

                if log_file is not None:
                    log_file.write(line + "\n")

    except KeyboardInterrupt:
        print("\nSerial monitor stopped.")

    except SerialException as error:
        print(f"Serial-port error: {error}")
        print(
            "Check the COM-port name and close CubeIDE, PuTTY, "
            "or any other application using the port."
        )

    finally:
        if log_file is not None:
            log_file.close()
            
            
show_serial_ports()

view_stm32_printf(
    port="COM4",
    baudrate=921600,
    log_path="logs/stm32_output.txt",
)