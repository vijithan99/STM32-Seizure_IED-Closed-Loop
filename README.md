# STM32 Closed-Loop Seizure Detection System

Real-time seizure detection pipeline implemented on an STM32H723 microcontroller using UART-streamed and SPI iEEG data and block-wise feature extraction.

This project aims to perform closed-loop neurostimulation to disrupt seizures

---

## Features

- Real-time UART data streaming (Python → STM32)
- Interrupt-driven data acquisition (HAL UART)
- Ping-pong buffer architecture for continuous processing
- Block-wise signal processing (low-latency)
- Line Length (LL) feature extraction
- Adaptive thresholding using EMA (mean + variance)
- Persistence-based seizure detection logic
- Refractory period to prevent over-triggering
- Live monitoring via serial console and Python visualization

---

## Detection Algorithm

The system processes incoming EEG data in fixed-size blocks:

1. Compute Line Length:
   - Measures signal activity via absolute differences
2. Estimate baseline:
   - Exponential Moving Average (mean + variance)
3. Compute adaptive threshold:
   - `threshold = mean + k * std`
4. Detection logic:
   - Trigger event if signal exceeds threshold for N consecutive blocks
5. Apply refractory period:
   - Prevents repeated detections

---

## System Architecture
