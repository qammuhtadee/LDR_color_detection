# LDR Color Detector

An Arduino-based color detection system that uses an **LDR and RGB illumination** to estimate the color of a surface. The system combines sensor calibration, multi-sample acquisition, normalization, gamma correction, and **Euclidean-distance color classification** with a menu-driven OLED interface.

## Overview

The detector sequentially illuminates the target surface with red, green, and blue light and measures the reflected intensity using an LDR.

The measured values are then:

1. Averaged to reduce short-term electrical noise.
2. Corrected using stored black and white calibration references.
3. Normalized independently for each RGB channel.
4. Gamma-corrected.
5. Converted into an RGB-like `0–255` representation.
6. Compared against a predefined color database using Euclidean distance.

The closest reference color is reported as the detected color.

## Features

* RGB sequential illumination using an Arduino
* LDR-based reflected-light measurement
* Multi-sample averaging for sensor readings
* Black and white reference calibration
* Calibration persistence using EEPROM
* Per-channel normalization
* Gamma correction
* Euclidean-distance color classification
* OLED graphical user interface
* Button-driven finite state machine
* Raw sensor-data inspection
* Resettable calibration data
* Screensaver / display power management
* Color database stored in `PROGMEM` to reduce RAM usage

## Hardware

| Component        | Purpose                        |
| ---------------- | ------------------------------ |
| Arduino          | Main controller                |
| LDR              | Light/reflection sensor        |
| Red LED          | Red illumination               |
| Green LED        | Green illumination             |
| Blue LED         | Blue illumination              |
| SSD1306 OLED     | User interface                 |
| 2 × Push Buttons | Menu navigation and control    |
| EEPROM           | Persistent calibration storage |

### Pin Configuration

| Pin | Function           |
| --- | ------------------ |
| D2  | Button 1           |
| D7  | Button 2           |
| D3  | Red illumination   |
| D5  | Green illumination |
| D6  | Blue illumination  |
| A2  | LDR analog input   |

## System Architecture

```text
              ┌──────────────────┐
              │   User Interface │
              │    2 Buttons     │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │   State Machine  │
              └────────┬─────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
 ┌─────────────────┐       ┌─────────────────┐
 │   Calibration   │       │ Color Detection │
 └────────┬────────┘       └────────┬────────┘
          │                         │
          └────────────┬────────────┘
                       ▼
              ┌──────────────────┐
              │ RGB Illumination │
              │   + LDR Sensor   │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │ Signal Sampling  │
              │ & Averaging      │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │  Calibration &   │
              │   Normalization  │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │ Gamma Correction │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │ Euclidean Color  │
              │   Classification │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │  Detected Color  │
              │     on OLED      │
              └──────────────────┘
```

## Measurement Pipeline

For each RGB channel, the corresponding LED is enabled and the sensor is allowed to stabilize before measurements are taken.

Five ADC samples are collected:

```text
ADC samples → average → calibrated value
```

The calibration removes the measured black reference:

```text
x = measured - black_reference
```

The result is then scaled according to the calibrated black-to-white range:

```text
normalized = x / (white_reference - black_reference)
```

The normalized value is gamma corrected:

```text
corrected = normalized ^ 0.6
```

and converted to an RGB-like 8-bit value:

```text
RGB = corrected × 255
```

Values are constrained to the range:

```text
0–255
```

## Color Classification

The system currently uses a predefined database containing 12 reference colors:

* Red
* Green
* Blue
* Yellow
* Cyan
* Magenta
* Orange
* Purple
* Pink
* Lime
* White
* Black

For each reference color, the system calculates the squared Euclidean distance:

```text
d² = (R - Rref)²
   + (G - Gref)²
   + (B - Bref)²
```

The reference with the smallest distance is selected.

This is effectively a **nearest-neighbor classifier in RGB space**.

## Calibration

Two calibration references are stored:

### Black Calibration

The sensor measures the minimum/reference response with the target treated as black.

```text
blackRef = {R, G, B}
```

### White Calibration

The sensor measures the maximum/reference response using a white target.

```text
whiteRef = {R, G, B}
```

Both values are stored in EEPROM so that calibration persists after power loss.

## User Interface

The OLED interface is implemented as a finite state machine with the following states:

```text
WELCOME
   ↓
MAIN_MENU
   ├── CALIB_SUB
   │     └── CALIB_RUNNING
   │
   ├── DETECTING
   │
   ├── RAW_DATA
   │
   └── RESET_CONFIRM
```

### Main Menu

The system provides four modes:

```text
Calibration
Color Detect
Raw Data
Reset System
```

### Raw Data Mode

The system can display both calibration values and the latest sensor measurements, making it possible to inspect the sensor pipeline without external serial debugging.

## Memory Considerations

The color database is stored in **AVR program memory (`PROGMEM`)** rather than normal RAM.

During classification, entries are copied from program memory using `memcpy_P()`.

The implementation also avoids unnecessary dynamic `String` usage in the OLED rendering path, which helps keep memory usage predictable on resource-constrained microcontrollers.

## Libraries

The project uses:

* `Wire`
* `Adafruit_GFX`
* `Adafruit_SSD1306`
* `EEPROM`
* `math.h`

## Possible Improvements

Potential future improvements include:

* Empirically determining the optimal gamma value instead of using the current `0.6` value.
* Increasing the number of calibration samples.
* Using median filtering or more robust statistical filtering.
* Expanding the reference-color database with experimentally measured samples.
* Using multiple samples per color class instead of a single RGB reference.
* Calibrating against a standardized color chart.
* Adding confidence/error margins to classifications.
* Characterizing performance under different ambient-light conditions.
* Adding serial logging for experimental data collection.

## Project Focus

This project explores several embedded-systems concepts in a single application:

**Embedded C/C++ · Sensor Acquisition · ADC Sampling · Signal Filtering · Calibration · EEPROM Persistence · RGB Sensing · Numerical Processing · Nearest-Neighbor Classification · Finite State Machines · OLED Interfaces · Resource-Constrained Programming**

---

### Status

**Functional prototype.**

The current implementation is designed primarily as an embedded color-detection experiment and provides raw-data and calibration interfaces for further characterization and tuning.
