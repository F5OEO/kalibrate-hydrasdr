# **Kalibrate-HydraSDR for HydraSDR RFOne**

A specialized and optimized port of `kalibrate`, designed specifically for the **HydraSDR RFOne** high-performance Software Defined Radio.
It scans GSM base stations and uses their broadcast carrier as a highly stable frequency reference.
This allows precise measurement and permanent correction of the HydraSDR’s internal TCXO offset (PPM/PPB).

---

# **New HydraSDR Features**

## 1. Hardware-Specific DSP

* Implements a **Polyphase Rational Resampler** to convert the HydraSDR native sampling rate (2.5 MSPS) to the GSM symbol rate (270.833 kSPS).
* Provides **>60 dB aliasing rejection** for clean decimation.
* Ensures **high-precision timing** required for GSM frequency analysis.

## 2. Direct Flash Calibration

* Can **read** and **write** frequency correction values directly from/to the HydraSDR's SPI Flash.
* Automatically **resets** the device after writing calibration to apply changes immediately.

## 3. Visualization & Diagnostics

* **ASCII Art FFT (`-A`)**: Real-time spectrum visualization directly in the console.
* **DSP Benchmark (`-B`)**: Measures DSP processing throughput on the host CPU.

## 4. Optimized Scanning

* Adds a fast **Power Scan** (coarse scan) before fine-frequency detection.
* Improves band scanning performance by approximately **10×** compared to earlier versions.

## 5. Multi-Platform

* Cross-platform build system powered by **CMake** with support for Windows, Linux, and macOS.

---

# **Calibration Procedure (Step-by-Step)**

Follow these steps to permanently calibrate your HydraSDR RFOne TCXO.

---

## **Step 1: Clear Existing Calibration**

Reset the calibration value to zero before measuring. This also validates device communication.

```bash
kal.exe -W 0
```

(The device will reset. Wait a few seconds for it to re-enumerate.)

---

## **Step 2: Scan for GSM Base Stations**

Scan to find a strong and stable reference signal.

**Recommended bands:**

* `EGSM` (900 MHz)
* `GSM850`
* `DCS` (1800 MHz) — supported but near hardware limit
* `PCS` (1900 MHz) — disabled

```bash
kal.exe -s EGSM -g 16
```

**Example Output:**

```text
kal: Scanning for E-GSM-900 base stations.
E-GSM-900:
 chan:    2 (935.4MHz -36Hz) power:  -33.4 dBFS
 chan:    4 (935.8MHz -134Hz) power:  -52.1 dBFS
 chan:   83 (951.6MHz -147Hz) power:  -49.5 dBFS
 chan:   84 (951.8MHz -315Hz) power:  -50.6 dBFS
 chan:   85 (952.0MHz -180Hz) power:  -57.5 dBFS
 chan:   87 (952.4MHz  29Hz) power:  -50.9 dBFS
 chan:  113 (957.6MHz -16Hz) power:  -33.5 dBFS
 chan:  114 (957.8MHz -140Hz) power:  -42.9 dBFS
 chan:  115 (958.0MHz  117Hz) power:  -54.3 dBFS
 chan:  116 (958.2MHz -83Hz) power:  -54.9 dBFS
 chan:  118 (958.6MHz -54Hz) power:  -24.1 dBFS
...
```

Select the channel with the highest POWER (for example, 118).

---

## **Step 3: Measure Frequency Error**

Tune to the selected channel and measure the frequency offset.

```bash
kal.exe -c 118 -g 16
```

**Example Output:**

```text
kal: Calculating clock frequency offset.
Using GSM-900 channel 118 (958.6MHz)
Scanning for FCCH bursts ('.' = searching, '+' = found)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

--------------------------------------------------
Results (100 valid bursts out of 100 attempts)
--------------------------------------------------
average         [min, max]      (range, stddev)
-70Hz           [-77, -67]      (10, 2.267751)
overruns: 0
not found: 0

Average Error: -0.073 ppm (-73.379 ppb)

```

Record the PPB value (in this example: -73).

---

## **Step 4: Write Calibration to Flash**

Store the PPB correction value permanently in the device.

```bash
kal.exe -W -73
```

**Output:**

```
[-] Erasing flash sector 2 (Calibration area)...
[-] Writing Calibration: -73 ppb (Timestamp: XXXXXXX)...
[+] Calibration written successfully.
[!] Resetting HydraSDR to apply changes...
[+] Device reset command sent.
```

The device will reboot after writing.

---

## **Step 5: Verify Calibration**

Re-measure to confirm the correction is applied.

```bash
kal.exe -c 118 -g 16
```

**Expected Result:**

* Remaining offset should be within ±0.5 ppm
* Typically around ±250 ppb with a strong signal

---

# **Command Usage**

## Scan GSM Base Stations

```bash
kal.exe -s <band> [options]
```

## Compute Frequency Offset

```bash
kal.exe -f <freq> | -c <chan> [options]
```

## Device Maintenance

```bash
kal.exe -R               # Read calibration from flash
kal.exe -W <ppb_value>   # Write calibration and reset device
```

---

# **Command-Line Options**

| Option | Description                                                                  |
| ------ | ---------------------------------------------------------------------------- |
| `-s`   | Scan a band (`EGSM`, `GSM900`, `DCS`, `GSM850`, `GSM-R`). `PCS` is disabled. |
| `-f`   | Absolute frequency of a GSM carrier.                                         |
| `-c`   | Channel number (ARFCN).                                                      |
| `-b`   | Band indicator (required when using `-c`).                                   |
| `-g`   | Gain (0–21 for HydraSDR Linearity Gain).                                     |
| `-R`   | Read calibration from flash.                                                 |
| `-W`   | Write calibration value (PPB) and reset the device.                          |
| `-A`   | Display ASCII FFT spectrum.                                                  |
| `-B`   | Run DSP benchmark and exit.                                                  |
| `-v`   | Verbose output.                                                              |
| `-D`   | Debug messages.                                                              |
| `-h`   | Help text.                                                                   |

---

# **Build & Runtime Notes**

## 1. Runtime Libraries

Ensure the following runtime libraries are present in the same directory as `kal.exe` under Windows:

* `libhydrasdr.dll` or `hydrasdr.dll`
* `libusb-1.0.dll`
* `libwinpthread-1.dll` (unless statically linked)
* `libfftw3-3.dll` (optional if statically linked)

## 2. Frequency Limitations

* Supported: `GSM850`, `GSM900` / `EGSM`, `GSM-R`
* Warning: `DCS` (1800 MHz) is near the device's hardware limit
* Disabled: `PCS` (1900 MHz)

---

# **License**

This project is licensed under the **BSD 2-Clause License**.

Based on and heavily refactored from work by:

* Joshua Lackey (2010)
* Steve Markgraf (2012)
* Stan Pitucha

Additional modifications © 2025 **Benjamin Vernoux** `<bvernoux@hydrasdr.com>`
