# HealthMonitor
> **Status: Work In Progress (Active Development)** 🚧

![Health Monitor Rev2 PCB](assets/HM-Rev2.jpeg)

This repository contains the hardware and firmware for a custom, bare-metal biomedical wearable designed to capture synchronized Electrocardiogram (ECG) and Seismocardiogram (SCG) data.

## 📖 Project Overview
I designed a PCB with a low-power STM32U5 microcontroller, a low-noise density biosensor from ST (currently utilizing just its IMU), and an AD8232 analog front-end for ECG. 

The microcontroller receives an external interrupt from the IMU when a new measurement is ready, triggering a Direct Memory Access (DMA) read for both the IMU and the Analog to Digital Converter (ADC). This ensures the ECG is captured (almost) in sync with the corresponding physical acceleration. 

I have implemented a modified Pan-Tompkins algorithm to detect R-wave peaks, allowing for the real-time calculation of instantaneous heart rate, moving average heart rate, and heart rate variability. This data is streamed to a host machine via a USB Virtual COM Port.

## 🗄️ Repository Structure
* `/hardware/Rev1`: Initial board design. **Note:** See `ERRATA.md` for a known schematic routing flaw regarding the AD8232.
* `/hardware/Rev2`: Current working iteration. AD8232 flaw corrected.
* `/firmware`: STM32 bare-metal C code, DSP pipelines, and TinyUSB implementation.

## 🚀 Current Focus & Next Steps
- [x] Hardware validation.
- [x] Synchronized IMU and ECG data capture with DMA and BPM calculation with Pan-Tompkins.
- [x] Bare metal USB CDC communication via TinyUSB.
- [ ] **Current:** IMU DSP pipeline for real-time respiratory rate extraction.
- [ ] Heartbeat detection via Seismocardiography (SCG).
- [ ] Offloading Human Activity Recognition (HAR) to the IMU's Machine Learning Core.
- [ ] Firmware refactoring and documentation cleanup.

## ⚖️ Acknowledgments
* USB stack powered by the excellent open-source [TinyUSB](https://github.com/hathach/tinyusb) library (MIT License).