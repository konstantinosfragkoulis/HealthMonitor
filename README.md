# HealthMonitor 

Bare-metal wearable for real-time ECG, Seismocardiogram (SCG), and respiratory processing.

![Health Monitor Rev2 PCB](images/HM-Rev2.jpeg)

**Core Stack:** STM32U5 (Cortex-M33) | ST1VAFE6AX (6-axis IMU) | AD8232 (ECG AFE)

## Architecture & DSP
To guarantee precise timing, the firmware relies entirely on hardware synchronization. The IMU’s data-ready external interrupt triggers a simultaneous DMA transfer for both the SPI bus (IMU) and the ADC (ECG). This locks both data streams strictly to a 480 Hz sample rate, eliminating software polling jitter.

The main loop runs three independent DSP pipelines on-device:

* **ECG (Pan-Tompkins++):** Bandpass -> derivative -> squaring -> flattop smoothing -> MWI. Uses adaptive thresholding with search-back for real-time R-peak detection [1].
* **SCG (Template Matching NCC):** Accel Z-axis (dorso-ventral) filtered at 7–30 Hz. Bootstraps a heartbeat template during the first 10s via a 4th-power envelope and pairwise NCC [2]. Runs a sliding normalized cross-correlation against the template for continuous AO detection. The template is double-buffered and refreshed every 60s.
* **Respiratory Rate:** Accel Z-axis LPF -> decimated 48:1 (to 10 Hz) -> bandpass 0.1–0.6 Hz. Fed into a 30-second autocorrelation window, outputting a new breath rate every 5 seconds.

HRV metrics (RMSSD, SDNN, pNN50, SD1, SD2) are calculated independently for both the ECG and SCG streams over a rolling 32-beat window based on Task Force standards [3,4]. Data is streamed to a host via a USB Virtual COM Port with CRC-32 integrity.

## Project Structure
* `/hardware/Rev1`: First spin. **See `ERRATA.md`** (known routing/schematic flaw with the AD8232).
* `/hardware/Rev2`: Fixed AD8232 layout, swapped to USB-C, placed components on the bottom layer.
* `/fw`: STM32CubeIDE project, bare-metal C source, DSP logic, and TinyUSB stack.
* `/software`: Python telemetry parser and plotter.

## Roadmap
- [x] Hardware validation.
- [x] Synchronized IMU + ECG DMA capture.
- [x] ECG R-peak detection (Pan-Tompkins++).
- [x] SCG heartbeat detection (Template Matching NCC).
- [x] Real-time respiratory rate from chest-wall accelerometry.
- [x] On-device HRV metrics calculation.
- [x] USB CDC telemetry via TinyUSB.
- [ ] **WIP:** On-device validation — benchmarking ECG vs SCG HRV against published datasets.
- [ ] Offload HAR (Human Activity Recognition) to the IMU's hardware Machine Learning Core.
- [ ] Multi-axis respiratory fusion (PCA + gyro).

## Build Instructions
Compiled with **STM32CubeIDE**.
1. `git clone https://github.com/konstantinosfragkoulis/HealthMonitor.git`
2. Open STM32CubeIDE > `File > STM32 Project Create/Import > Import STM32 Project > STM32CubeMX/STM32CubeIDE Project`.
3. Point to the `/fw` directory.
4. Build (Release/Debug) and flash via ST-Link.

## Host Telemetry (Python)
The STM32 streams binary packets over USB CDC. Use the included Python script to parse the stream, verify CRC-32 payloads, and plot the DSP pipelines in real time.

```bash
pip install pyserial numpy matplotlib # You might want to use a virtual environment for this

# Auto-detect port
python monitor.py

# Manually select port
python monitor.py COM3         # Windows
python monitor.py /dev/ttyACM0 # Linux
```

## References
- USB stack powered by the open-source [TinyUSB](https://github.com/hathach/tinyusb) library (MIT License).

[1] M. N. Imtiaz and N. Khan, "Pan-Tompkins++: A Robust Approach to Detect R-peaks in ECG Signals," in *Proc. IEEE Int. Conf. Bioinformatics and Biomedicine (BIBM) Workshops*, 2022. [Online]. Available: https://arxiv.org/abs/2211.03171

[2] S. Parlato, J. Centracchio, D. Esposito, E. Andreozzi, "Fully automated template matching method for ECG-free heartbeat detection in cardiomechanical signals," *Physical and Engineering Sciences in Medicine*, vol. 48, pp. 649–664, 2025.

[3] Task Force of the European Society of Cardiology and the North American Society of Pacing and Electrophysiology, "Heart rate variability: standards of measurement, physiological interpretation and clinical use," *Circulation*, vol. 93, pp. 1043–1065, 1996.

[4] F. Shaffer and J. P. Ginsberg, "An Overview of Heart Rate Variability Metrics and Norms," *Frontiers in Public Health*, vol. 5, 258, 2017.
