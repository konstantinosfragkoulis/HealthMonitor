#!/usr/bin/env python3
"""
monitor.py — Real-time ECG + IMU monitor for the HealthMonitor firmware.

Reads the binary USB CDC stream, parses all packet types, and plots:
  - ECG waveform with R-peak markers
  - Accelerometer XYZ with SCG AO markers
  - Gyroscope XYZ

Displays BPM (ECG + SCG), HRV metrics, respiratory rate, and packet stats.

Usage:
    python monitor.py              # auto-detect port
    python monitor.py /dev/ttyACM0 # explicit port
    python monitor.py COM3         # Windows

Packet formats (all little-endian, packed, 32-bit CRC-32/MPEG-2):

  TelemetryPacket (0xAABB, 24 bytes):
    header    uint16    ecg       int16     timestamp uint32
    ax,ay,az  int16*3   gx,gy,gz  int16*3
    crc       uint32

  HeartBeatEvent (0xCCDD, 16 bytes):
    header    uint16    bpm       int16(*10)
    source    uint8(0=ECG,1=SCG)  reserved  uint8     reserved2 uint16
    timestamp uint32    crc       uint32

  BreathEvent (0xEEFF, 12 bytes):
    header    uint16    rate      int16(*10 BrPM)
    timestamp uint32    crc       uint32

  HRVReport (0xFFAA, 24 bytes):
    header    uint16    bpm       int16(*10)
    rmssd     uint16    sdnn      uint16    sd1       uint16    sd2  uint16
    pnn50     uint8     n_beats   uint8     source    uint8     reserved uint8
    timestamp uint32    crc       uint32
"""

import sys
import struct
from collections import deque

import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Config ────────────────────────────────────────────────────────────────────
BAUD = 115200           # CDC ignores this, but pyserial requires a value
DISPLAY_SECONDS = 4.0   # scrolling window width
FS = 480                # sample rate (Hz)

# ── Packet definitions ────────────────────────────────────────────────────────
PACKETS = {
    0xAABB: ("telemetry", 24, "<HhIhhhhhhI"),
    0xCCDD: ("beat",      16, "<HhBBHII"),
    0xEEFF: ("breath",    12, "<HhII"),
    0xFFAA: ("hrv",       24, "<HhHHHHBBBBII"),
}


# ── CRC-32/MPEG-2 (matches STM32 default CRC peripheral) ─────────────────────
def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if (crc & 0x80000000) else (crc << 1)
            crc &= 0xFFFFFFFF
    return crc


def verify_crc(packet: bytes) -> bool:
    """Verify 32-bit CRC over all bytes except the last 4 (the CRC field)."""
    expected = struct.unpack_from("<I", packet, len(packet) - 4)[0]
    return crc32_mpeg2(packet[:-4]) == expected


# ── Serial port detection ─────────────────────────────────────────────────────
def find_serial_port() -> str:
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == 0xCAFE or "healthmonitor" in (p.description or "").lower():
            print(f"Found device: {p.device} ({p.description})")
            return p.device
    for p in ports:
        if "acm" in (p.device or "").lower() or "usbmodem" in (p.device or "").lower():
            print(f"Guessing device: {p.device} ({p.description})")
            return p.device
    print("Available ports:")
    for p in ports:
        print(f"  {p.device}: {p.description} [VID:PID={p.vid}:{p.pid}]")
    sys.exit("Could not auto-detect port. Specify it as an argument.")


# ── Packet parser ─────────────────────────────────────────────────────────────
class PacketParser:
    """Scans a byte stream for known packet headers, validates CRC, unpacks."""

    def __init__(self):
        self.buf = bytearray()
        self.packets = {name: [] for name, _, _ in PACKETS.values()}
        self.crc_errors = 0
        self.sync_drops = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        for lst in self.packets.values():
            lst.clear()

        while len(self.buf) >= 2:
            header = struct.unpack_from("<H", self.buf, 0)[0]

            if header in PACKETS:
                name, size, fmt = PACKETS[header]
                if len(self.buf) < size:
                    break
                pkt = bytes(self.buf[:size])
                del self.buf[:size]
                if verify_crc(pkt):
                    self.packets[name].append(struct.unpack(fmt, pkt))
                else:
                    self.crc_errors += 1
            else:
                del self.buf[:1]
                self.sync_drops += 1


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_serial_port()
    ser = serial.Serial(port, BAUD, timeout=0.05)
    print(f"Opened {port}")

    parser = PacketParser()
    max_samples = int(DISPLAY_SECONDS * FS * 1.2)

    # Telemetry ring buffers
    ecg_data = deque(maxlen=max_samples)
    ecg_ts = deque(maxlen=max_samples)
    acc_x = deque(maxlen=max_samples)
    acc_y = deque(maxlen=max_samples)
    acc_z = deque(maxlen=max_samples)
    gyr_x = deque(maxlen=max_samples)
    gyr_y = deque(maxlen=max_samples)
    gyr_z = deque(maxlen=max_samples)

    # Beat markers: (sample_index, signal_value) for scatter plots
    rpeak_marks = deque(maxlen=200)
    ao_marks = deque(maxlen=200)
    pending_rpeak_ts = []
    pending_ao_ts = []

    sample_idx = 0
    total_packets = 0

    # Display state
    ecg_bpm = "--"
    scg_bpm = "--"
    resp_rate = "--"
    ecg_hrv = ""
    scg_hrv = ""

    # ── Plot setup ────────────────────────────────────────────────────────────
    fig, (ax_ecg, ax_acc, ax_gyr) = plt.subplots(
        3, 1, figsize=(14, 9), gridspec_kw={"height_ratios": [2, 1, 1]})
    fig.suptitle("HealthMonitor \u2014 Real-Time Telemetry", fontsize=13,
                 fontweight="bold")

    # ECG subplot
    ax_ecg.set_ylabel("ECG (ADC)")
    (line_ecg,) = ax_ecg.plot([], [], "k-", linewidth=0.7)
    scat_rp = ax_ecg.scatter([], [], c="red", s=40, zorder=5, label="R-peak")
    ax_ecg.legend(loc="upper right", fontsize=8)

    info_box = ax_ecg.text(
        0.01, 0.97, "", transform=ax_ecg.transAxes, fontsize=9,
        verticalalignment="top", fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85))

    # Accelerometer subplot
    ax_acc.set_ylabel("Accel (LSB)")
    (line_ax,) = ax_acc.plot([], [], "r-", lw=0.5, alpha=0.7, label="X")
    (line_ay,) = ax_acc.plot([], [], "g-", lw=0.5, alpha=0.7, label="Y")
    (line_az,) = ax_acc.plot([], [], "b-", lw=0.5, alpha=0.8, label="Z")
    scat_ao = ax_acc.scatter([], [], c="magenta", s=40, zorder=5,
                             marker="D", label="AO")
    ax_acc.legend(loc="upper right", fontsize=8)

    # Gyroscope subplot
    ax_gyr.set_ylabel("Gyro (LSB)")
    ax_gyr.set_xlabel("Sample")
    (line_gx,) = ax_gyr.plot([], [], "r-", lw=0.5, alpha=0.7, label="X")
    (line_gy,) = ax_gyr.plot([], [], "g-", lw=0.5, alpha=0.7, label="Y")
    (line_gz,) = ax_gyr.plot([], [], "b-", lw=0.5, alpha=0.8, label="Z")
    ax_gyr.legend(loc="upper right", fontsize=8)

    resp_box = ax_gyr.text(
        0.01, 0.92, "", transform=ax_gyr.transAxes, fontsize=9,
        verticalalignment="top", fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightcyan", alpha=0.85))

    plt.tight_layout()

    # ── Helpers ───────────────────────────────────────────────────────────────
    def match_ts_to_sample(target_ts, signal):
        """Find the sample index and signal value closest to a TIM2 timestamp.
        Uses unsigned wraparound-safe arithmetic."""
        if not ecg_ts:
            return None, None
        best_i, best_diff = None, 0xFFFFFFFF
        for i, ts in enumerate(ecg_ts):
            diff = (int(ts) - int(target_ts)) & 0xFFFFFFFF
            if diff > 0x80000000:
                diff = 0xFFFFFFFF - diff + 1
            if diff < best_diff:
                best_diff = diff
                best_i = i
        if best_i is not None:
            base = sample_idx - len(ecg_ts)
            return base + best_i, list(signal)[best_i]
        return None, None

    def visible_markers(marks, start):
        """Filter marker deque to only those visible in the current window."""
        vx = [p[0] for p in marks if p[0] >= start]
        vy = [p[1] for p in marks if p[0] >= start]
        return np.column_stack([vx, vy]) if vx else np.empty((0, 2))

    def auto_ylim(ax, *arrays):
        """Set y-limits with 10% margin around data range."""
        vals = []
        for a in arrays:
            vals.extend(a)
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        margin = max((hi - lo) * 0.1, 10)
        ax.set_ylim(lo - margin, hi + margin)

    # ── Animation update ──────────────────────────────────────────────────────
    def update(_frame):
        nonlocal sample_idx, total_packets
        nonlocal ecg_bpm, scg_bpm, resp_rate, ecg_hrv, scg_hrv

        raw = ser.read(8192)
        if raw:
            parser.feed(raw)

        # Beat events
        for _, bpm_x10, source, _, _, ts, _ in parser.packets["beat"]:
            bpm_str = f"{bpm_x10 / 10.0:.1f}"
            if source == 0:
                ecg_bpm = bpm_str
                pending_rpeak_ts.append(ts)
            else:
                scg_bpm = bpm_str
                pending_ao_ts.append(ts)

        # Breath events
        for _, rate_x10, _, _ in parser.packets["breath"]:
            resp_rate = f"{rate_x10 / 10.0:.1f}"

        # HRV reports
        for _, bpm, rmssd, sdnn, sd1, sd2, pnn50, n, src, _, _, _ in \
                parser.packets["hrv"]:
            text = (f"RMSSD={rmssd / 10:.1f}  SDNN={sdnn / 10:.1f}  "
                    f"pNN50={pnn50}%  SD1={sd1 / 10:.1f}  SD2={sd2 / 10:.1f}  "
                    f"({n}b)")
            if src == 0:
                ecg_hrv = text
            else:
                scg_hrv = text

        # Telemetry samples
        for _, ecg, ts, ax, ay, az, gx, gy, gz, _ in \
                parser.packets["telemetry"]:
            ecg_data.append(ecg)
            ecg_ts.append(ts)
            acc_x.append(ax); acc_y.append(ay); acc_z.append(az)
            gyr_x.append(gx); gyr_y.append(gy); gyr_z.append(gz)
            sample_idx += 1
            total_packets += 1

        # Match pending beat timestamps to the nearest telemetry sample
        still_pending_rp = []
        for rp_ts in pending_rpeak_ts:
            idx, val = match_ts_to_sample(rp_ts, ecg_data)
            if idx is not None:
                rpeak_marks.append((idx, val))
            else:
                still_pending_rp.append(rp_ts)
        pending_rpeak_ts.clear()
        pending_rpeak_ts.extend(still_pending_rp)

        still_pending_ao = []
        for ao_ts in pending_ao_ts:
            idx, val = match_ts_to_sample(ao_ts, acc_z)
            if idx is not None:
                ao_marks.append((idx, val))
            else:
                still_pending_ao.append(ao_ts)
        pending_ao_ts.clear()
        pending_ao_ts.extend(still_pending_ao)

        # ── Redraw ────────────────────────────────────────────────────────────
        n = len(ecg_data)
        if n > 0:
            xs = np.arange(sample_idx - n, sample_idx)
            start = xs[0]

            ecg_arr = list(ecg_data)
            line_ecg.set_data(xs, ecg_arr)
            ax_ecg.set_xlim(start, xs[-1])
            auto_ylim(ax_ecg, ecg_arr)
            scat_rp.set_offsets(visible_markers(rpeak_marks, start))

            ax_list = list(acc_x)
            ay_list = list(acc_y)
            az_list = list(acc_z)
            line_ax.set_data(xs, ax_list)
            line_ay.set_data(xs, ay_list)
            line_az.set_data(xs, az_list)
            ax_acc.set_xlim(start, xs[-1])
            auto_ylim(ax_acc, ax_list, ay_list, az_list)
            scat_ao.set_offsets(visible_markers(ao_marks, start))

            gx_list = list(gyr_x)
            gy_list = list(gyr_y)
            gz_list = list(gyr_z)
            line_gx.set_data(xs, gx_list)
            line_gy.set_data(xs, gy_list)
            line_gz.set_data(xs, gz_list)
            ax_gyr.set_xlim(start, xs[-1])
            auto_ylim(ax_gyr, gx_list, gy_list, gz_list)

        # Info text
        lines = [f"ECG: {ecg_bpm} BPM   SCG: {scg_bpm} BPM   "
                 f"Resp: {resp_rate} BrPM"]
        if ecg_hrv:
            lines.append(f"ECG HRV: {ecg_hrv}")
        if scg_hrv:
            lines.append(f"SCG HRV: {scg_hrv}")
        lines.append(f"Pkts: {total_packets}  CRC err: {parser.crc_errors}  "
                     f"Sync drop: {parser.sync_drops}")
        info_box.set_text("\n".join(lines))

        resp_box.set_text(f"Resp: {resp_rate} BrPM")

        return (line_ecg, scat_rp, info_box,
                line_ax, line_ay, line_az, scat_ao,
                line_gx, line_gy, line_gz, resp_box)

    _ani = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
