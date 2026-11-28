"""
plot_encoder.py: live graph and logger for the AMT132S encoder driver.

Reads the CSV lines the board sends (main_snippets_csv.c), shows joint angle,
speed and acceleration live, displays turns and Z-index errors in the title,
and saves everything to encoder_YYYYMMDD_HHMMSS.csv for Excel or later analysis.

Setup (once):
    pip install pyserial matplotlib

Run:
    python plot_encoder.py          (COM4)
    python plot_encoder.py COM7     (other port)

Close PuTTY first: only one program can use the port. Close the graph window
to stop; the CSV file is saved automatically.
"""

import collections
import csv
import sys
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200
WINDOW_S = 10.0          # seconds of history shown on screen
REFRESH_MS = 50          # how often the graph redraws

# Must match ENC_CSV_HEADER in encoder.h
COLUMNS = ["time_ms", "pos_counts", "turns", "joint_deg", "joint_speed_degs",
           "joint_accel_degs2", "rpm", "index_errors"]

ser = serial.Serial(PORT, BAUD, timeout=0.05)
out_name = time.strftime("encoder_%Y%m%d_%H%M%S.csv")
out_file = open(out_name, "w", newline="")
writer = csv.writer(out_file)
writer.writerow(["time_s"] + COLUMNS[1:])
print(f"Reading {PORT} at {BAUD} baud, saving to {out_name}")

# Recent samples for the plots (deque: fast to trim from the front)
t_buf, ang_buf, spd_buf, acc_buf = (collections.deque() for _ in range(4))
latest = {"turns": 0, "index_errors": 0}
t0_ms = None


def parse_line(raw):
    """Return a dict for one CSV data line, or None for the header or a bad line."""
    parts = raw.split(",")
    if len(parts) != len(COLUMNS):
        return None
    try:
        row = dict(zip(COLUMNS, parts))
        return {
            "time_ms": int(row["time_ms"]),
            "pos_counts": int(row["pos_counts"]),
            "turns": int(row["turns"]),
            "joint_deg": float(row["joint_deg"]),
            "joint_speed_degs": float(row["joint_speed_degs"]),
            "joint_accel_degs2": float(row["joint_accel_degs2"]),
            "rpm": float(row["rpm"]),
            "index_errors": int(row["index_errors"]),
        }
    except ValueError:
        return None                       # header text or a garbled line


rx_partial = ""                           # incomplete line carried to the next read


def read_available_lines():
    """Read everything waiting on the port; log and keep each complete line."""
    global t0_ms, rx_partial
    if not ser.in_waiting:
        return
    rx_partial += ser.read(ser.in_waiting).decode(errors="ignore")
    *lines, rx_partial = rx_partial.split("\n")   # last piece may be unfinished
    for raw in lines:
        d = parse_line(raw.strip())
        if d is None:
            continue
        if t0_ms is None:
            t0_ms = d["time_ms"]          # time 0 = first line received
        t_s = (d["time_ms"] - t0_ms) / 1000.0

        writer.writerow([f"{t_s:.3f}"] + [d[c] for c in COLUMNS[1:]])
        t_buf.append(t_s)
        ang_buf.append(d["joint_deg"])
        spd_buf.append(d["joint_speed_degs"])
        acc_buf.append(d["joint_accel_degs2"])
        latest["turns"] = d["turns"]
        latest["index_errors"] = d["index_errors"]

    while t_buf and t_buf[-1] - t_buf[0] > WINDOW_S:
        for buf in (t_buf, ang_buf, spd_buf, acc_buf):
            buf.popleft()


fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
ax_ang, ax_spd, ax_acc = axes
(line_ang,) = ax_ang.plot([], [], lw=1.5)
(line_spd,) = ax_spd.plot([], [], lw=1.5, color="tab:orange")
(line_acc,) = ax_acc.plot([], [], lw=1.5, color="tab:green")
ax_ang.set_ylabel("Joint angle (deg)")
ax_spd.set_ylabel("Speed (deg/s)")
ax_acc.set_ylabel("Accel (deg/s²)")
ax_acc.set_xlabel("Time (s)")
for ax in axes:
    ax.grid(True, alpha=0.3)
for ax in (ax_spd, ax_acc):
    ax.axhline(0, color="gray", lw=0.8)
title = fig.suptitle("AMT132S encoder: waiting for data")


def update(_frame):
    read_available_lines()
    if t_buf:
        line_ang.set_data(t_buf, ang_buf)
        line_spd.set_data(t_buf, spd_buf)
        line_acc.set_data(t_buf, acc_buf)
        for ax in axes:
            ax.relim()
            ax.autoscale_view()
        ax_acc.set_xlim(max(0.0, t_buf[-1] - WINDOW_S), max(WINDOW_S, t_buf[-1]))
        errs = latest["index_errors"]
        title.set_text(f"AMT132S encoder   turns: {latest['turns']}   "
                       f"Z-index errors: {errs}" + ("   <- CHECK WIRING/NOISE" if errs else ""))
        title.set_color("red" if errs else "black")
    return line_ang, line_spd, line_acc, title


anim = FuncAnimation(fig, update, interval=REFRESH_MS, cache_frame_data=False)

try:
    plt.tight_layout()
    plt.show()                            # blocks until the window is closed
finally:
    out_file.close()
    ser.close()
    print(f"Saved {out_name}")
