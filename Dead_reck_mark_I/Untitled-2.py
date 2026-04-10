
"""
PDR Localization Visualizer
Connects via BLE to ESP32-S3 PDR device and plots:
  - Local x-y dead reckoning trajectory
  - Raw GPS trajectory
  - Fused GPS trajectory
  - Heading, step count, altitude, GPS status

Requirements:
    pip install bleak matplotlib numpy
"""

import asyncio
import csv
import math
import time
import threading
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from bleak import BleakClient, BleakScanner

# ── BLE CONFIG ──────────────────────────────────────────────
DEVICE_NAME    = "PDR_ESP32"
CHAR_UUID      = "12345678-1234-1234-1234-1234567890CD"
SCAN_TIMEOUT   = 15   # seconds

# ── CSV LOG ─────────────────────────────────────────────────
CSV_FILE = f"pdr_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
CSV_HEADERS = [
    "timestamp", "steps", "step_flag", "heading_deg",
    "x", "y", "altitude", "pressure",
    "raw_lat", "raw_lon", "fused_lat", "fused_lon",
    "gps_speed", "gps_course", "sats", "gps_valid"
]

# ── SHARED DATA (thread-safe via simple lock) ────────────────
MAX_POINTS = 2000   # history length on plots

data_lock   = threading.Lock()
pdr_x_hist  = deque(maxlen=MAX_POINTS)
pdr_y_hist  = deque(maxlen=MAX_POINTS)
raw_lat_hist = deque(maxlen=MAX_POINTS)
raw_lon_hist = deque(maxlen=MAX_POINTS)
fus_lat_hist = deque(maxlen=MAX_POINTS)
fus_lon_hist = deque(maxlen=MAX_POINTS)

# Scalar telemetry
latest = {
    "steps": 0, "step_flag": 0,
    "heading": 0.0, "x": 0.0, "y": 0.0,
    "altitude": 0.0, "pressure": 0.0,
    "raw_lat": 0.0,  "raw_lon": 0.0,
    "fused_lat": 0.0,"fused_lon": 0.0,
    "gps_speed": 0.0,"gps_course": 0.0,
    "sats": 0, "gps_valid": 0,
    "ts": 0.0
}
origin_lat = None
origin_lon = None
connected  = False

heading_hist   = deque(maxlen=MAX_POINTS)
altitude_hist  = deque(maxlen=MAX_POINTS)
step_hist      = deque(maxlen=MAX_POINTS)
time_hist      = deque(maxlen=MAX_POINTS)


def latlon_to_xy(origin_lat, origin_lon, lat, lon):
    """Convert lat/lon to local x (east) / y (north) in metres."""
    R = 6371000.0
    y = (lat - origin_lat) * math.pi / 180.0 * R
    x = (lon - origin_lon) * math.pi / 180.0 * R * \
        math.cos(origin_lat * math.pi / 180.0)
    return x, y


def parse_packet(raw: str):
    """Parse comma-separated BLE packet into a dict."""
    try:
        fields = raw.strip().split(",")
        if len(fields) < 15:
            return None
        return {
            "steps":      int(fields[0]),
            "step_flag":  int(fields[1]),
            "heading":    float(fields[2]),
            "x":          float(fields[3]),
            "y":          float(fields[4]),
            "altitude":   float(fields[5]),
            "pressure":   float(fields[6]),
            "raw_lat":    float(fields[7]),
            "raw_lon":    float(fields[8]),
            "fused_lat":  float(fields[9]),
            "fused_lon":  float(fields[10]),
            "gps_speed":  float(fields[11]),
            "gps_course": float(fields[12]),
            "sats":       int(fields[13]),
            "gps_valid":  int(fields[14]),
            "ts":         time.time()
        }
    except Exception as e:
        print(f"[PARSE ERR] {e} | raw: {raw[:60]}")
        return None


def on_ble_data(sender, data: bytearray):
    global origin_lat, origin_lon
    raw = data.decode("utf-8", errors="ignore")
    pkt = parse_packet(raw)
    if pkt is None:
        return

    # Write to CSV
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().isoformat(),
            pkt["steps"], pkt["step_flag"], pkt["heading"],
            pkt["x"], pkt["y"], pkt["altitude"], pkt["pressure"],
            pkt["raw_lat"], pkt["raw_lon"],
            pkt["fused_lat"], pkt["fused_lon"],
            pkt["gps_speed"], pkt["gps_course"],
            pkt["sats"], pkt["gps_valid"]
        ])

    # Set origin for GPS→local conversion on Python side
    if origin_lat is None and pkt["gps_valid"] and pkt["sats"] >= 4:
        if pkt["raw_lat"] != 0.0:
            origin_lat = pkt["raw_lat"]
            origin_lon = pkt["raw_lon"]
            print(f"[VIZ] GPS origin set: {origin_lat:.6f}, {origin_lon:.6f}")

    with data_lock:
        # PDR trajectory (comes directly from ESP32 in metres)
        pdr_x_hist.append(pkt["x"])
        pdr_y_hist.append(pkt["y"])

        # GPS trajectories converted to local metres for comparison
        if origin_lat is not None and pkt["raw_lat"] != 0.0:
            rx, ry = latlon_to_xy(origin_lat, origin_lon,
                                  pkt["raw_lat"], pkt["raw_lon"])
            raw_lat_hist.append(rx)
            raw_lon_hist.append(ry)

            fx, fy = latlon_to_xy(origin_lat, origin_lon,
                                  pkt["fused_lat"], pkt["fused_lon"])
            fus_lat_hist.append(fx)
            fus_lon_hist.append(fy)

        heading_hist.append(pkt["heading"])
        altitude_hist.append(pkt["altitude"])
        step_hist.append(pkt["steps"])
        time_hist.append(pkt["ts"])
        latest.update(pkt)


# ── BLE ASYNC RUNNER ────────────────────────────────────────
async def ble_runner():
    global connected
    print(f"[BLE] Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME, timeout=SCAN_TIMEOUT)
    if device is None:
        print(f"[BLE] Device '{DEVICE_NAME}' not found. "
              "Check BLE advertising.")
        return

    print(f"[BLE] Found: {device.address}")
    async with BleakClient(device) as client:
        connected = True
        print("[BLE] Connected!")
        await client.start_notify(CHAR_UUID, on_ble_data)
        print("[BLE] Receiving data. Close plot window to stop.")
        # Keep running until killed
        while connected:
            await asyncio.sleep(0.5)


def ble_thread():
    asyncio.run(ble_runner())


# ── MATPLOTLIB SETUP ─────────────────────────────────────────
fig = plt.figure(figsize=(16, 9), facecolor="#1a1a2e")
fig.suptitle("PDR Localization — Real-Time Monitor",
             color="white", fontsize=14, fontweight="bold")

gs = gridspec.GridSpec(3, 3, figure=fig,
                       hspace=0.45, wspace=0.35)

# Main trajectory plot
ax_traj   = fig.add_subplot(gs[0:2, 0:2])
# Heading over time
ax_head   = fig.add_subplot(gs[0, 2])
# Altitude over time
ax_alt    = fig.add_subplot(gs[1, 2])
# Step count over time
ax_steps  = fig.add_subplot(gs[2, 0])
# GPS error (raw GPS vs fused distance)
ax_err    = fig.add_subplot(gs[2, 1])
# Status text
ax_status = fig.add_subplot(gs[2, 2])
ax_status.axis("off")

DARK_BG   = "#1a1a2e"
PANEL_BG  = "#16213e"
TEXT_COL  = "#e0e0e0"
COL_PDR   = "#00d4ff"   # cyan   — dead reckoning
COL_RAW   = "#ff6b6b"   # red    — raw GPS
COL_FUS   = "#51cf66"   # green  — fused

for ax in [ax_traj, ax_head, ax_alt, ax_steps, ax_err]:
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=TEXT_COL, labelsize=8)
    ax.xaxis.label.set_color(TEXT_COL)
    ax.yaxis.label.set_color(TEXT_COL)
    ax.title.set_color(TEXT_COL)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444466")

ax_traj.set_title("Trajectory (metres from origin)")
ax_traj.set_xlabel("East (m)")
ax_traj.set_ylabel("North (m)")
ax_traj.set_aspect("equal", "datalim")
ax_traj.grid(True, alpha=0.2)

line_pdr, = ax_traj.plot([], [], color=COL_PDR, lw=2,
                          label="PDR (dead reckoning)")
line_raw, = ax_traj.plot([], [], color=COL_RAW, lw=1.5,
                          linestyle="--", label="Raw GPS")
line_fus, = ax_traj.plot([], [], color=COL_FUS, lw=2,
                          label="Fused GPS")
dot_cur,  = ax_traj.plot([], [], "o", color="yellow",
                          markersize=8, zorder=5)
ax_traj.legend(loc="upper left", fontsize=8,
               facecolor=PANEL_BG, labelcolor=TEXT_COL)

ax_head.set_title("Heading (°)")
ax_head.set_ylabel("Degrees")
ax_head.grid(True, alpha=0.2)
line_head, = ax_head.plot([], [], color="#ffd43b", lw=1.5)

ax_alt.set_title("Relative Altitude (m)")
ax_alt.set_ylabel("Metres")
ax_alt.grid(True, alpha=0.2)
line_alt, = ax_alt.plot([], [], color="#a29bfe", lw=1.5)

ax_steps.set_title("Cumulative Steps")
ax_steps.set_ylabel("Steps")
ax_steps.grid(True, alpha=0.2)
line_steps, = ax_steps.plot([], [], color="#fd7e14", lw=1.5)

ax_err.set_title("GPS-PDR Offset (m)")
ax_err.set_ylabel("Distance (m)")
ax_err.grid(True, alpha=0.2)
line_err, = ax_err.plot([], [], color="#f06595", lw=1.5)
err_hist = deque(maxlen=MAX_POINTS)

status_text = ax_status.text(
    0.05, 0.95, "", transform=ax_status.transAxes,
    color=TEXT_COL, fontsize=9, va="top", fontfamily="monospace")


def update_plot(frame):
    with data_lock:
        px = list(pdr_x_hist)
        py = list(pdr_y_hist)
        rx = list(raw_lat_hist)
        ry = list(raw_lon_hist)
        fx = list(fus_lat_hist)
        fy = list(fus_lon_hist)
        hh = list(heading_hist)
        ah = list(altitude_hist)
        sh = list(step_hist)
        th = list(time_hist)
        lv = dict(latest)

    # ── Trajectory ──
    if px:
        line_pdr.set_data(px, py)
        dot_cur.set_data([px[-1]], [py[-1]])
    if rx:
        line_raw.set_data(rx, ry)
    if fx:
        line_fus.set_data(fx, fy)
    ax_traj.relim()
    ax_traj.autoscale_view()

    # Draw heading arrow from current position
    for artist in ax_traj.patches:
        artist.remove()
    if px:
        head_rad = lv["heading"] * math.pi / 180.0
        arrow_len = max(2.0, (max(px) - min(px) + 1) * 0.08)
        ax_traj.annotate(
            "", xy=(px[-1] + arrow_len * math.cos(head_rad),
                    py[-1] + arrow_len * math.sin(head_rad)),
            xytext=(px[-1], py[-1]),
            arrowprops=dict(arrowstyle="->", color="yellow", lw=2))

    # ── Time-domain plots ──
    if th:
        t0   = th[0]
        trel = [t - t0 for t in th]
        line_head.set_data(trel, hh)
        ax_head.relim(); ax_head.autoscale_view()

        line_alt.set_data(trel, ah)
        ax_alt.relim(); ax_alt.autoscale_view()

        line_steps.set_data(trel, sh)
        ax_steps.relim(); ax_steps.autoscale_view()

        # GPS-PDR offset
        err = []
        for i in range(min(len(rx), len(px))):
            # raw GPS vs PDR
            offset = math.sqrt((rx[i]-px[i])**2 + (ry[i]-py[i])**2) \
                     if i < len(rx) else 0
            err.append(offset)
        if err:
            err_hist.clear()
            err_hist.extend(err)
            line_err.set_data(trel[:len(err)], err)
            ax_err.relim(); ax_err.autoscale_view()

    # ── Status panel ──
    gps_str  = f"{'YES ✓' if lv['gps_valid'] else 'NO  ✗'}"
    status   = (
        f"Steps:     {lv['steps']}\n"
        f"Heading:   {lv['heading']:.1f}°\n"
        f"X / Y:     {lv['x']:.2f} / {lv['y']:.2f} m\n"
        f"Altitude:  {lv['altitude']:.2f} m\n"
        f"Pressure:  {lv['pressure']:.1f} hPa\n"
        f"GPS Valid: {gps_str}\n"
        f"Sats:      {lv['sats']}\n"
        f"GPS Speed: {lv['gps_speed']:.2f} m/s\n"
        f"Raw Lat:   {lv['raw_lat']:.6f}\n"
        f"Raw Lon:   {lv['raw_lon']:.6f}\n"
        f"Fused Lat: {lv['fused_lat']:.6f}\n"
        f"Fused Lon: {lv['fused_lon']:.6f}\n"
        f"BLE:       {'Connected' if connected else 'Scanning...'}"
    )
    status_text.set_text(status)

    return (line_pdr, line_raw, line_fus, dot_cur,
            line_head, line_alt, line_steps, line_err, status_text)


# ── MAIN ────────────────────────────────────────────────────
if __name__ == "__main__":
    # Initialise CSV
    with open(CSV_FILE, "w", newline="") as f:
        csv.writer(f).writerow(CSV_HEADERS)
    print(f"[LOG] Logging to {CSV_FILE}")

    # Start BLE in background thread
    t = threading.Thread(target=ble_thread, daemon=True)
    t.start()

    # Start plot animation (100ms interval)
    ani = FuncAnimation(fig, update_plot, interval=100,
                        blit=False, cache_frame_data=False)

    plt.show()
    connected = False
    print("[EXIT] Done. CSV saved.")