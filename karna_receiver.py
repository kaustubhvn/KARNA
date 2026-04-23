
"""
karna_receiver.py  —  KARNA BLE Hub Receiver
Runs on laptop. Connects to KARNA_HUB (ESP32-S3 hub node) via BLE.
Displays gunshot and dead-reckoning data in terminal and on a live
matplotlib scatter plot.

Usage:
    pip install bleak matplotlib
    python karna_receiver.py

BLE characteristics subscribed:
    AAAA0002  →  GS packets  "GS,<az>,<x>,<y>,<range>,<conf>,<zcs>,<ms>"
    AAAA0003  →  DR packets  "DR,<stepCount>,<stepFlag>,<heading>,<ekf_x>,<ekf_y>,..."
"""

import asyncio
asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())


import sys
import math
import time
from datetime import datetime
from bleak import BleakClient, BleakScanner

# ── BLE UUIDs (must match hub_esp32s3.ino) ────────────────────────────────────
HUB_NAME       = "KARNA_HUB"
HUB_SERVICE    = "AAAA0001-0000-1000-8000-00805F9B34FB"
GS_CHAR_UUID   = "AAAA0002-0000-1000-8000-00805F9B34FB"
DR_CHAR_UUID   = "AAAA0003-0000-1000-8000-00805F9B34FB"

# ── Matplotlib (optional — set False to disable) ──────────────────────────────
ENABLE_PLOT = True

# ── State ─────────────────────────────────────────────────────────────────────
gs_events  = []   # list of (az_deg, x_m, y_m, range_m, conf, ts)
dr_history = []   # list of (ekf_x, ekf_y, heading_deg, steps, ts)


# =============================================================================
# PARSE HELPERS
# =============================================================================

def parse_gs(raw: str):
    """Parse "GS,az,x,y,range,conf,zcs,ms" → dict or None."""
    parts = raw.strip().split(",")
    if len(parts) < 7 or parts[0] != "GS":
        return None
    try:
        return {
            "az":    float(parts[1]),
            "x":     float(parts[2]),
            "y":     float(parts[3]),
            "range": float(parts[4]),
            "conf":  float(parts[5]),
            "zcs":   float(parts[6]),
            "ms":    int(parts[7]) if len(parts) > 7 else 0,
            "ts":    time.time(),
        }
    except ValueError:
        return None


def parse_dr(raw: str):
    """Parse "DR,stepCount,stepFlag,heading,ekf_x,ekf_y,alt,pres,..." → dict or None."""
    parts = raw.strip().split(",")
    if len(parts) < 6 or parts[0] != "DR":
        return None
    try:
        return {
            "steps":   int(parts[1]),
            "step_flag": int(parts[2]),
            "heading": float(parts[3]),
            "x":       float(parts[4]),
            "y":       float(parts[5]),
            "alt":     float(parts[6]) if len(parts) > 6 else 0.0,
            "pres":    float(parts[7]) if len(parts) > 7 else 0.0,
            "ts":      time.time(),
        }
    except ValueError:
        return None


def compass8(az_deg: float) -> str:
    dirs = ["N","NE","E","SE","S","SW","W","NW"]
    return dirs[int((az_deg % 360 + 22.5) / 45) % 8]


# =============================================================================
# TERMINAL DISPLAY
# =============================================================================

def print_gs(d: dict):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"\n{'─'*55}")
    print(f"  [{ts}] GUNSHOT DETECTED")
    print(f"  Azimuth  : {d['az']:6.1f}°  ({compass8(d['az'])})")
    print(f"  Position : X={d['x']:+.2f} m   Y={d['y']:+.2f} m")
    print(f"  Range    : {d['range']:.2f} m")
    print(f"  Conf     : {d['conf']*100:.0f}%")
    print(f"  ZCS      : {d['zcs']:.1f} µs")
    print(f"{'─'*55}")


def print_dr(d: dict):
    step_arrow = " ← STEP" if d["step_flag"] else ""
    print(f"  [DR] Steps={d['steps']:4d}  Hdg={d['heading']:+7.1f}°  "
          f"X={d['x']:+6.2f} m  Y={d['y']:+6.2f} m  "
          f"Alt={d['alt']:+.1f} m{step_arrow}")


# =============================================================================
# MATPLOTLIB LIVE PLOT
# =============================================================================

def init_plot():
    import matplotlib
    matplotlib.use("TkAgg")   # change to Qt5Agg if TkAgg unavailable
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title("KARNA — Gunshot & Dead Reckoning", fontsize=11)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axvline(0, color="gray", linewidth=0.5)
    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)

    # Mic array square (14 cm × 14 cm, shown at origin, scaled up for visibility)
    array_patch = mpatches.Rectangle((-0.07, -0.07), 0.14, 0.14,
                                      linewidth=1, edgecolor="black",
                                      facecolor="lightgray", label="Mic array")
    ax.add_patch(array_patch)

    gs_scatter = ax.scatter([], [], c="red",    s=120, zorder=5,
                             label="Gunshot",   marker="*")
    dr_line,    = ax.plot([], [], "b-", alpha=0.5, linewidth=1, label="DR path")
    dr_dot      = ax.scatter([], [], c="blue",  s=60,  zorder=6,
                              label="DR pos",   marker="o")
    az_lines    = []

    ax.legend(loc="upper right", fontsize=8)
    plt.ion()
    plt.show()

    return fig, ax, gs_scatter, dr_line, dr_dot, az_lines


def update_plot(fig, ax, gs_scatter, dr_line, dr_dot, az_lines):
    import matplotlib.pyplot as plt

    # GS scatter
    if gs_events:
        gx = [e[1] for e in gs_events]
        gy = [e[2] for e in gs_events]
        gs_scatter.set_offsets(list(zip(gx, gy)))

        # Clear old azimuth lines
        for ln in az_lines:
            ln.remove()
        az_lines.clear()

        # Draw azimuth ray for last 5 events
        for ev in gs_events[-5:]:
            az_r  = math.radians(ev[0])
            rng   = ev[3] if ev[3] > 0.5 else 10.0
            end_x = math.cos(az_r) * rng
            end_y = math.sin(az_r) * rng
            ln, = ax.plot([0, end_x], [0, end_y], "r--", alpha=0.3, linewidth=0.8)
            az_lines.append(ln)

    # DR path
    if dr_history:
        dx = [p[0] for p in dr_history]
        dy = [p[1] for p in dr_history]
        dr_line.set_data(dx, dy)
        dr_dot.set_offsets([[dx[-1], dy[-1]]])

        # Auto-scale axes to fit DR path
        margin = 3.0
        all_x = dx + ([e[1] for e in gs_events] if gs_events else [0])
        all_y = dy + ([e[2] for e in gs_events] if gs_events else [0])
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    fig.canvas.draw()
    fig.canvas.flush_events()


# =============================================================================
# BLE CALLBACKS
# =============================================================================

def gs_callback(_, data: bytearray):
    raw = data.decode("utf-8", errors="replace")
    d = parse_gs(raw)
    if d:
        gs_events.append((d["az"], d["x"], d["y"], d["range"], d["conf"], d["ts"]))
        print_gs(d)
    else:
        print(f"[GS-RAW] {raw}")


def dr_callback(_, data: bytearray):
    raw = data.decode("utf-8", errors="replace")
    d = parse_dr(raw)
    if d:
        dr_history.append((d["x"], d["y"], d["heading"], d["steps"], d["ts"]))
        print_dr(d)
    else:
        print(f"[DR-RAW] {raw}")


# =============================================================================
# SCAN + CONNECT
# =============================================================================

async def find_hub():
    print(f"[BLE] Scanning for '{HUB_NAME}'...")
    device = await BleakScanner.find_device_by_name(HUB_NAME, timeout=15.0)
    if device is None:
        print(f"[BLE] '{HUB_NAME}' not found. Ensure hub is powered and advertising.")
        sys.exit(1)
    print(f"[BLE] Found: {device.address}")
    return device


async def run():
    device = await find_hub()
    plot_assets = None

    if ENABLE_PLOT:
        try:
            plot_assets = init_plot()
            print("[PLT] Matplotlib plot initialised")
        except Exception as e:
            print(f"[PLT] Plot init failed ({e}) — terminal-only mode")
            plot_assets = None

    async with BleakClient(device.address) as client:
        print(f"[BLE] Connected to {device.address}")

        await client.start_notify(GS_CHAR_UUID, gs_callback)
        await client.start_notify(DR_CHAR_UUID, dr_callback)
        print("[BLE] Subscribed to GS + DR notifications. Waiting for events...")
        print("      Press Ctrl-C to exit.\n")

        try:
            while True:
                await asyncio.sleep(0.25)
                if plot_assets:
                    update_plot(*plot_assets)
        except asyncio.CancelledError:
            pass
        finally:
            await client.stop_notify(GS_CHAR_UUID)
            await client.stop_notify(DR_CHAR_UUID)
            print("\n[BLE] Disconnected")


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nExiting.")
