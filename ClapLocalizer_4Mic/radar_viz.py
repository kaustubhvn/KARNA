#!/usr/bin/env python3
"""
radar_viz.py  —  CS-style acoustic radar for GunshotLocalize.ino (3-mic)
=========================================================================
Reads  DIR,<angle_deg>,<compass>,<rms_max>,<energy>,<conf>  from serial.

WHY PYGAME (not matplotlib):
  matplotlib FuncAnimation redraws the entire figure through Python every
  frame → typically 8-15 fps with noticeable stutter.
  Pygame uses direct surface blitting at 60 fps with hardware-accelerated
  drawing.  The serial reader runs in its own thread and writes into a
  lock-free deque, so the render loop is never blocked by I/O.

INSTALL:
  pip install pygame pyserial

RUN:
  python radar_viz.py                         # defaults to COM16, 115200
  python radar_viz.py --port COM16
  python radar_viz.py --port /dev/ttyACM0 --baud 115200
  python radar_viz.py --port COM16 --radius 8   # map radius 8 m

CONTROLS:
  ESC / Q   — quit
  C         — clear all blips
  +  /  -   — zoom in / out (map radius)
  Space     — pause / resume blip drawing (serial keeps reading)
"""

import argparse
import collections
import math
import sys
import threading
import time

# ── third-party (install with pip) ────────────────────────────────────────────
try:
    import pygame
except ImportError:
    sys.exit("pygame not found.  Run:  pip install pygame")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not found.  Run:  pip install pyserial")


# =============================================================================
#  CONFIGURATION  ← change these to match your setup
# =============================================================================

DEFAULT_PORT    = "COM16"
DEFAULT_BAUD    = 115200
DEFAULT_RADIUS  = 6.0      # initial map radius in metres

# Mic array side length (equilateral triangle, metres) — display only
MIC_SIDE_M      = 0.145

# How long a blip lives on screen (seconds) — older ones fade out
BLIP_LIFETIME   = 12.0

# Max blips kept in history
MAX_BLIPS       = 120

# Radar sweep speed (degrees per second)
SWEEP_SPEED_DPS = 90.0     # full rotation in 4 s

# Target frame rate
FPS             = 60

# Window size
WIN_W, WIN_H    = 1280, 800


# =============================================================================
#  COLOURS  (R, G, B)
# =============================================================================

C_BG        = ( 10,  12,  10)
C_PANEL     = ( 14,  16,  14)
C_BORDER    = ( 40,  50,  40)
C_GRID      = ( 22,  30,  22)
C_GRID_TXT  = ( 55,  65,  55)
C_SWEEP     = (  0, 255,  65)          # CS green
C_PLAYER    = (  0, 170, 255)
C_MIC       = (  0, 220,  80)
C_NORTH     = (200, 200, 200)
C_ARROW_HI  = (255,  60,  60)         # high conf blip
C_ARROW_MID = (255, 160,   0)         # mid conf
C_ARROW_LO  = (120, 120, 120)         # low conf
C_RING_HI   = (255,  60,  60)
C_RING_MID  = (255, 160,   0)
C_RING_LO   = (100, 100, 100)
C_TEXT      = (200, 210, 200)
C_TEXT_DIM  = ( 90, 100,  90)
C_LOG_BG    = (  8,  10,   8)
C_LOG_LINE  = ( 30,  38,  30)
C_STATUS    = (  0, 180,  60)


# =============================================================================
#  DATA MODEL
# =============================================================================

class Blip:
    __slots__ = ["az","rms","energy","conf","born"]
    def __init__(self, az, rms, energy, conf):
        self.az     = az      # degrees  (math convention: 0=East CCW)
        self.rms    = rms
        self.energy = energy
        self.conf   = conf
        self.born   = time.monotonic()

    def age(self):
        return time.monotonic() - self.born

    def alpha(self):
        """0.0 (dead) → 1.0 (fresh)"""
        return max(0.0, 1.0 - self.age() / BLIP_LIFETIME)

    def colour(self):
        if self.conf >= 0.65:
            return C_RING_HI
        elif self.conf >= 0.35:
            return C_RING_MID
        return C_RING_LO

    def arrow_colour(self):
        if self.conf >= 0.65:
            return C_ARROW_HI
        elif self.conf >= 0.35:
            return C_ARROW_MID
        return C_ARROW_LO

    # XY on map  (metres, relative to array centre)
    def xy_m(self, radius_m=5.0):
        """Returns (x, y) in metres for a nominal placement on the blip ring."""
        # We don't have real XY from this firmware — only angle.
        # Place the blip on a ring whose radius is proportional to energy.
        # Higher energy → closer (louder = nearer).
        # Clamp to [0.5, radius_m - 0.3]
        r = max(0.5, min(radius_m * 0.85,
                         radius_m * (1.0 - 0.6 * min(1.0, self.energy * 80))))
        rad = math.radians(self.az)
        return r * math.cos(rad), r * math.sin(rad)


# =============================================================================
#  SERIAL READER  (runs in its own thread)
# =============================================================================

class SerialReader:
    def __init__(self, port, baud, blip_queue: collections.deque):
        self.port       = port
        self.baud       = baud
        self.q          = blip_queue
        self._ser       = None
        self._running   = False
        self._thread    = None
        self.connected  = False
        self.rx_count   = 0
        self.last_line  = ""
        self.error_msg  = ""

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._ser:
            try: self._ser.close()
            except: pass

    def _run(self):
        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.connected = True
                self.error_msg = ""
                while self._running:
                    raw = self._ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    self.last_line = line
                    if line.startswith("DIR,"):
                        blip = self._parse(line)
                        if blip:
                            self.q.append(blip)
                            self.rx_count += 1
            except serial.SerialException as e:
                self.connected = False
                self.error_msg = str(e)
                time.sleep(1.0)
            except Exception as e:
                self.connected = False
                self.error_msg = str(e)
                time.sleep(0.5)

    @staticmethod
    def _parse(line: str):
        # DIR,<angle_deg>,<compass>,<rms_max>,<energy>,<conf>
        try:
            parts = line.split(",")
            if len(parts) < 6:
                return None
            az_compass = float(parts[1])      # compass: 0=N CW
            rms    = float(parts[3])
            energy = float(parts[4])
            conf   = float(parts[5])
            # Convert compass → math angle (0=E CCW) for trig
            az_math = 90.0 - az_compass
            az_math %= 360.0
            return Blip(az_math, rms, energy, conf)
        except (ValueError, IndexError):
            return None


# =============================================================================
#  RADAR SURFACE  (pre-rendered static background — only redrawn on zoom)
# =============================================================================

def build_radar_bg(cx, cy, R_px, radius_m, font_sm):
    """
    Returns a Surface with grid rings, compass labels, mic array.
    Regenerated only when zoom changes — then blitted cheaply every frame.
    """
    surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
    surf.fill((0, 0, 0, 0))

    def m2px(m):
        return int(m / radius_m * R_px)

    # Radar circle fill
    pygame.draw.circle(surf, C_BG, (cx, cy), R_px)

    # Grid rings
    ring_ms = [1, 2, 3, 4, 5, 6, 8, 10, 12, 15]
    for rm in ring_ms:
        r_px = m2px(rm)
        if r_px < 4 or r_px > R_px:
            continue
        pygame.draw.circle(surf, C_GRID, (cx, cy), r_px, 1)
        # Label
        lx = cx + int(r_px * 0.707) + 3
        ly = cy - int(r_px * 0.707) - 10
        lbl = font_sm.render(f"{rm}m", True, C_GRID_TXT)
        surf.blit(lbl, (lx, ly))

    # Cross-hairs
    pygame.draw.line(surf, C_GRID, (cx - R_px, cy), (cx + R_px, cy), 1)
    pygame.draw.line(surf, C_GRID, (cx, cy - R_px), (cx, cy + R_px), 1)

    # Outer border circle
    pygame.draw.circle(surf, C_BORDER, (cx, cy), R_px, 2)

    # Compass labels
    font_compass = pygame.font.SysFont("consolas", 13, bold=True)
    for label, angle_math in [("N",90), ("E",0), ("S",270), ("W",180)]:
        rad = math.radians(angle_math)
        lx = cx + int((R_px - 18) * math.cos(rad))
        ly = cy - int((R_px - 18) * math.sin(rad))
        txt = font_compass.render(label, True, C_NORTH)
        surf.blit(txt, txt.get_rect(center=(lx, ly)))

    # Mic triangle  (equilateral, scaled)
    side_px = m2px(MIC_SIDE_M)
    mics_m  = [
        (0.0,          0.0),
        (MIC_SIDE_M,   0.0),
        (MIC_SIDE_M/2, MIC_SIDE_M * 0.866),
    ]
    # Centre the triangle on the origin
    cx_tri = MIC_SIDE_M / 2
    cy_tri = MIC_SIDE_M * 0.866 / 3
    mic_px = []
    for mx, my in mics_m:
        px = cx + m2px(mx - cx_tri)
        py = cy - m2px(my - cy_tri)
        mic_px.append((px, py))
        pygame.draw.circle(surf, C_MIC, (px, py), 4)
    if len(mic_px) == 3:
        pygame.draw.polygon(surf, C_MIC, mic_px, 1)

    return surf


# =============================================================================
#  LOG PANEL  (right strip)
# =============================================================================

LOG_X      = WIN_W - 320
LOG_W      = 320
LOG_PAD    = 10
LOG_HEADER = 100       # px for header / stats section

def draw_log(screen, blips, font, font_sm, reader, radius_m, paused):
    """Draw the right-side event log panel."""
    # Panel background
    panel_rect = pygame.Rect(LOG_X, 0, LOG_W, WIN_H)
    pygame.draw.rect(screen, C_LOG_BG, panel_rect)
    pygame.draw.line(screen, C_BORDER, (LOG_X, 0), (LOG_X, WIN_H), 2)

    y = 12

    # Title
    t = font.render("ACOUSTIC RADAR", True, C_STATUS)
    screen.blit(t, (LOG_X + LOG_PAD, y)); y += 26

    # Status
    if reader.connected:
        s_col = C_STATUS
        s_txt = f"● {reader.port}  {reader.baud}"
    else:
        s_col = C_ARROW_HI
        s_txt = f"✕ {reader.port} — reconnecting…"
    t = font_sm.render(s_txt, True, s_col)
    screen.blit(t, (LOG_X + LOG_PAD, y)); y += 18

    t = font_sm.render(f"Events: {reader.rx_count}   Zoom: {radius_m:.1f} m",
                        True, C_TEXT_DIM)
    screen.blit(t, (LOG_X + LOG_PAD, y)); y += 18

    if paused:
        t = font_sm.render("[ PAUSED ]", True, C_ARROW_MID)
        screen.blit(t, (LOG_X + LOG_PAD, y))
    y += 20

    # Divider
    pygame.draw.line(screen, C_LOG_LINE, (LOG_X, y), (WIN_W, y), 1); y += 8

    # Column header
    hdr = font_sm.render(" #   AZ     CONF   ENERGY  AGE", True, C_TEXT_DIM)
    screen.blit(hdr, (LOG_X + LOG_PAD, y)); y += 16
    pygame.draw.line(screen, C_LOG_LINE, (LOG_X, y), (WIN_W, y), 1); y += 6

    # Rows — most recent first
    visible = list(reversed(blips))[:25]
    for i, blip in enumerate(visible):
        age   = blip.age()
        alpha = blip.alpha()
        if alpha <= 0:
            continue
        col = tuple(int(c * max(0.35, alpha)) for c in blip.colour())
        az_compass = (90.0 - blip.az) % 360.0
        row = (f"{reader.rx_count - i:3d}  "
               f"{az_compass:5.1f}°  "
               f"{blip.conf*100:4.0f}%  "
               f"{blip.energy:.4f}  "
               f"{age:4.1f}s")
        txt = font_sm.render(row, True, col)
        screen.blit(txt, (LOG_X + LOG_PAD, y))
        y += 15
        if y > WIN_H - 50:
            break

    # Bottom — last raw line
    pygame.draw.line(screen, C_LOG_LINE,
                     (LOG_X, WIN_H-36), (WIN_W, WIN_H-36), 1)
    raw = reader.last_line[:42]
    t = font_sm.render(raw, True, C_TEXT_DIM)
    screen.blit(t, (LOG_X + LOG_PAD, WIN_H - 26))

    # Key hints
    hints = font_sm.render("ESC=quit  C=clear  +/-=zoom  Space=pause",
                             True, C_TEXT_DIM)
    screen.blit(hints, (LOG_X + LOG_PAD, WIN_H - 50))


# =============================================================================
#  SWEEP SURFACE  (full-screen alpha, redrawn cheaply each frame)
# =============================================================================

def draw_sweep(surf, cx, cy, R_px, angle_deg):
    """
    Draw the rotating green sweep line and its trailing wedge onto surf.
    surf must be an SRCALPHA surface filled with (0,0,0,0) each frame.
    """
    rad    = math.radians(angle_deg)
    tip_x  = cx + int(R_px * math.cos(rad))
    tip_y  = cy - int(R_px * math.sin(rad))

    # Trailing wedge — 30° sector behind the sweep
    N_TRAIL = 40
    for i in range(N_TRAIL):
        a   = angle_deg - i * (30.0 / N_TRAIL)
        ar  = math.radians(a)
        px  = cx + int(R_px * math.cos(ar))
        py  = cy - int(R_px * math.sin(ar))
        alpha_w = int(60 * (1.0 - i / N_TRAIL))
        pygame.draw.line(surf, (0, 255, 65, alpha_w), (cx, cy), (px, py), 1)

    # Main sweep line
    pygame.draw.line(surf, (0, 255, 65, 220), (cx, cy), (tip_x, tip_y), 2)


# =============================================================================
#  BLIP DRAWING
# =============================================================================

def draw_blips(screen, blips, cx, cy, R_px, radius_m, last_blip):
    def m2px(m): return m / radius_m * R_px

    for blip in blips:
        alpha = blip.alpha()
        if alpha <= 0.02:
            continue
        a = int(255 * alpha)

        bx_m, by_m = blip.xy_m(radius_m)
        bx = cx + int(m2px(bx_m))
        by = cy - int(m2px(by_m))   # y-axis: up = positive

        # Clamp to radar circle
        dx, dy   = bx - cx, by - cy
        dist_px  = math.hypot(dx, dy)
        if dist_px > R_px - 4:
            scale = (R_px - 4) / dist_px
            bx = cx + int(dx * scale)
            by = cy + int(dy * scale)

        col = blip.colour()
        col_a = (*col, a)

        is_last = (blip is last_blip)

        # Confidence ring
        ring_r = int(6 + blip.conf * 12)
        ring_surf = pygame.Surface((ring_r*2+4, ring_r*2+4), pygame.SRCALPHA)
        pygame.draw.circle(ring_surf, (*col, int(a * 0.4)),
                           (ring_r+2, ring_r+2), ring_r, 1)
        screen.blit(ring_surf, (bx - ring_r - 2, by - ring_r - 2))

        # Blip dot
        dot_r = 7 if is_last else max(3, int(5 * alpha))
        dot_surf = pygame.Surface((dot_r*2+2, dot_r*2+2), pygame.SRCALPHA)
        pygame.draw.circle(dot_surf, (*col, a),
                           (dot_r+1, dot_r+1), dot_r)
        if is_last:
            # white outline
            pygame.draw.circle(dot_surf, (255,255,255,200),
                               (dot_r+1, dot_r+1), dot_r, 1)
        screen.blit(dot_surf, (bx - dot_r - 1, by - dot_r - 1))

    # Draw direction arrow from centre to latest blip
    if last_blip and last_blip.alpha() > 0.1:
        rad     = math.radians(last_blip.az)
        arrow_l = min(R_px * 0.55, m2px(3.5))
        ax      = cx + int(arrow_l * math.cos(rad))
        ay      = cy - int(arrow_l * math.sin(rad))
        col     = last_blip.arrow_colour()
        pygame.draw.line(screen, col, (cx, cy), (ax, ay), 2)

        # Arrowhead
        head_len = 12
        head_ang = 0.45    # radians
        for sign in [1, -1]:
            hx = ax - int(head_len * math.cos(rad - sign * head_ang))
            hy = ay + int(head_len * math.sin(rad - sign * head_ang))
            pygame.draw.line(screen, col, (ax, ay), (hx, hy), 2)

        # Range arc
        az_c = (90.0 - last_blip.az) % 360.0
        bx_m, by_m = last_blip.xy_m(radius_m)
        arc_r_px = int(math.hypot(m2px(bx_m), m2px(by_m)))
        if arc_r_px > 4:
            arc_rect = pygame.Rect(cx - arc_r_px, cy - arc_r_px,
                                    arc_r_px*2, arc_r_px*2)
            start_a = math.radians(az_c - 8)
            stop_a  = math.radians(az_c + 8)
            try:
                pygame.draw.arc(screen, col, arc_rect, start_a, stop_a, 1)
            except Exception:
                pass

        # Label
        font_label = pygame.font.SysFont("consolas", 10)
        az_c_str = f"{az_c:.1f}° ({compass8(az_c)})  conf {last_blip.conf*100:.0f}%"
        lbl = font_label.render(az_c_str, True, col)
        lbx = cx + int(m2px(bx_m)) + 10
        lby = cy - int(m2px(by_m)) - 14
        # Keep label in map area
        lbx = max(10, min(LOG_X - lbl.get_width() - 5, lbx))
        lby = max(5,  min(WIN_H - 20, lby))
        screen.blit(lbl, (lbx, lby))


def compass8(az_deg_compass):
    dirs = ["N","NE","E","SE","S","SW","W","NW"]
    return dirs[int((az_deg_compass + 22.5) / 45) % 8]


# =============================================================================
#  PLAYER ICON  (you = array centre)
# =============================================================================

def draw_player(screen, cx, cy):
    pygame.draw.circle(screen, C_PLAYER, (cx, cy), 7)
    pygame.draw.circle(screen, (255,255,255), (cx, cy), 7, 1)
    # Small cross
    pygame.draw.line(screen, (255,255,255), (cx-11,cy), (cx+11,cy), 1)
    pygame.draw.line(screen, (255,255,255), (cx,cy-11), (cx,cy+11), 1)


# =============================================================================
#  MAIN
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="CS radar for GunshotLocalize.ino")
    ap.add_argument("--port",   default=DEFAULT_PORT,   help="Serial port (e.g. COM16)")
    ap.add_argument("--baud",   type=int, default=DEFAULT_BAUD)
    ap.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                    help="Initial map radius in metres")
    args = ap.parse_args()

    # ── pygame init ────────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption(f"Acoustic Radar  ·  {args.port}")
    clock  = pygame.time.Clock()

    font    = pygame.font.SysFont("consolas", 14, bold=True)
    font_sm = pygame.font.SysFont("consolas", 12)

    # Radar geometry
    radar_cx = (WIN_W - LOG_W) // 2
    radar_cy = WIN_H // 2
    R_px     = min(radar_cx, radar_cy) - 30

    # ── serial reader ──────────────────────────────────────────────────────────
    blip_deque = collections.deque(maxlen=MAX_BLIPS)
    reader     = SerialReader(args.port, args.baud, blip_deque)
    reader.start()

    # ── state ─────────────────────────────────────────────────────────────────
    radius_m    = args.radius
    sweep_angle = 90.0     # start pointing North
    paused      = False
    bg_surf     = build_radar_bg(radar_cx, radar_cy, R_px, radius_m, font_sm)
    sweep_surf  = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)

    # ── main loop ─────────────────────────────────────────────────────────────
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0   # delta-time in seconds

        # ── events ────────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_c:
                    blip_deque.clear()
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS,
                                   pygame.K_KP_PLUS):
                    radius_m = max(1.0, radius_m - 1.0)
                    bg_surf  = build_radar_bg(radar_cx, radar_cy, R_px,
                                              radius_m, font_sm)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    radius_m = min(50.0, radius_m + 1.0)
                    bg_surf  = build_radar_bg(radar_cx, radar_cy, R_px,
                                              radius_m, font_sm)

        # ── advance sweep ─────────────────────────────────────────────────────
        sweep_angle = (sweep_angle + SWEEP_SPEED_DPS * dt) % 360.0

        # ── consume new blips from serial thread ──────────────────────────────
        # (blip_deque is already updated by the reader thread)
        blips = list(blip_deque)   # snapshot; cheap O(n) copy

        # Expire dead blips
        blips = [b for b in blips if b.alpha() > 0.01]
        last_blip = blips[-1] if blips else None

        # ── render ────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # 1. Static background (grid + mics)
        screen.blit(bg_surf, (0, 0))

        # 2. Sweep (alpha surface)
        sweep_surf.fill((0, 0, 0, 0))
        draw_sweep(sweep_surf, radar_cx, radar_cy, R_px, sweep_angle)
        screen.blit(sweep_surf, (0, 0))

        # 3. Blips + arrow
        if not paused:
            draw_blips(screen, blips, radar_cx, radar_cy, R_px,
                       radius_m, last_blip)

        # 4. Player icon (on top of blips)
        draw_player(screen, radar_cx, radar_cy)

        # 5. Log panel
        draw_log(screen, blips, font, font_sm, reader, radius_m, paused)

        # 6. FPS counter (top-left)
        fps_txt = font_sm.render(f"{clock.get_fps():.0f} fps", True, C_TEXT_DIM)
        screen.blit(fps_txt, (8, 8))

        # 7. Title overlay (bottom-left of radar)
        title = font_sm.render(
            "3-MIC EQUILATERAL  ·  14.5 cm  ·  GCC-PHAT DoA",
            True, C_TEXT_DIM)
        screen.blit(title, (10, WIN_H - 20))

        pygame.display.flip()

    # ── cleanup ────────────────────────────────────────────────────────────────
    reader.stop()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
