#!/usr/bin/env python3
"""Graphical telemetry viewer for KAKUTEH7 serial stream.

Usage:
  python scripts/read_serial_gui.py --port COM9

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import math
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk
from typing import Iterable

np = None
try:
    if importlib.util.find_spec("numpy") is not None:
        np = importlib.import_module("numpy")
except Exception:  # pragma: no cover
    np = None

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required. Install it with: pip install pyserial") from exc

imufusion = None
imufusion_error = ""
try:
    if importlib.util.find_spec("imufusion") is not None:
        imufusion = importlib.import_module("imufusion")
except Exception as exc:  # pragma: no cover
    imufusion = None
    imufusion_error = str(exc)


SOF1 = 0xA5
SOF2 = 0x5A
PACKET_TYPE_TELEMETRY = 0x01
TELEMETRY_PAYLOAD_LEN = 73
TELEMETRY_STRUCT = struct.Struct("<Iiii6hBIIi16H")
IMU_ACCEL_LSB_PER_G = 2048.0
IMU_GYRO_DPS_PER_LSB = 2000.0 / 32768.0


def _checksum(packet_type: int, payload: bytes) -> int:
    result = packet_type ^ len(payload)
    for byte in payload:
        result ^= byte
    return result & 0xFF


def _list_ports() -> Iterable[str]:
    for port in list_ports.comports():
        yield f"{port.device}: {port.description}"


def _pick_port() -> str | None:
    preferred_keywords = ("Betaflight", "STM32H743", "USB Serial Device", "CDC")
    fallback = None

    for port in list_ports.comports():
        description = f"{port.device}: {port.description}"
        if "STLink" in port.description or "ST-LINK" in port.description:
            continue
        if any(keyword.lower() in description.lower() for keyword in preferred_keywords):
            return port.device
        if fallback is None:
            fallback = port.device

    return fallback


def _decode_frames(buffer: bytearray) -> Iterable[tuple[int, ...]]:
    while True:
        start = buffer.find(bytes((SOF1, SOF2)))
        if start < 0:
            if buffer:
                buffer[:] = buffer[-1:] if buffer[-1] == SOF1 else b""
            return

        if start > 0:
            del buffer[:start]

        if len(buffer) < 4:
            return

        packet_type = buffer[2]
        payload_len = buffer[3]
        frame_len = 4 + payload_len + 1

        if len(buffer) < frame_len:
            return

        payload = bytes(buffer[4 : 4 + payload_len])
        checksum = buffer[4 + payload_len]
        del buffer[:frame_len]

        if checksum != _checksum(packet_type, payload):
            continue
        if packet_type != PACKET_TYPE_TELEMETRY:
            continue
        if payload_len != TELEMETRY_PAYLOAD_LEN:
            continue

        yield TELEMETRY_STRUCT.unpack(payload)


@dataclass
class TelemetryState:
    sequence: int = 0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    ax: int = 0
    ay: int = 0
    az: int = 0
    pressure_pa: int = 0
    altitude_cm: int = 0
    rc_link_ok: bool = False
    rc_frames: int = 0
    channels: list[int] = field(default_factory=lambda: [0] * 16)
    last_update_monotonic: float = 0.0
    connected: bool = False
    port_name: str = ""
    status_text: str = "Sniffing serial ports..."


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, state: TelemetryState, lock: threading.Lock):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.state = state
        self.lock = lock
        self.running = True
        self.error_message = ""

    def stop(self) -> None:
        self.running = False

    def _candidate_ports(self) -> list[str]:
        if self.port.lower() != "auto":
            return [self.port]

        ports: list[str] = []
        for p in list_ports.comports():
            if "STLink" in p.description or "ST-LINK" in p.description:
                continue
            ports.append(p.device)
        return ports

    def run(self) -> None:
        while self.running:
            candidates = self._candidate_ports()
            if not candidates:
                with self.lock:
                    self.state.connected = False
                    self.state.status_text = "Sniffing: no candidate serial ports"
                time.sleep(0.4)
                continue

            connected_once = False
            for candidate in candidates:
                if not self.running:
                    break
                try:
                    with serial.Serial(candidate, self.baud, timeout=0.2) as ser:
                        connected_once = True
                        with self.lock:
                            self.state.connected = True
                            self.state.port_name = candidate
                            self.state.status_text = f"Connected: {candidate}"

                        buffer = bytearray()
                        last_packet_time = time.monotonic()
                        while self.running:
                            chunk = ser.read(ser.in_waiting or 1)
                            if not chunk:
                                # If stream goes idle for too long, force reopen so reconnect/sniff can recover.
                                if (time.monotonic() - last_packet_time) > 2.0:
                                    with self.lock:
                                        self.state.connected = False
                                        self.state.status_text = f"No data on {candidate}, reopening..."
                                    break
                                continue
                            buffer.extend(chunk)
                            for values in _decode_frames(buffer):
                                (
                                    sequence,
                                    pitch_cd,
                                    roll_cd,
                                    yaw_cd,
                                    gx,
                                    gy,
                                    gz,
                                    ax,
                                    ay,
                                    az,
                                    rc_link,
                                    rc_frames,
                                    pressure_pa,
                                    altitude_cm,
                                    *channels,
                                ) = values

                                with self.lock:
                                    self.state.sequence = int(sequence)
                                    self.state.pitch_deg = int(pitch_cd) / 100.0
                                    self.state.roll_deg = int(roll_cd) / 100.0
                                    self.state.yaw_deg = int(yaw_cd) / 100.0
                                    self.state.gx = int(gx)
                                    self.state.gy = int(gy)
                                    self.state.gz = int(gz)
                                    self.state.ax = int(ax)
                                    self.state.ay = int(ay)
                                    self.state.az = int(az)
                                    self.state.pressure_pa = int(pressure_pa)
                                    self.state.altitude_cm = int(altitude_cm)
                                    self.state.rc_link_ok = bool(rc_link)
                                    self.state.rc_frames = int(rc_frames)
                                    self.state.channels = [int(v) for v in channels]
                                    self.state.last_update_monotonic = time.monotonic()
                                    self.state.connected = True
                                    self.state.status_text = f"Connected: {candidate}"
                                last_packet_time = time.monotonic()
                except serial.SerialException as exc:
                    self.error_message = str(exc)
                    with self.lock:
                        self.state.connected = False
                        self.state.status_text = f"Port lost: {candidate} (sniffing...)"
                    continue
                except Exception as exc:  # pragma: no cover - defensive reconnect path
                    self.error_message = str(exc)
                    with self.lock:
                        self.state.connected = False
                        self.state.status_text = f"Reader error: {candidate} ({exc})"
                    continue

            if not connected_once:
                with self.lock:
                    self.state.connected = False
                    self.state.status_text = "Sniffing serial ports..."
                time.sleep(0.3)


class TelemetryApp:
    def __init__(self, root: tk.Tk, state: TelemetryState, lock: threading.Lock):
        self.root = root
        self.state = state
        self.lock = lock

        root.title("KAKUTEH7 Telemetry GUI")
        root.geometry("1180x760")
        root.minsize(980, 640)

        self.header_var = tk.StringVar(value="Waiting for data...")
        self.imu_var = tk.StringVar(value="")
        self.baro_var = tk.StringVar(value="")

        header = ttk.Frame(root, padding=10)
        header.pack(fill=tk.X)
        ttk.Label(header, textvariable=self.header_var, font=("Segoe UI", 13, "bold")).pack(anchor=tk.W)
        ttk.Label(header, textvariable=self.imu_var, font=("Consolas", 11)).pack(anchor=tk.W)
        ttk.Label(header, textvariable=self.baro_var, font=("Consolas", 11)).pack(anchor=tk.W)

        body = ttk.Frame(root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(body)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.att_canvas = tk.Canvas(left, width=560, height=340, bg="#0f1720", highlightthickness=0)
        self.att_canvas.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        self.stick_canvas = tk.Canvas(left, width=560, height=280, bg="#151a22", highlightthickness=0)
        self.stick_canvas.pack(fill=tk.BOTH, expand=True)

        board3d_card = ttk.Frame(right)
        board3d_card.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        ttk.Label(board3d_card, text="Kakute 3D Body", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
        self.board3d_canvas = tk.Canvas(board3d_card, width=520, height=290, bg="#0b1220", highlightthickness=0)
        self.board3d_canvas.pack(fill=tk.BOTH, expand=True)

        channel_card = ttk.Frame(right)
        channel_card.pack(fill=tk.BOTH, expand=True)

        ttk.Label(channel_card, text="Channels", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))

        self.channel_canvas = tk.Canvas(channel_card, bg="#0f1319", highlightthickness=0)
        self.channel_canvas.pack(fill=tk.BOTH, expand=True)

        self.footer_var = tk.StringVar(value="")
        ttk.Label(root, textvariable=self.footer_var, font=("Consolas", 10), padding=(10, 0, 10, 8)).pack(anchor=tk.W)
        self._render_prev_valid = False
        self._render_prev_pitch = 0.0
        self._render_prev_roll = 0.0
        self._render_prev_yaw = 0.0
        self._fusion_ahrs = None
        self._fusion_bias = None
        self._fusion_last_monotonic = 0.0
        self._fusion_ready = False
        if imufusion is not None:
            if hasattr(imufusion, "Bias") and hasattr(imufusion, "BiasSettings"):
                self._fusion_bias = imufusion.Bias()
                self._fusion_bias.settings = imufusion.BiasSettings(sample_rate=100)
            elif hasattr(imufusion, "Offset"):
                self._fusion_bias = imufusion.Offset(100)
            self._fusion_ahrs = imufusion.Ahrs()
            if hasattr(imufusion, "AhrsSettings"):
                self._fusion_ahrs.settings = imufusion.AhrsSettings(
                    convention=imufusion.CONVENTION_NWU,
                    gain=0.45,
                    gyroscope_range=2000,
                    acceleration_rejection=90,
                    magnetic_rejection=0,
                    recovery_trigger_period=100,
                )
            elif hasattr(imufusion, "Settings"):
                self._fusion_ahrs.settings = imufusion.Settings(
                    imufusion.CONVENTION_NWU,
                    0.45,
                    2000,
                    90,
                    0,
                    100,
                )
            self._fusion_ready = self._fusion_bias is not None

    @staticmethod
    def _wrap_deg(value: float) -> float:
        while value > 180.0:
            value -= 360.0
        while value < -180.0:
            value += 360.0
        return value

    @classmethod
    def _angle_delta_deg(cls, to_value: float, from_value: float) -> float:
        return cls._wrap_deg(to_value - from_value)

    def _select_continuous_euler(self, pitch: float, roll: float, yaw: float) -> tuple[float, float, float]:
        c1_pitch = self._wrap_deg(pitch)
        c1_roll = self._wrap_deg(roll)
        c1_yaw = self._wrap_deg(yaw)

        # Equivalent ZYX Euler solution branch.
        c2_pitch = self._wrap_deg(180.0 - pitch)
        c2_roll = self._wrap_deg(roll + 180.0)
        c2_yaw = self._wrap_deg(yaw + 180.0)

        if not self._render_prev_valid:
            self._render_prev_pitch = c1_pitch
            self._render_prev_roll = c1_roll
            self._render_prev_yaw = c1_yaw
            self._render_prev_valid = True
            return c1_pitch, c1_roll, c1_yaw

        c1_cost = (
            abs(self._angle_delta_deg(c1_pitch, self._render_prev_pitch))
            + abs(self._angle_delta_deg(c1_roll, self._render_prev_roll))
            + abs(self._angle_delta_deg(c1_yaw, self._render_prev_yaw))
        )
        c2_cost = (
            abs(self._angle_delta_deg(c2_pitch, self._render_prev_pitch))
            + abs(self._angle_delta_deg(c2_roll, self._render_prev_roll))
            + abs(self._angle_delta_deg(c2_yaw, self._render_prev_yaw))
        )

        if c2_cost < c1_cost:
            sel_pitch, sel_roll, sel_yaw = c2_pitch, c2_roll, c2_yaw
        else:
            sel_pitch, sel_roll, sel_yaw = c1_pitch, c1_roll, c1_yaw

        self._render_prev_pitch = self._wrap_deg(self._render_prev_pitch + self._angle_delta_deg(sel_pitch, self._render_prev_pitch))
        self._render_prev_roll = self._wrap_deg(self._render_prev_roll + self._angle_delta_deg(sel_roll, self._render_prev_roll))
        self._render_prev_yaw = self._wrap_deg(self._render_prev_yaw + self._angle_delta_deg(sel_yaw, self._render_prev_yaw))
        return self._render_prev_pitch, self._render_prev_roll, self._render_prev_yaw

    def _channel_to_norm(self, raw: int) -> float:
        clamped = max(999, min(2000, raw))
        return ((clamped - 999) / 1001.0) * 2.0 - 1.0

    def _draw_attitude(self, pitch: float, roll: float, yaw: float) -> None:
        c = self.att_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()

        cx = w / 2.0
        cy = h / 2.0

        sky = "#2b5a8a"
        ground = "#5f3f2d"
        # Draw a proper attitude sphere slice: horizon shifts with pitch and rotates with roll.
        roll_rad = math.radians(roll)
        pitch_px = max(-h * 0.35, min(h * 0.35, pitch * (h / 90.0) * 0.5))
        horizon_y = cy + pitch_px

        # Use an oversized world polygon and rotate it into screen space.
        span = max(w, h) * 2.4
        cosr = math.cos(roll_rad)
        sinr = math.sin(roll_rad)

        def rot(x: float, y: float) -> tuple[float, float]:
            dx = x - cx
            dy = y - horizon_y
            rx = dx * cosr - dy * sinr
            ry = dx * sinr + dy * cosr
            return cx + rx, horizon_y + ry

        sky_poly_world = [
            (cx - span, horizon_y - span),
            (cx + span, horizon_y - span),
            (cx + span, horizon_y),
            (cx - span, horizon_y),
        ]
        ground_poly_world = [
            (cx - span, horizon_y),
            (cx + span, horizon_y),
            (cx + span, horizon_y + span),
            (cx - span, horizon_y + span),
        ]
        sky_poly = [coord for p in sky_poly_world for coord in rot(*p)]
        ground_poly = [coord for p in ground_poly_world for coord in rot(*p)]

        c.create_polygon(sky_poly, fill=sky, outline="")
        c.create_polygon(ground_poly, fill=ground, outline="")

        # Horizon line
        hx0, hy0 = rot(cx - span, horizon_y)
        hx1, hy1 = rot(cx + span, horizon_y)
        c.create_line(hx0, hy0, hx1, hy1, fill="#f5f7fa", width=3)

        # Pitch ladder every 10 degrees around center.
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y_off = (deg / 45.0) * (h * 0.25)
            lx0, ly0 = rot(cx - 38, horizon_y + y_off)
            lx1, ly1 = rot(cx + 38, horizon_y + y_off)
            c.create_line(lx0, ly0, lx1, ly1, fill="#dbeafe", width=2)

        # Crosshair
        c.create_line(cx - 40, cy, cx + 40, cy, fill="#e5e7eb", width=2)
        c.create_line(cx, cy - 30, cx, cy + 30, fill="#e5e7eb", width=2)
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#e5e7eb", outline="")

        # Yaw tape
        bar_y = h - 32
        c.create_rectangle(20, bar_y, w - 20, bar_y + 16, outline="#94a3b8", width=1)
        yaw_norm = ((yaw + 180.0) % 360.0) - 180.0
        x = 20 + ((yaw_norm + 180.0) / 360.0) * (w - 40)
        c.create_line(x, bar_y, x, bar_y + 16, fill="#f59e0b", width=3)

        c.create_text(14, 14, anchor="nw", fill="#f8fafc", font=("Consolas", 12, "bold"), text=f"P {pitch:+6.2f}  R {roll:+6.2f}  Y {yaw:+6.2f}")

    def _draw_sticks(self, ch: list[int]) -> None:
        c = self.stick_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()

        pad = 30
        size = min((w - pad * 3) / 2, h - 60)
        y0 = (h - size) / 2

        left_x0 = pad
        right_x0 = pad * 2 + size

        self._draw_single_stick(c, left_x0, y0, size, self._channel_to_norm(ch[3]), self._channel_to_norm(ch[2]), "Left CH4/CH3")
        self._draw_single_stick(c, right_x0, y0, size, self._channel_to_norm(ch[0]), self._channel_to_norm(ch[1]), "Right CH1/CH2")

    def _draw_single_stick(self, c: tk.Canvas, x0: float, y0: float, size: float, x_norm: float, y_norm: float, label: str) -> None:
        x1 = x0 + size
        y1 = y0 + size

        c.create_rectangle(x0, y0, x1, y1, outline="#9ca3af", width=2)
        c.create_line((x0 + x1) / 2, y0, (x0 + x1) / 2, y1, fill="#4b5563")
        c.create_line(x0, (y0 + y1) / 2, x1, (y0 + y1) / 2, fill="#4b5563")

        px = x0 + ((x_norm + 1.0) * 0.5) * size
        py = y0 + (1.0 - (y_norm + 1.0) * 0.5) * size
        c.create_oval(px - 8, py - 8, px + 8, py + 8, fill="#22d3ee", outline="")

        c.create_text(x0, y0 - 8, anchor="sw", fill="#e5e7eb", font=("Consolas", 11), text=f"{label}  X {x_norm:+.2f} Y {y_norm:+.2f}")

    def _draw_channels(self, ch: list[int]) -> None:
        c = self.channel_canvas
        c.delete("all")
        w = c.winfo_width()

        left = 75
        right = w - 24
        top = 24
        row_h = 22

        for i, raw in enumerate(ch, start=1):
            y = top + (i - 1) * row_h
            n = self._channel_to_norm(raw)
            zero_x = left + (right - left) * 0.5
            val_x = left + (right - left) * ((n + 1.0) * 0.5)

            c.create_text(10, y + 8, anchor="w", fill="#e2e8f0", font=("Consolas", 10), text=f"CH{i:02d}")
            c.create_rectangle(left, y, right, y + 14, outline="#374151", width=1)
            c.create_line(zero_x, y, zero_x, y + 14, fill="#4b5563")

            if val_x >= zero_x:
                c.create_rectangle(zero_x, y + 1, val_x, y + 13, fill="#16a34a", outline="")
            else:
                c.create_rectangle(val_x, y + 1, zero_x, y + 13, fill="#2563eb", outline="")

            c.create_text(right + 6, y + 8, anchor="w", fill="#cbd5e1", font=("Consolas", 10), text=f"{raw:4d}")

    def _draw_board_3d(self, pitch: float, roll: float, yaw: float, quaternion=None) -> None:
        c = self.board3d_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        cx = w / 2.0
        cy = h / 2.0

        # Body model (mm-like arbitrary units): short PCB slab + front arrow.
        bw = 90.0
        bd = 90.0
        bh = 8.0
        verts = [
            (-bw, -bd, -bh),
            ( bw, -bd, -bh),
            ( bw,  bd, -bh),
            (-bw,  bd, -bh),
            (-bw, -bd,  bh),
            ( bw, -bd,  bh),
            ( bw,  bd,  bh),
            (-bw,  bd,  bh),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        # Front arrow from center toward +X body axis.
        arrow = [
            (0.0, 0.0, bh + 2.0),
            (120.0, 0.0, bh + 2.0),
            (95.0, -16.0, bh + 2.0),
            (95.0, 16.0, bh + 2.0),
        ]

        axis_len = 130.0
        axis_origin = (0.0, 0.0, 0.0)
        axis_x = (axis_len, 0.0, 0.0)
        axis_y = (0.0, axis_len, 0.0)
        axis_z = (0.0, 0.0, axis_len)

        if quaternion is not None:
            qw = float(quaternion.w)
            qx = float(quaternion.x)
            qy = float(quaternion.y)
            qz = float(quaternion.z)

            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qz * qw)
            r02 = 2.0 * (qx * qz + qy * qw)
            r10 = 2.0 * (qx * qy + qz * qw)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            r12 = 2.0 * (qy * qz - qx * qw)
            r20 = 2.0 * (qx * qz - qy * qw)
            r21 = 2.0 * (qy * qz + qx * qw)
            r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        else:
            pr = math.radians(pitch)
            rr = math.radians(roll)
            yr = math.radians(yaw)

            cp = math.cos(pr)
            sp = math.sin(pr)
            cr = math.cos(rr)
            sr = math.sin(rr)
            cyaw = math.cos(yr)
            syaw = math.sin(yr)

            # Direct ZYX (yaw-pitch-roll) rotation matrix application.
            # This matches the firmware/Fusion Euler convention and avoids axis swap at non-zero yaw.
            r00 = cyaw * cp
            r01 = cyaw * sp * sr - syaw * cr
            r02 = cyaw * sp * cr + syaw * sr
            r10 = syaw * cp
            r11 = syaw * sp * sr + cyaw * cr
            r12 = syaw * sp * cr - cyaw * sr
            r20 = -sp
            r21 = cp * sr
            r22 = cp * cr

        def rotate(v: tuple[float, float, float]) -> tuple[float, float, float]:
            x, y, z = v
            xr = r00 * x + r01 * y + r02 * z
            yr = r10 * x + r11 * y + r12 * z
            zr = r20 * x + r21 * y + r22 * z
            return xr, yr, zr

        def project(v: tuple[float, float, float]) -> tuple[float, float, float]:
            x, y, z = v
            # Display mapping so zero attitude appears flat and +X points into screen.
            x_view = y
            y_view = z
            z_view = -x
            dist = 420.0
            scale = dist / (dist - z_view)
            sx = cx + x_view * scale * 1.2
            sy = cy - y_view * scale * 1.2
            return sx, sy, z_view

        rverts = [project(rotate(v)) for v in verts]
        rarrow = [project(rotate(v)) for v in arrow]
        ax_o = project(rotate(axis_origin))
        ax_x = project(rotate(axis_x))
        ax_y = project(rotate(axis_y))
        ax_z = project(rotate(axis_z))

        c.create_rectangle(0, 0, w, h, fill="#0b1220", outline="")
        c.create_line(0, cy, w, cy, fill="#1f2937")
        c.create_line(cx, 0, cx, h, fill="#1f2937")

        # Draw edges with simple depth cue.
        for a, b in edges:
            x1, y1, z1 = rverts[a]
            x2, y2, z2 = rverts[b]
            zavg = (z1 + z2) * 0.5
            color = "#60a5fa" if zavg >= 0 else "#334155"
            c.create_line(x1, y1, x2, y2, fill=color, width=2)

        # Draw labeled body axes: X forward, Y right, Z up.
        ox, oy, _ = ax_o
        xx, xy, _ = ax_x
        yx, yy, _ = ax_y
        zx, zy, _ = ax_z
        c.create_line(ox, oy, xx, xy, fill="#ef4444", width=3)
        c.create_line(ox, oy, yx, yy, fill="#22c55e", width=3)
        c.create_line(ox, oy, zx, zy, fill="#3b82f6", width=3)
        c.create_text(xx + 8, xy, anchor="w", fill="#fecaca", font=("Consolas", 10, "bold"), text="+X")
        c.create_text(yx + 8, yy, anchor="w", fill="#bbf7d0", font=("Consolas", 10, "bold"), text="+Y")
        c.create_text(zx + 8, zy, anchor="w", fill="#bfdbfe", font=("Consolas", 10, "bold"), text="+Z")

        # Draw top-face contour brighter.
        top_face = [4, 5, 6, 7, 4]
        for i in range(len(top_face) - 1):
            x1, y1, _ = rverts[top_face[i]]
            x2, y2, _ = rverts[top_face[i + 1]]
            c.create_line(x1, y1, x2, y2, fill="#93c5fd", width=3)

        # Front arrow in orange.
        ax0, ay0, _ = rarrow[0]
        ax1, ay1, _ = rarrow[1]
        ax2, ay2, _ = rarrow[2]
        ax3, ay3, _ = rarrow[3]
        c.create_line(ax0, ay0, ax1, ay1, fill="#f59e0b", width=3)
        c.create_polygon(ax1, ay1, ax2, ay2, ax3, ay3, fill="#f59e0b", outline="#fbbf24")

        c.create_text(10, 10, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text="FRONT (+X)")
        c.create_text(
            10,
            h - 12,
            anchor="sw",
            fill="#cbd5e1",
            font=("Consolas", 10),
            text=f"P {pitch:+6.2f}  R {roll:+6.2f}  Y {yaw:+6.2f}",
        )

    @staticmethod
    def _apply_alignment(x: float, y: float, z: float) -> tuple[float, float, float]:
        # Must match firmware IMU_SENSOR_ALIGNMENT = IMU_ALIGN_CW270.
        return -y, x, z

    def _compute_fusion_euler(self, snapshot: TelemetryState) -> tuple[float, float, float] | None:
        if (not self._fusion_ready) or (self._fusion_ahrs is None) or (self._fusion_bias is None) or (np is None):
            return None

        now = time.monotonic()
        if self._fusion_last_monotonic <= 0.0:
            dt = 0.01
        else:
            dt = now - self._fusion_last_monotonic
            if dt < 0.001:
                dt = 0.001
            elif dt > 0.05:
                dt = 0.05
        self._fusion_last_monotonic = now

        gx_dps = snapshot.gx * IMU_GYRO_DPS_PER_LSB
        gy_dps = snapshot.gy * IMU_GYRO_DPS_PER_LSB
        gz_dps = snapshot.gz * IMU_GYRO_DPS_PER_LSB
        ax_g = snapshot.ax / IMU_ACCEL_LSB_PER_G
        ay_g = snapshot.ay / IMU_ACCEL_LSB_PER_G
        az_g = snapshot.az / IMU_ACCEL_LSB_PER_G

        gx_aligned, gy_aligned, gz_aligned = self._apply_alignment(gx_dps, gy_dps, gz_dps)
        ax_aligned, ay_aligned, az_aligned = self._apply_alignment(ax_g, ay_g, az_g)

        gyro_input = np.array([gx_aligned, gy_aligned, gz_aligned], dtype=float)
        accel_input = np.array([ax_aligned, ay_aligned, az_aligned], dtype=float)

        gyro = self._fusion_bias.update(gyro_input)
        self._fusion_ahrs.update_no_magnetometer(gyro, accel_input, dt)
        q = self._fusion_ahrs.quaternion
        if hasattr(q, "to_euler"):
            euler = q.to_euler()
            return float(-euler[1]), float(euler[0]), float(euler[2])

        if hasattr(imufusion, "quaternion_to_euler"):
            euler = imufusion.quaternion_to_euler(q)
            return float(-euler[1]), float(euler[0]), float(euler[2])

        # Last-resort conversion from quaternion elements.
        qw = float(q.w)
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        roll = math.degrees(math.atan2(qy * qz + qw * qx, qw * qw + qz * qz - 0.5))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qx * qz)))))
        yaw = math.degrees(math.atan2(qx * qy + qw * qz, qw * qw + qx * qx - 0.5))
        return float(-pitch), float(roll), float(yaw)

    def _compute_fusion_quaternion(self, snapshot: TelemetryState):
        if (not self._fusion_ready) or (self._fusion_ahrs is None) or (self._fusion_bias is None) or (np is None):
            return None

        now = time.monotonic()
        if self._fusion_last_monotonic <= 0.0:
            dt = 0.01
        else:
            dt = now - self._fusion_last_monotonic
            if dt < 0.001:
                dt = 0.001
            elif dt > 0.05:
                dt = 0.05
        self._fusion_last_monotonic = now

        gx_dps = snapshot.gx * IMU_GYRO_DPS_PER_LSB
        gy_dps = snapshot.gy * IMU_GYRO_DPS_PER_LSB
        gz_dps = snapshot.gz * IMU_GYRO_DPS_PER_LSB
        ax_g = snapshot.ax / IMU_ACCEL_LSB_PER_G
        ay_g = snapshot.ay / IMU_ACCEL_LSB_PER_G
        az_g = snapshot.az / IMU_ACCEL_LSB_PER_G

        gx_aligned, gy_aligned, gz_aligned = self._apply_alignment(gx_dps, gy_dps, gz_dps)
        ax_aligned, ay_aligned, az_aligned = self._apply_alignment(ax_g, ay_g, az_g)

        gyro_input = np.array([gx_aligned, gy_aligned, gz_aligned], dtype=float)
        accel_input = np.array([ax_aligned, ay_aligned, az_aligned], dtype=float)

        gyro = self._fusion_bias.update(gyro_input)
        self._fusion_ahrs.update_no_magnetometer(gyro, accel_input, dt)
        return self._fusion_ahrs.quaternion

    @staticmethod
    def _quaternion_to_euler_display(q) -> tuple[float, float, float]:
        if hasattr(q, "to_euler"):
            euler = q.to_euler()
            return float(-euler[1]), float(euler[0]), float(euler[2])

        if hasattr(imufusion, "quaternion_to_euler"):
            euler = imufusion.quaternion_to_euler(q)
            return float(-euler[1]), float(euler[0]), float(euler[2])

        qw = float(q.w)
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        roll = math.degrees(math.atan2(qy * qz + qw * qx, qw * qw + qz * qz - 0.5))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qx * qz)))))
        yaw = math.degrees(math.atan2(qx * qy + qw * qz, qw * qw + qx * qx - 0.5))
        return float(-pitch), float(roll), float(yaw)

    def update(self) -> None:
        with self.lock:
            snapshot = TelemetryState(
                sequence=self.state.sequence,
                pitch_deg=self.state.pitch_deg,
                roll_deg=self.state.roll_deg,
                yaw_deg=self.state.yaw_deg,
                gx=self.state.gx,
                gy=self.state.gy,
                gz=self.state.gz,
                ax=self.state.ax,
                ay=self.state.ay,
                az=self.state.az,
                pressure_pa=self.state.pressure_pa,
                altitude_cm=self.state.altitude_cm,
                rc_link_ok=self.state.rc_link_ok,
                rc_frames=self.state.rc_frames,
                channels=list(self.state.channels),
                last_update_monotonic=self.state.last_update_monotonic,
                connected=self.state.connected,
                port_name=self.state.port_name,
                status_text=self.state.status_text,
            )

        age_ms = (time.monotonic() - snapshot.last_update_monotonic) * 1000.0 if snapshot.last_update_monotonic else -1.0
        link = "OK" if snapshot.rc_link_ok else "--"
        ser = "UP" if snapshot.connected else "DN"
        port = snapshot.port_name if snapshot.port_name else "n/a"
        self.header_var.set(
            f"SEQ {snapshot.sequence:06d}   RC {link}   RF {snapshot.rc_frames:6d}   "
            f"SER {ser} ({port})   Age {age_ms:6.1f} ms"
        )
        self.imu_var.set(
            f"GX {snapshot.gx:6d}  GY {snapshot.gy:6d}  GZ {snapshot.gz:6d}    "
            f"AX {snapshot.ax:6d}  AY {snapshot.ay:6d}  AZ {snapshot.az:6d}"
        )
        self.baro_var.set(
            f"BARO P {snapshot.pressure_pa:7d} Pa   ALT {snapshot.altitude_cm / 100.0:+7.2f} m"
        )
        fusion_label = "FusionGUI:ON" if self._fusion_ready else "FusionGUI:OFF"
        if (not self._fusion_ready) and imufusion_error:
            fusion_label = f"{fusion_label} ({imufusion_error})"
        self.footer_var.set(f"{snapshot.status_text} | {fusion_label} | Scale: channels map 999..2000 to -1..+1")

        fusion_quaternion = self._compute_fusion_quaternion(snapshot)
        if fusion_quaternion is not None:
            source_pitch, source_roll, source_yaw = self._quaternion_to_euler_display(fusion_quaternion)
        else:
            source_pitch, source_roll, source_yaw = snapshot.pitch_deg, snapshot.roll_deg, snapshot.yaw_deg

        self._draw_attitude(source_pitch, source_roll, source_yaw)
        self._draw_sticks(snapshot.channels)
        self._draw_board_3d(source_pitch, source_roll, source_yaw, fusion_quaternion)
        self._draw_channels(snapshot.channels)

        self.root.after(33, self.update)


def main() -> int:
    parser = argparse.ArgumentParser(description="Graphical telemetry viewer for serial telemetry")
    parser.add_argument("--port", default="COM6", help="Serial port, for example COM9 or auto")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    if args.port.lower() == "auto":
        args.port = _pick_port()

    if not args.port:
        ports = list(_list_ports())
        if not ports:
            print("No serial ports found.")
        else:
            print("Available serial ports:")
            for p in ports:
                print(f"  {p}")
        return 1

    state = TelemetryState()
    lock = threading.Lock()
    reader = SerialReader(args.port, args.baud, state, lock)
    reader.start()

    root = tk.Tk()
    app = TelemetryApp(root, state, lock)

    def _on_close() -> None:
        reader.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    app.update()
    root.mainloop()

    if reader.error_message:
        print(f"Serial error on {args.port}: {reader.error_message}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
