#!/usr/bin/env python3
"""Minimal 3D body telemetry viewer for KAKUTEH7.

This script is standalone and does not depend on read_serial_gui.py.
It reads telemetry from serial/TCP and renders only the 3D body using
pitch/roll/yaw values sent by firmware.

This is a legacy debug view, not the authoritative pose-display path.

Usage:
  python scripts/read_serial_3body.py --port COM6
  python scripts/read_serial_3body.py --port auto
  python scripts/read_serial_3body.py --port 127.0.0.1:9000

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required. Install it with: pip install pyserial") from exc

try:
    import tm_protocol
except Exception:  # pragma: no cover
    tm_protocol = None


SOF1 = 0xA5
SOF2 = 0x5A
PACKET_TYPE_TELEMETRY = 0x01

TELEMETRY_PAYLOAD_LEN_V1 = 73
TELEMETRY_PAYLOAD_LEN_V2 = 83
TELEMETRY_PAYLOAD_LEN_V3 = 84
TELEMETRY_PAYLOAD_LEN_V4 = 92
TELEMETRY_PAYLOAD_LEN_V5 = 94
TELEMETRY_PAYLOAD_LEN_V6 = 98
TELEMETRY_PAYLOAD_LEN_V7 = 104
TELEMETRY_PAYLOAD_LEN_V8 = 116
TELEMETRY_PAYLOAD_LEN_V9 = 122
TELEMETRY_PAYLOAD_LEN_V10 = 130

TELEMETRY_STRUCT_V1 = struct.Struct("<Iiii6hBIIi16H")
TELEMETRY_STRUCT_V2 = struct.Struct("<Iiii6hBIIi16HBB4H")
TELEMETRY_STRUCT_V3 = struct.Struct("<Iiii6hBIIi16HBBB4H")
TELEMETRY_STRUCT_V4 = struct.Struct("<Iiii9hBIIi16HBBB5H")
TELEMETRY_STRUCT_V5 = struct.Struct("<Iiii9hBIIiH16HBBB5H")
TELEMETRY_STRUCT_V6 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBB")
TELEMETRY_STRUCT_V7 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhh")
TELEMETRY_STRUCT_V8 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhh")
TELEMETRY_STRUCT_V9 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhhhhh")
TELEMETRY_STRUCT_V10 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhhhhhhhhh")

TM_COMBINED_FAST_STRUCT = struct.Struct("<hhhhhhHHHHHHHHHHBBIIIII")

SERIAL_READ_TIMEOUT_S = 0.03
SOCKET_READ_TIMEOUT_S = 0.05
GUI_UPDATE_INTERVAL_MS = 16


@dataclass
class AttitudeState:
    sequence: int = 0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    last_update_monotonic: float = 0.0
    connected: bool = False
    port_name: str = ""
    status_text: str = "Sniffing serial ports..."


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


def _parse_tcp_endpoint(port_spec: str) -> tuple[str, int] | None:
    text = (port_spec or "").strip()
    if not text or text.lower() == "auto":
        return None
    if text.upper().startswith("COM"):
        return None
    if ":" not in text:
        return None

    host, port_text = text.rsplit(":", 1)
    host = host.strip()
    port_text = port_text.strip()
    if not host or not port_text.isdigit():
        return None

    port = int(port_text)
    if port < 1 or port > 65535:
        return None

    return host, port


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

        if payload_len == TELEMETRY_PAYLOAD_LEN_V1:
            yield TELEMETRY_STRUCT_V1.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V2:
            yield TELEMETRY_STRUCT_V2.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V3:
            yield TELEMETRY_STRUCT_V3.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V4:
            yield TELEMETRY_STRUCT_V4.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V5:
            yield TELEMETRY_STRUCT_V5.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V6:
            yield TELEMETRY_STRUCT_V6.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V7:
            yield TELEMETRY_STRUCT_V7.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V8:
            yield TELEMETRY_STRUCT_V8.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V9:
            yield TELEMETRY_STRUCT_V9.unpack(payload)
            continue
        if payload_len == TELEMETRY_PAYLOAD_LEN_V10:
            yield TELEMETRY_STRUCT_V10.unpack(payload)
            continue


def _extract_euler_from_legacy(values: tuple[int, ...]) -> tuple[int, float, float, float] | None:
    if len(values) < 4:
        return None
    sequence = int(values[0])
    pitch_deg = int(values[1]) / 100.0
    roll_deg = int(values[2]) / 100.0
    yaw_deg = int(values[3]) / 100.0
    return sequence, pitch_deg, roll_deg, yaw_deg


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, state: AttitudeState, lock: threading.Lock):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.tcp_endpoint = _parse_tcp_endpoint(port)
        self.state = state
        self.lock = lock
        self.running = True
        self.error_message = ""

    def stop(self) -> None:
        self.running = False

    def _apply_attitude(self, candidate: str, sequence: int, pitch_deg: float, roll_deg: float, yaw_deg: float) -> None:
        with self.lock:
            self.state.sequence = int(sequence)
            self.state.pitch_deg = float(pitch_deg)
            self.state.roll_deg = float(roll_deg)
            self.state.yaw_deg = float(yaw_deg)
            self.state.last_update_monotonic = time.monotonic()
            self.state.connected = True
            self.state.port_name = candidate
            self.state.status_text = f"Connected: {candidate}"

    def _candidate_ports(self) -> list[str]:
        if self.tcp_endpoint is not None:
            host, tcp_port = self.tcp_endpoint
            return [f"{host}:{tcp_port}"]

        ports: list[str] = []
        seen: set[str] = set()

        if self.port.lower() != "auto":
            ports.append(self.port)
            seen.add(self.port.upper())

        for p in list_ports.comports():
            if "STLink" in p.description or "ST-LINK" in p.description:
                continue
            key = p.device.upper()
            if key in seen:
                continue
            ports.append(p.device)
            seen.add(key)
        return ports

    def run(self) -> None:
        while self.running:
            if self.tcp_endpoint is not None:
                host, tcp_port = self.tcp_endpoint
                candidate = f"{host}:{tcp_port}"
                try:
                    with socket.create_connection((host, tcp_port), timeout=2.0) as sock:
                        sock.settimeout(SOCKET_READ_TIMEOUT_S)
                        parser = tm_protocol.StreamParser() if tm_protocol is not None else None
                        with self.lock:
                            self.state.connected = True
                            self.state.port_name = candidate
                            self.state.status_text = f"Connected: {candidate}"

                        buffer = bytearray()
                        while self.running:
                            try:
                                chunk = sock.recv(4096)
                            except TimeoutError:
                                chunk = b""
                            except OSError:
                                break

                            if not chunk:
                                continue

                            if parser is not None:
                                for frame in parser.feed(chunk):
                                    if frame.packet_type != int(tm_protocol.PacketType.TM_COMBINED_FAST):
                                        continue
                                    if len(frame.payload) != TM_COMBINED_FAST_STRUCT.size:
                                        continue
                                    unpacked = TM_COMBINED_FAST_STRUCT.unpack(frame.payload)
                                    roll_cd = int(unpacked[3])
                                    pitch_cd = int(unpacked[4])
                                    yaw_cd = int(unpacked[5])
                                    self._apply_attitude(
                                        candidate,
                                        int(frame.sequence),
                                        pitch_cd / 100.0,
                                        roll_cd / 100.0,
                                        yaw_cd / 100.0,
                                    )
                            else:
                                buffer.extend(chunk)
                                for values in _decode_frames(buffer):
                                    result = _extract_euler_from_legacy(values)
                                    if result is None:
                                        continue
                                    sequence, pitch_deg, roll_deg, yaw_deg = result
                                    self._apply_attitude(candidate, sequence, pitch_deg, roll_deg, yaw_deg)

                except OSError as exc:
                    self.error_message = str(exc)
                    with self.lock:
                        self.state.connected = False
                        self.state.status_text = f"Socket lost: {candidate} (retrying...)"
                    time.sleep(0.3)
                continue

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
                    with serial.Serial(candidate, self.baud, timeout=SERIAL_READ_TIMEOUT_S) as ser:
                        connected_once = True
                        with self.lock:
                            self.state.connected = True
                            self.state.port_name = candidate
                            self.state.status_text = f"Connected: {candidate}"

                        buffer = bytearray()
                        while self.running:
                            chunk = ser.read(ser.in_waiting or 1)
                            if not chunk:
                                continue
                            buffer.extend(chunk)
                            for values in _decode_frames(buffer):
                                result = _extract_euler_from_legacy(values)
                                if result is None:
                                    continue
                                sequence, pitch_deg, roll_deg, yaw_deg = result
                                self._apply_attitude(candidate, sequence, pitch_deg, roll_deg, yaw_deg)

                except serial.SerialException as exc:
                    self.error_message = str(exc)
                    with self.lock:
                        self.state.connected = False
                        self.state.status_text = f"Port lost: {candidate} (sniffing...)"
                    continue
                except Exception as exc:  # pragma: no cover
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


class Body3DApp:
    def __init__(self, root: tk.Tk, state: AttitudeState, lock: threading.Lock):
        self.root = root
        self.state = state
        self.lock = lock

        root.title("KAKUTEH7 3D Body Only")
        root.geometry("820x620")
        root.minsize(640, 420)

        self.header_var = tk.StringVar(value="Waiting for telemetry...")
        tk.Label(root, textvariable=self.header_var, font=("Segoe UI", 12, "bold"), anchor="w").pack(fill=tk.X, padx=10, pady=(8, 2))

        self.canvas = tk.Canvas(root, bg="#0b1220", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=(4, 10))

    def _draw_board_3d(self, pitch: float, roll: float, yaw: float) -> None:
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        cx = w / 2.0
        cy = h / 2.0

        bw = 90.0
        bd = 90.0
        bh = 8.0
        z_top = -bh
        z_bottom = bh

        verts = [
            (-bw, -bd, z_top),
            (bw, -bd, z_top),
            (bw, bd, z_top),
            (-bw, bd, z_top),
            (-bw, -bd, z_bottom),
            (bw, -bd, z_bottom),
            (bw, bd, z_bottom),
            (-bw, bd, z_bottom),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        arrow = [
            (0.0, 0.0, z_top - 2.0),
            (120.0, 0.0, z_top - 2.0),
            (95.0, -16.0, z_top - 2.0),
            (95.0, 16.0, z_top - 2.0),
        ]

        pr = math.radians(pitch)
        rr = math.radians(roll)
        yr = math.radians(yaw)

        cp = math.cos(pr)
        sp = math.sin(pr)
        cr = math.cos(rr)
        sr = math.sin(rr)
        cyaw = math.cos(yr)
        syaw = math.sin(yr)

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
            return (
                r00 * x + r01 * y + r02 * z,
                r10 * x + r11 * y + r12 * z,
                r20 * x + r21 * y + r22 * z,
            )

        def project(v: tuple[float, float, float]) -> tuple[float, float, float]:
            x, y, z = v
            x_view = (1.00 * y) + (0.10 * x)
            y_view = (-1.00 * z) + (0.05 * x)
            z_view = -1.00 * x
            dist = 520.0
            scale = dist / (dist - z_view)
            sx = cx + x_view * scale * 1.2
            sy = cy - y_view * scale * 1.2
            return sx, sy, z_view

        rverts = [project(rotate(v)) for v in verts]
        rarrow = [project(rotate(v)) for v in arrow]

        c.create_rectangle(0, 0, w, h, fill="#0b1220", outline="")

        p0, p1, p2, p3 = rverts[0], rverts[1], rverts[2], rverts[3]
        b0, b1, b2, b3 = rverts[4], rverts[5], rverts[6], rverts[7]

        c.create_polygon(b0[0], b0[1], b1[0], b1[1], b2[0], b2[1], b3[0], b3[1], fill="#0f172a", outline="")
        c.create_polygon(p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], fill="#1e3a8a", stipple="gray25", outline="")

        def lerp2(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float]:
            return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t

        for i in range(1, 7):
            t = i / 7.0
            a = lerp2(p0, p3, t)
            b = lerp2(p1, p2, t)
            c.create_line(a[0], a[1], b[0], b[1], fill="#93c5fd", width=1)

            c1 = lerp2(p0, p1, t)
            c2 = lerp2(p3, p2, t)
            c.create_line(c1[0], c1[1], c2[0], c2[1], fill="#60a5fa", width=1)

        for a_idx, b_idx in edges:
            x1, y1, z1 = rverts[a_idx]
            x2, y2, z2 = rverts[b_idx]
            zavg = (z1 + z2) * 0.5
            color = "#60a5fa" if zavg >= 0 else "#334155"
            c.create_line(x1, y1, x2, y2, fill=color, width=2)

        c.create_line(p0[0], p0[1], p1[0], p1[1], fill="#93c5fd", width=3)
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#93c5fd", width=3)
        c.create_line(p2[0], p2[1], p3[0], p3[1], fill="#93c5fd", width=3)
        c.create_line(p3[0], p3[1], p0[0], p0[1], fill="#93c5fd", width=3)

        ax0, ay0, _ = rarrow[0]
        ax1, ay1, _ = rarrow[1]
        ax2, ay2, _ = rarrow[2]
        ax3, ay3, _ = rarrow[3]
        c.create_line(ax0, ay0, ax1, ay1, fill="#f59e0b", width=3)
        c.create_polygon(ax1, ay1, ax2, ay2, ax3, ay3, fill="#f59e0b", outline="#fbbf24")

        c.create_text(12, 12, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text="3D BODY (LEGACY EULER DEBUG VIEW)")
        c.create_text(12, h - 14, anchor="sw", fill="#cbd5e1", font=("Consolas", 11), text=f"P {pitch:+6.2f}  R {roll:+6.2f}  Yrel {yaw:+6.2f}")

    def update(self) -> None:
        with self.lock:
            sequence = self.state.sequence
            pitch = self.state.pitch_deg
            roll = self.state.roll_deg
            yaw = self.state.yaw_deg
            connected = self.state.connected
            port_name = self.state.port_name
            status = self.state.status_text
            last = self.state.last_update_monotonic

        age_ms = (time.monotonic() - last) * 1000.0 if last else -1.0
        link = "UP" if connected else "DN"
        port_text = port_name if port_name else "n/a"
        self.header_var.set(f"SEQ {sequence:06d}   LINK {link} ({port_text})   Age {age_ms:6.1f} ms   {status}")

        self._draw_board_3d(pitch, roll, yaw)
        self.root.after(GUI_UPDATE_INTERVAL_MS, self.update)


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone 3D body-only telemetry viewer")
    parser.add_argument("--port", default="COM6", help="Serial COM port (e.g. COM6/auto) or TCP host:port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--verbose", action="store_true", help="Print serial diagnostics to console")
    args = parser.parse_args()

    if args.port.lower() == "auto":
        args.port = _pick_port()

    if not args.port:
        ports = list(_list_ports())
        if args.verbose:
            if not ports:
                print("No serial ports found.")
            else:
                print("Available serial ports:")
                for p in ports:
                    print(f"  {p}")
        return 1

    state = AttitudeState()
    lock = threading.Lock()
    reader = SerialReader(args.port, args.baud, state, lock)
    reader.start()

    root = tk.Tk()
    app = Body3DApp(root, state, lock)

    def _on_close() -> None:
        reader.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    app.update()
    root.mainloop()

    if reader.error_message:
        if args.verbose:
            print(f"Serial error on {args.port}: {reader.error_message}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
