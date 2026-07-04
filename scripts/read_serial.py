#!/usr/bin/env python3
"""Read line-based telemetry from the KAKUTEH7 serial stream.

Usage:
  python scripts/read_serial.py --port COM9

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from typing import Iterable


class ProbeTracker:
    def __init__(self) -> None:
        self.prev_sequence: int | None = None
        self.prev_roll_deg: float | None = None
        self.prev_pitch_deg: float | None = None
        self.last_result = "PROBE: waiting for motion"

    def update(self, sequence: int, roll_deg: float, pitch_deg: float, gx: int, gy: int, gz: int) -> str:
        if self.prev_sequence is None:
            self.prev_sequence = sequence
            self.prev_roll_deg = roll_deg
            self.prev_pitch_deg = pitch_deg
            return self.last_result

        dseq = sequence - self.prev_sequence
        if dseq <= 0:
            dseq = 1

        droll = roll_deg - (self.prev_roll_deg if self.prev_roll_deg is not None else roll_deg)
        dpitch = pitch_deg - (self.prev_pitch_deg if self.prev_pitch_deg is not None else pitch_deg)

        self.prev_sequence = sequence
        self.prev_roll_deg = roll_deg
        self.prev_pitch_deg = pitch_deg

        # Require clear single-axis motion before reporting mapping.
        axes = [("GX", gx), ("GY", gy), ("GZ", gz)]
        axes_sorted = sorted(axes, key=lambda item: abs(item[1]), reverse=True)
        primary_name, primary_val = axes_sorted[0]
        secondary_val = abs(axes_sorted[1][1])

        if abs(primary_val) < 40:
            self.last_result = "PROBE: move one axis faster (gyro < 40 dps raw)"
            return self.last_result

        if abs(primary_val) < (secondary_val * 2):
            self.last_result = "PROBE: mixed-axis motion, isolate a single-axis move"
            return self.last_result

        angle_name = "ROLL" if abs(droll) >= abs(dpitch) else "PITCH"
        angle_delta = droll if angle_name == "ROLL" else dpitch

        if abs(angle_delta) < 0.05:
            self.last_result = f"PROBE: {primary_name} active but angle response is small"
            return self.last_result

        gyro_sign = "+" if primary_val >= 0 else "-"
        angle_sign = "+" if angle_delta >= 0 else "-"
        self.last_result = f"PROBE: {primary_name}{gyro_sign} -> {angle_name}{angle_sign}  dR={droll:+.2f} dP={dpitch:+.2f}"
        return self.last_result

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - host dependency only
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


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

SOF1 = 0xA5
SOF2 = 0x5A
PACKET_TYPE_TELEMETRY = 0x01
TELEMETRY_PAYLOAD_LEN = 73
TELEMETRY_STRUCT = struct.Struct("<Iiii6hBIIi16H")


def _checksum(packet_type: int, payload: bytes) -> int:
    result = packet_type ^ len(payload)
    for byte in payload:
        result ^= byte
    return result & 0xFF


def _channel_to_norm(raw: int) -> float:
    clamped = max(999, min(2000, raw))
    return ((clamped - 999) / 1001.0) * 2.0 - 1.0


def _build_stick(label: str, x_norm: float, y_norm: float) -> list[str]:
    size = 11
    center = size // 2
    grid = [[" " for _ in range(size)] for _ in range(size)]

    for i in range(size):
        grid[center][i] = "-"
        grid[i][center] = "|"
    grid[center][center] = "+"

    x_col = int(round((x_norm + 1.0) * 0.5 * (size - 1)))
    y_row = int(round((1.0 - (y_norm + 1.0) * 0.5) * (size - 1)))
    x_col = max(0, min(size - 1, x_col))
    y_row = max(0, min(size - 1, y_row))
    grid[y_row][x_col] = "O"

    lines = [f"{label}  X:{x_norm:+.2f} Y:{y_norm:+.2f}"]
    lines.extend("[" + "".join(row) + "]" for row in grid)
    lines.append("  -1.0         +1.0")
    return lines


def _build_orientation(pitch_deg: float, roll_deg: float, yaw_deg: float) -> list[str]:
    width = 25
    height = 11
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0

    # Compute gravity direction in body frame from Euler angles.
    # This remains numerically stable near vertical attitudes.
    pitch_rad = pitch_deg * math.pi / 180.0
    roll_rad = roll_deg * math.pi / 180.0
    gx = -math.sin(pitch_rad)
    gy = math.sin(roll_rad) * math.cos(pitch_rad)
    gz = math.cos(roll_rad) * math.cos(pitch_rad)

    # Projection of gravity onto X/Y gives intuitive tilt indicator.
    # X axis: nose down(+), nose up(-). Y axis: right wing down(+), left wing down(-).
    grid = [[" " for _ in range(width)] for _ in range(height)]
    cxi = int(round(cx))
    cyi = int(round(cy))
    for x in range(width):
        grid[cyi][x] = "-"
    for y in range(height):
        grid[y][cxi] = "|"
    grid[cyi][cxi] = "+"

    # Scale vector so full deflection fits in panel.
    vx = int(round(gx * (width // 2 - 1)))
    vy = int(round(gy * (height // 2 - 1)))
    dot_x = max(0, min(width - 1, cxi + vx))
    dot_y = max(0, min(height - 1, cyi - vy))
    grid[dot_y][dot_x] = "O"

    # Keep a classic horizon only when not too close to gimbal-lock region.
    if abs(pitch_deg) < 75.0:
        pitch_shift = max(-4.0, min(4.0, pitch_deg / 5.0))
        roll_slope = max(-1.0, min(1.0, roll_deg / 45.0))
        for x in range(width):
            y_float = cy + pitch_shift + (x - cx) * roll_slope * 0.35
            y_line = int(round(y_float))
            if 0 <= y_line < height and grid[y_line][x] == " ":
                grid[y_line][x] = "."
        mode = "MODE: blended (horizon+tilt)"
    else:
        mode = "MODE: vertical-safe tilt"

    yaw_norm = (yaw_deg + 180.0) % 360.0 - 180.0
    yaw_col = int(round((yaw_norm + 180.0) * (width - 1) / 360.0))
    yaw_col = max(0, min(width - 1, yaw_col))
    yaw_bar = ["." for _ in range(width)]
    yaw_bar[yaw_col] = "^"

    lines = [f"BOARD ORIENTATION  P:{pitch_deg:+6.2f} R:{roll_deg:+6.2f} Y:{yaw_deg:+6.2f}"]
    lines.append(mode)
    lines.extend("[" + "".join(row) + "]" for row in grid)
    lines.append("[" + "".join(yaw_bar) + "]")
    lines.append(f" Tilt g-body: X:{gx:+.2f} Y:{gy:+.2f} Z:{gz:+.2f}")
    lines.append(" Yaw: -180                                +180")
    return lines


def _render_dashboard(
    values: tuple[int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int],
    probe_result: str,
) -> str:
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
        ch1,
        ch2,
        ch3,
        ch4,
        ch5,
        ch6,
        ch7,
        ch8,
        ch9,
        ch10,
        ch11,
        ch12,
        ch13,
        ch14,
        ch15,
        ch16,
    ) = values

    left_x = _channel_to_norm(ch4)   # yaw
    left_y = _channel_to_norm(ch3)   # throttle
    right_x = _channel_to_norm(ch1)  # roll
    right_y = _channel_to_norm(ch2)  # pitch

    left_stick = _build_stick("LEFT STICK  CH4/CH3", left_x, left_y)
    right_stick = _build_stick("RIGHT STICK CH1/CH2", right_x, right_y)
    orient = _build_orientation(pitch_cd / 100.0, roll_cd / 100.0, yaw_cd / 100.0)

    stick_lines = [f"{l}   {r}" for l, r in zip(left_stick, right_stick)]

    ch_rest = [
        f"CH5:{ch5:4d}",
        f"CH6:{ch6:4d}",
        f"CH7:{ch7:4d}",
        f"CH8:{ch8:4d}",
        f"CH9:{ch9:4d}",
        f"CH10:{ch10:4d}",
        f"CH11:{ch11:4d}",
        f"CH12:{ch12:4d}",
        f"CH13:{ch13:4d}",
        f"CH14:{ch14:4d}",
        f"CH15:{ch15:4d}",
        f"CH16:{ch16:4d}",
    ]

    rest_lines = []
    for i in range(0, len(ch_rest), 3):
        rest_lines.append("   ".join(ch_rest[i:i + 3]))

    rc_state = "OK" if rc_link else "--"
    header = (
        f"SEQ:{sequence:06d}  RC:{rc_state}  RF:{rc_frames:6d}  "
        f"P:{pitch_cd / 100.0:+6.2f} R:{roll_cd / 100.0:+6.2f} Y:{yaw_cd / 100.0:+6.2f}"
    )
    imu = f"GX:{gx:6d} GY:{gy:6d} GZ:{gz:6d}  AX:{ax:6d} AY:{ay:6d} AZ:{az:6d}"
    ch_primary = f"CH1:{ch1:4d} CH2:{ch2:4d} CH3:{ch3:4d} CH4:{ch4:4d}"

    lines = [header, imu, ch_primary, ""]
    lines.append(probe_result)
    lines.append("")
    lines.extend(stick_lines)
    lines.append("")
    lines.extend(orient)
    lines.append("")
    lines.append(f"BARO: {pressure_pa:7d} Pa   ALT: {altitude_cm / 100.0:+7.2f} m")
    lines.extend(rest_lines)
    lines.append("")
    lines.append("Scale: 999..2000 maps to -1.0..+1.0")
    return "\n".join(lines)


def _decode_frames(buffer: bytearray) -> Iterable[tuple[int, ...]]:
    while True:
        start = buffer.find(bytes((SOF1, SOF2)))
        if start < 0:
            if buffer:
                # Keep a tiny tail in case SOF1/SOF2 straddles a read boundary.
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

        payload = bytes(buffer[4:4 + payload_len])
        checksum = buffer[4 + payload_len]
        del buffer[:frame_len]

        if checksum != _checksum(packet_type, payload):
            continue

        if packet_type != PACKET_TYPE_TELEMETRY:
            continue

        if payload_len != TELEMETRY_PAYLOAD_LEN:
            continue

        yield TELEMETRY_STRUCT.unpack(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read telemetry from a serial COM port.")
    parser.add_argument("--port", help="Serial port, for example COM6 or 'auto'", default="COM6")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    if args.port.lower() == "auto":
        args.port = _pick_port()

    if not args.port:
        ports = list(_list_ports())
        if not ports:
            print("No serial ports found.", file=sys.stderr)
        else:
            print("Available serial ports:", file=sys.stderr)
            for port in ports:
                print(f"  {port}", file=sys.stderr)
        print("\nExample: python scripts/read_serial.py --port COM9", file=sys.stderr)
        return 1

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print(f"Listening on {args.port} at {args.baud} baud...", file=sys.stderr)
            buffer = bytearray()
            dashboard_boot = False
            last_packet_time = time.monotonic()
            probe = ProbeTracker()
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buffer.extend(chunk)
                saw_packet = False
                for values in _decode_frames(buffer):
                    sequence = int(values[0])
                    roll_deg = int(values[2]) / 100.0
                    pitch_deg = int(values[1]) / 100.0
                    gx = int(values[4])
                    gy = int(values[5])
                    gz = int(values[6])
                    probe_result = probe.update(sequence, roll_deg, pitch_deg, gx, gy, gz)
                    rendered = _render_dashboard(values, probe_result)
                    if not dashboard_boot:
                        sys.stdout.write("\x1b[2J")
                        dashboard_boot = True
                    sys.stdout.write("\x1b[H\x1b[J" + rendered)
                    sys.stdout.flush()
                    saw_packet = True
                    last_packet_time = time.monotonic()
                if not saw_packet and time.monotonic() - last_packet_time >= 5.0:
                    last_packet_time = time.monotonic()
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
