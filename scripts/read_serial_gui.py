#!/usr/bin/env python3
"""Graphical telemetry viewer for KAKUTEH7 serial stream.

Usage:
    python scripts/read_serial_gui.py --port COM6

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import os
import struct
import threading
import time
import tkinter as tk
from collections import deque
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
PACKET_TYPE_MOTOR_TEST = 0x10
PACKET_TYPE_DISPLAY_FILTER = 0x11
TELEMETRY_PAYLOAD_LEN_V1 = 73
TELEMETRY_PAYLOAD_LEN_V2 = 83
TELEMETRY_PAYLOAD_LEN_V3 = 84
TELEMETRY_PAYLOAD_LEN_V4 = 92
TELEMETRY_PAYLOAD_LEN_V5 = 94
TELEMETRY_PAYLOAD_LEN_V6 = 98
TELEMETRY_STRUCT_V1 = struct.Struct("<Iiii6hBIIi16H")
TELEMETRY_STRUCT_V2 = struct.Struct("<Iiii6hBIIi16HBB4H")
TELEMETRY_STRUCT_V3 = struct.Struct("<Iiii6hBIIi16HBBB4H")
TELEMETRY_STRUCT_V4 = struct.Struct("<Iiii9hBIIi16HBBB5H")
TELEMETRY_STRUCT_V5 = struct.Struct("<Iiii9hBIIiH16HBBB5H")
TELEMETRY_STRUCT_V6 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBB")
IMU_ACCEL_LSB_PER_G = 2048.0
IMU_GYRO_DPS_PER_LSB = 2000.0 / 32768.0
LAYOUT_SCHEMA_VERSION = 2


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
        if payload_len == TELEMETRY_PAYLOAD_LEN_V1:
            unpacked = TELEMETRY_STRUCT_V1.unpack(payload)
            yield unpacked + (0, 0, 0, 1000, 1000, 1000, 1000)
            continue

        if payload_len == TELEMETRY_PAYLOAD_LEN_V2:
            unpacked = TELEMETRY_STRUCT_V2.unpack(payload)
            # Insert gui_test_active=0 between motors_armed and motor us fields.
            yield unpacked[:32] + (0,) + unpacked[32:]
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


def _build_motor_test_packet(motor_index: int, motor_us: int) -> bytes:
    payload = bytes(
        (
            max(1, min(4, int(motor_index))),
            int(max(1000, min(2000, int(motor_us)))) & 0xFF,
            (int(max(1000, min(2000, int(motor_us)))) >> 8) & 0xFF,
        )
    )
    packet_type = PACKET_TYPE_MOTOR_TEST
    frame = bytearray((SOF1, SOF2, packet_type, len(payload)))
    frame.extend(payload)
    frame.append(_checksum(packet_type, payload))
    return bytes(frame)


def _build_motor_test_ascii(motor_index: int, motor_us: int) -> bytes:
    idx = max(1, min(4, int(motor_index)))
    us = max(1000, min(2000, int(motor_us)))
    return f"MTEST {idx} {us}\n".encode("ascii")


def _build_display_filter_packet(cutoff_hz: int) -> bytes:
    cutoff = max(5, min(150, int(cutoff_hz)))
    payload = bytes((cutoff & 0xFF, (cutoff >> 8) & 0xFF))
    packet_type = PACKET_TYPE_DISPLAY_FILTER
    frame = bytearray((SOF1, SOF2, packet_type, len(payload)))
    frame.extend(payload)
    frame.append(_checksum(packet_type, payload))
    return bytes(frame)


def _build_display_filter_ascii(cutoff_hz: int) -> bytes:
    cutoff = max(5, min(150, int(cutoff_hz)))
    return f"DFLT {cutoff}\n".encode("ascii")


@dataclass
class TelemetryState:
    sequence: int = 0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    gx_filtered: int = 0
    gy_filtered: int = 0
    gz_filtered: int = 0
    ax: int = 0
    ay: int = 0
    az: int = 0
    display_filter_cutoff_hz: int = 40
    pressure_pa: int = 0
    altitude_cm: int = 0
    battery_mv: int | None = None
    battery_voltage_v: float | None = None
    battery_adc_raw: int = 0
    battery_adc_channel: int = 255
    battery_adc_ready: int = 0
    rc_link_ok: bool = False
    rc_frames: int = 0
    channels: list[int] = field(default_factory=lambda: [0] * 16)
    motor_mode: int = 0
    motors_armed: int = 0
    gui_test_active: int = 0
    gyro_filter_available: bool = False
    motors_us: list[int] = field(default_factory=lambda: [1000, 1000, 1000, 1000])
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
        self._tx_lock = threading.Lock()
        self._tx_queue: list[bytes] = []

    def stop(self) -> None:
        self.running = False

    def queue_command(self, frame: bytes) -> None:
        if not frame:
            return
        with self._tx_lock:
            self._tx_queue.append(frame)

    def _drain_tx(self, ser: serial.Serial) -> None:
        pending: list[bytes]
        with self._tx_lock:
            pending = self._tx_queue
            self._tx_queue = []

        for frame in pending:
            try:
                ser.write(frame)
            except Exception:
                self.error_message = "Failed to send command frame"
                return

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
                            self._drain_tx(ser)
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
                                if len(values) == 45:
                                    (
                                        sequence,
                                        pitch_cd,
                                        roll_cd,
                                        yaw_cd,
                                        gx,
                                        gy,
                                        gz,
                                        gx_filtered,
                                        gy_filtered,
                                        gz_filtered,
                                        ax,
                                        ay,
                                        az,
                                        rc_link,
                                        rc_frames,
                                        pressure_pa,
                                        altitude_cm,
                                        battery_mv,
                                        *channels,
                                    ) = values
                                    display_filter_cutoff_hz = int(channels[23])
                                    battery_adc_raw = int(channels[24])
                                    battery_adc_channel = int(channels[25])
                                    battery_adc_ready = int(channels[26])
                                    battery_mv = int(battery_mv)
                                    battery_voltage_v = float(battery_mv) * 0.001
                                    gyro_filter_available = True
                                elif len(values) == 42:
                                    (
                                        sequence,
                                        pitch_cd,
                                        roll_cd,
                                        yaw_cd,
                                        gx,
                                        gy,
                                        gz,
                                        gx_filtered,
                                        gy_filtered,
                                        gz_filtered,
                                        ax,
                                        ay,
                                        az,
                                        rc_link,
                                        rc_frames,
                                        pressure_pa,
                                        altitude_cm,
                                        battery_mv,
                                        *channels,
                                    ) = values
                                    display_filter_cutoff_hz = int(channels[23])
                                    battery_adc_raw = 0
                                    battery_adc_channel = 255
                                    battery_adc_ready = 0
                                    battery_mv = int(battery_mv)
                                    battery_voltage_v = float(battery_mv) * 0.001
                                    gyro_filter_available = True
                                elif len(values) == 41:
                                    (
                                        sequence,
                                        pitch_cd,
                                        roll_cd,
                                        yaw_cd,
                                        gx,
                                        gy,
                                        gz,
                                        gx_filtered,
                                        gy_filtered,
                                        gz_filtered,
                                        ax,
                                        ay,
                                        az,
                                        rc_link,
                                        rc_frames,
                                        pressure_pa,
                                        altitude_cm,
                                        *channels,
                                    ) = values
                                    display_filter_cutoff_hz = int(channels[23])
                                    battery_adc_raw = 0
                                    battery_adc_channel = 255
                                    battery_adc_ready = 0
                                    battery_mv = None
                                    battery_voltage_v = None
                                    gyro_filter_available = True
                                else:
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
                                    gx_filtered = gx
                                    gy_filtered = gy
                                    gz_filtered = gz
                                    display_filter_cutoff_hz = 40
                                    battery_adc_raw = 0
                                    battery_adc_channel = 255
                                    battery_adc_ready = 0
                                    battery_mv = None
                                    battery_voltage_v = None
                                    gyro_filter_available = False

                                motor_mode = int(channels[16])
                                motors_armed = int(channels[17])
                                gui_test_active = int(channels[18])
                                motors_us = [int(v) for v in channels[19:23]]
                                channels = channels[:16]

                                with self.lock:
                                    self.state.sequence = int(sequence)
                                    self.state.pitch_deg = int(pitch_cd) / 100.0
                                    self.state.roll_deg = int(roll_cd) / 100.0
                                    self.state.yaw_deg = int(yaw_cd) / 100.0
                                    self.state.gx = int(gx)
                                    self.state.gy = int(gy)
                                    self.state.gz = int(gz)
                                    self.state.gx_filtered = int(gx_filtered)
                                    self.state.gy_filtered = int(gy_filtered)
                                    self.state.gz_filtered = int(gz_filtered)
                                    self.state.ax = int(ax)
                                    self.state.ay = int(ay)
                                    self.state.az = int(az)
                                    self.state.display_filter_cutoff_hz = int(display_filter_cutoff_hz)
                                    self.state.pressure_pa = int(pressure_pa)
                                    self.state.altitude_cm = int(altitude_cm)
                                    self.state.battery_mv = battery_mv
                                    self.state.battery_voltage_v = battery_voltage_v
                                    self.state.battery_adc_raw = int(battery_adc_raw)
                                    self.state.battery_adc_channel = int(battery_adc_channel)
                                    self.state.battery_adc_ready = int(battery_adc_ready)
                                    self.state.rc_link_ok = bool(rc_link)
                                    self.state.rc_frames = int(rc_frames)
                                    self.state.channels = [int(v) for v in channels]
                                    self.state.motor_mode = motor_mode
                                    self.state.motors_armed = motors_armed
                                    self.state.gui_test_active = gui_test_active
                                    self.state.gyro_filter_available = bool(gyro_filter_available)
                                    self.state.motors_us = motors_us
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
    def __init__(self, root: tk.Tk, state: TelemetryState, lock: threading.Lock, reader: SerialReader):
        self.root = root
        self.state = state
        self.lock = lock
        self.reader = reader

        root.title("KAKUTEH7 Telemetry GUI")
        root.geometry("1180x760")
        root.minsize(980, 640)

        self._layout_path = os.path.join(os.path.dirname(__file__), ".read_serial_gui_layout.json")
        self._saved_layout: dict[str, object] = self._load_layout_config()
        self._pane_order: list[tuple[str, ttk.Panedwindow]] = []
        self._layout_restore_pending = True

        self.header_var = tk.StringVar(value="Waiting for data...")
        self.imu_var = tk.StringVar(value="")
        self.baro_var = tk.StringVar(value="")
        self.battery_var = tk.StringVar(value="BAT n/a")
        self.filter_target_var = tk.StringVar(value="Requested cutoff: 40 Hz")
        self.filter_fw_var = tk.StringVar(value="Firmware cutoff: 40 Hz")

        header = ttk.Frame(root, padding=10)
        header.pack(fill=tk.X)
        ttk.Label(header, textvariable=self.header_var, font=("Segoe UI", 13, "bold")).pack(anchor=tk.W)
        ttk.Label(header, textvariable=self.imu_var, font=("Consolas", 11)).pack(anchor=tk.W)
        env_row = ttk.Frame(header)
        env_row.pack(fill=tk.X)
        ttk.Label(env_row, textvariable=self.baro_var, font=("Consolas", 11)).pack(side=tk.LEFT, anchor=tk.W)
        ttk.Label(env_row, textvariable=self.battery_var, font=("Consolas", 11)).pack(side=tk.RIGHT, anchor=tk.E)

        body = ttk.Frame(root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)

        root_stack = ttk.Panedwindow(body, orient=tk.VERTICAL)
        root_stack.pack(fill=tk.BOTH, expand=True)
        self._pane_order.append(("root_stack", root_stack))

        top_row = ttk.Panedwindow(root_stack, orient=tk.HORIZONTAL)
        root_stack.add(top_row, weight=4)
        self._pane_order.append(("top_row", top_row))

        board3d_card = ttk.Frame(top_row)
        ttk.Label(board3d_card, text="Kakute 3D Body", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
        self.board3d_canvas = tk.Canvas(board3d_card, width=420, height=280, bg="#0b1220", highlightthickness=0)
        self.board3d_canvas.pack(fill=tk.BOTH, expand=True)
        top_row.add(board3d_card, weight=1)

        channel_card = ttk.Frame(top_row)
        ttk.Label(channel_card, text="Channels", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
        self.channel_canvas = tk.Canvas(channel_card, bg="#0f1319", highlightthickness=0)
        self.channel_canvas.pack(fill=tk.BOTH, expand=True)
        top_row.add(channel_card, weight=1)

        att_card = ttk.Frame(top_row)
        ttk.Label(att_card, text="Attitude Indicator", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
        self.att_canvas = tk.Canvas(att_card, width=420, height=280, bg="#0f1720", highlightthickness=0)
        self.att_canvas.pack(fill=tk.BOTH, expand=True)
        top_row.add(att_card, weight=1)

        filter_card = ttk.Frame(root_stack)
        ttk.Label(filter_card, text="Display Filter", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))

        filter_row = ttk.Frame(filter_card)
        filter_row.pack(fill=tk.X, padx=2, pady=(0, 6))
        ttk.Label(filter_row, textvariable=self.filter_target_var, font=("Consolas", 10, "bold")).pack(side=tk.LEFT)
        ttk.Label(filter_row, textvariable=self.filter_fw_var, font=("Consolas", 10), foreground="#94a3b8").pack(side=tk.RIGHT)

        self.filter_scale = ttk.Scale(
            filter_card,
            from_=5,
            to=150,
            orient=tk.HORIZONTAL,
            command=self._on_filter_cutoff_changed,
        )
        self.filter_scale.set(40)
        self.filter_scale.pack(fill=tk.X, padx=2, pady=(0, 6))

        self.gyro_canvas = tk.Canvas(filter_card, height=220, bg="#0f1720", highlightthickness=0)
        self.gyro_canvas.pack(fill=tk.BOTH, expand=True)
        root_stack.add(filter_card, weight=2)

        bottom_row = ttk.Panedwindow(root_stack, orient=tk.HORIZONTAL)
        root_stack.add(bottom_row, weight=3)
        self._pane_order.append(("bottom_row", bottom_row))

        stick_card = ttk.Frame(bottom_row)
        self.stick_canvas = tk.Canvas(stick_card, width=560, height=280, bg="#151a22", highlightthickness=0)
        self.stick_canvas.pack(fill=tk.BOTH, expand=True)
        bottom_row.add(stick_card, weight=2)

        controls_and_motors = ttk.Panedwindow(bottom_row, orient=tk.VERTICAL)
        bottom_row.add(controls_and_motors, weight=2)
        self._pane_order.append(("controls_and_motors", controls_and_motors))

        control_card = ttk.LabelFrame(controls_and_motors, text="Single Motor Test (USB)")

        self.test_motor_var = tk.IntVar(value=1)
        self.test_us_var = tk.IntVar(value=1100)
        self.test_running = False
        self.test_after_id: str | None = None

        row1 = ttk.Frame(control_card)
        row1.pack(fill=tk.X, padx=8, pady=(8, 4))
        ttk.Label(row1, text="Motor").pack(side=tk.LEFT)
        ttk.Combobox(
            row1,
            width=6,
            textvariable=self.test_motor_var,
            values=(1, 2, 3, 4),
            state="readonly",
        ).pack(side=tk.LEFT, padx=(8, 14))

        ttk.Label(row1, text="Power (us)").pack(side=tk.LEFT)
        self.test_us_scale = ttk.Scale(
            row1,
            from_=1000,
            to=2000,
            orient=tk.HORIZONTAL,
            command=self._on_test_us_changed,
        )
        self.test_us_scale.set(1100)
        self.test_us_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 8))
        self.test_us_label = ttk.Label(row1, width=8, text="1100 us")
        self.test_us_label.pack(side=tk.LEFT)

        row2 = ttk.Frame(control_card)
        row2.pack(fill=tk.X, padx=8, pady=(2, 8))
        self.test_button = ttk.Button(row2, text="Hold To Run")
        self.test_button.pack(side=tk.LEFT)
        self.test_button.bind("<ButtonPress-1>", self._on_test_press)
        self.test_button.bind("<ButtonRelease-1>", self._on_test_release)
        self.test_button.bind("<Leave>", self._on_test_release)
        ttk.Label(row2, text="Sends command every 80 ms with 250 ms firmware timeout.").pack(side=tk.LEFT, padx=(10, 0))
        controls_and_motors.add(control_card, weight=1)

        motor_card = ttk.Frame(controls_and_motors)
        ttk.Label(motor_card, text="Motors (M1-M4)", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
        self.motor_canvas = tk.Canvas(motor_card, width=520, height=220, bg="#10161d", highlightthickness=0)
        self.motor_canvas.pack(fill=tk.BOTH, expand=True)
        controls_and_motors.add(motor_card, weight=2)

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
        self._gyro_raw_history = deque(maxlen=240)
        self._gyro_filtered_history = deque(maxlen=240)
        self._filter_command_after_id: str | None = None
        self._last_filter_sent = 40
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

        self._apply_saved_layout_async()

    def _load_layout_config(self) -> dict[str, object]:
        try:
            with open(self._layout_path, "r", encoding="utf-8") as handle:
                loaded = json.load(handle)
            if isinstance(loaded, dict) and int(loaded.get("schema", 0)) == LAYOUT_SCHEMA_VERSION:
                return loaded
        except Exception:
            pass
        return {}

    def _save_layout_config(self) -> None:
        panes: dict[str, list[int]] = {}
        for name, pane in self._pane_order:
            try:
                pane_count = len(pane.panes())
                if pane_count < 2:
                    continue
                panes[name] = [int(pane.sashpos(i)) for i in range(pane_count - 1)]
            except Exception:
                continue

        payload = {
            "schema": LAYOUT_SCHEMA_VERSION,
            "geometry": self.root.winfo_geometry(),
            "panes": panes,
        }
        try:
            with open(self._layout_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
        except Exception:
            pass

    def _apply_saved_layout_async(self) -> None:
        geometry = self._saved_layout.get("geometry")
        if isinstance(geometry, str) and geometry:
            try:
                self.root.geometry(geometry)
            except Exception:
                pass
        self.root.after(120, self._apply_saved_panes)

    def _apply_saved_panes(self) -> None:
        self.root.update_idletasks()
        pane_data = self._saved_layout.get("panes")
        if not isinstance(pane_data, dict):
            self._layout_restore_pending = False
            return

        for name, pane in self._pane_order:
            saved_positions = pane_data.get(name)
            if not isinstance(saved_positions, list):
                continue

            pane_orient = str(pane.cget("orient"))
            pane_span = pane.winfo_width() if pane_orient == "horizontal" else pane.winfo_height()
            if pane_span <= 0:
                continue

            edge_margin = 120
            for idx, value in enumerate(saved_positions):
                try:
                    requested = int(value)
                    clamped = max(edge_margin, min(pane_span - edge_margin, requested))
                    pane.sashpos(idx, clamped)
                except Exception:
                    continue

        self._layout_restore_pending = False

    def on_close(self) -> None:
        self._save_layout_config()

    def _queue_motor_test(self, motor_index: int, motor_us: int) -> None:
        self.reader.queue_command(_build_motor_test_packet(motor_index, motor_us))
        self.reader.queue_command(_build_motor_test_ascii(motor_index, motor_us))

    def _queue_display_filter(self, cutoff_hz: int) -> None:
        self.reader.queue_command(_build_display_filter_packet(cutoff_hz))
        self.reader.queue_command(_build_display_filter_ascii(cutoff_hz))

    def _send_display_filter_now(self, cutoff_hz: int) -> None:
        cutoff = max(5, min(150, int(cutoff_hz)))
        self._last_filter_sent = cutoff
        self.filter_target_var.set(f"Requested cutoff: {cutoff} Hz")
        self._queue_display_filter(cutoff)

    def _on_filter_cutoff_changed(self, value: str) -> None:
        cutoff = int(float(value))
        self.filter_target_var.set(f"Requested cutoff: {cutoff} Hz")
        if self._filter_command_after_id is not None:
            self.root.after_cancel(self._filter_command_after_id)
        self._filter_command_after_id = self.root.after(35, lambda cutoff=cutoff: self._send_display_filter_now(cutoff))

    def _on_test_us_changed(self, value: str) -> None:
        us = int(float(value))
        self.test_us_var.set(us)
        self.test_us_label.configure(text=f"{us:4d} us")
        if self.test_running:
            self._queue_motor_test(self.test_motor_var.get(), us)

    def _test_tick(self) -> None:
        if not self.test_running:
            self.test_after_id = None
            return
        self._queue_motor_test(self.test_motor_var.get(), self.test_us_var.get())
        self.test_after_id = self.root.after(80, self._test_tick)

    def _on_test_press(self, _event=None) -> None:
        if self.test_running:
            return
        self.test_running = True
        self._queue_motor_test(self.test_motor_var.get(), self.test_us_var.get())
        self._test_tick()

    def _on_test_release(self, _event=None) -> None:
        if not self.test_running:
            return
        self.test_running = False
        if self.test_after_id is not None:
            self.root.after_cancel(self.test_after_id)
            self.test_after_id = None
        self._queue_motor_test(self.test_motor_var.get(), 1000)

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

    def _draw_motors(self, mode: int, armed: int, motors_us: list[int]) -> None:
        c = self.motor_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()

        cx = w / 2.0
        cy = h / 2.0 + 12.0
        dx = 120.0
        dy = 65.0
        r = 30.0

        mode_text = "REAL THROTTLE" if mode == 1 else "SIMULATION"
        mode_color = "#ef4444" if mode == 1 else "#f59e0b"
        arm_text = "ARMED" if armed else "DISARMED"
        arm_color = "#22c55e" if armed else "#94a3b8"

        c.create_text(10, 10, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text=f"MODE: {mode_text}")
        c.create_text(220, 10, anchor="nw", fill=arm_color, font=("Consolas", 11, "bold"), text=f"STATE: {arm_text}")
        c.create_text(cx, 14, anchor="n", fill="#fbbf24", font=("Consolas", 11, "bold"), text="FRONT")
        c.create_line(cx, 30, cx, 52, fill="#fbbf24", width=3, arrow=tk.FIRST)
        c.create_line(cx - 70, cy - 38, cx + 70, cy + 38, fill="#334155", width=2)
        c.create_line(cx + 70, cy - 38, cx - 70, cy + 38, fill="#334155", width=2)

        # INAV convention: M1 aft-right, M2 forward-right, M3 aft-left, M4 forward-left.
        positions = [
            (cx + dx, cy + dy),
            (cx + dx, cy - dy),
            (cx - dx, cy + dy),
            (cx - dx, cy - dy),
        ]

        role_labels = ["AR", "FR", "AL", "FL"]

        for idx, (mx, my) in enumerate(positions):
            us = int(motors_us[idx]) if idx < len(motors_us) else 1000
            norm = max(0.0, min(1.0, (us - 1000) / 1000.0))
            fill = "#1f2937" if us <= 1000 else mode_color
            c.create_oval(mx - r, my - r, mx + r, my + r, fill=fill, outline="#64748b", width=2)
            c.create_arc(mx - r, my - r, mx + r, my + r, start=90, extent=-360.0 * norm, style=tk.ARC, outline="#22c55e", width=4)
            c.create_text(mx, my - 8, fill="#e2e8f0", font=("Consolas", 10, "bold"), text=f"M{idx + 1}")
            c.create_text(mx, my + 4, fill="#93c5fd", font=("Consolas", 9, "bold"), text=role_labels[idx])
            c.create_text(mx, my + 16, fill="#cbd5e1", font=("Consolas", 9), text=f"{us:4d} us")

    def _append_gyro_history(self, snapshot: TelemetryState) -> None:
        raw_scale = IMU_GYRO_DPS_PER_LSB
        raw_sample = (snapshot.gx * raw_scale, snapshot.gy * raw_scale, snapshot.gz * raw_scale)
        if snapshot.gyro_filter_available:
            filtered_sample = (
                snapshot.gx_filtered * raw_scale,
                snapshot.gy_filtered * raw_scale,
                snapshot.gz_filtered * raw_scale,
            )
        else:
            filtered_sample = raw_sample

        self._gyro_raw_history.append(raw_sample)
        self._gyro_filtered_history.append(filtered_sample)

    def _draw_gyro_graph(self, snapshot: TelemetryState) -> None:
        c = self.gyro_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        split = h // 2

        c.create_rectangle(0, 0, w, h, fill="#0f1720", outline="")
        c.create_text(10, 8, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text=f"Gyro Display Filter  {snapshot.display_filter_cutoff_hz} Hz")

        def draw_plot(y0: int, y1: int, history: deque[tuple[float, float, float]], title: str) -> None:
            pad_x = 18
            top = y0 + 22
            bottom = y1 - 14
            left = pad_x
            right = w - pad_x
            center_y = (top + bottom) / 2.0
            if len(history) > 0:
                max_abs_dps = max(abs(v) for sample in history for v in sample)
            else:
                max_abs_dps = 0.0

            amp_dps = max(20.0, min(2000.0, max_abs_dps * 1.2))
            pixels_per_dps = ((bottom - top) * 0.5) / amp_dps
            c.create_text(left, y0 + 4, anchor="nw", fill="#cbd5e1", font=("Consolas", 10, "bold"), text=title)
            c.create_line(left, center_y, right, center_y, fill="#475569")
            c.create_line(left, top, right, top, fill="#1f2937")
            c.create_line(left, bottom, right, bottom, fill="#1f2937")
            c.create_text(left, top + 2, anchor="nw", fill="#94a3b8", font=("Consolas", 9), text=f"+/-{amp_dps:5.1f} dps")

            if len(history) < 2:
                return

            denom = max(1, len(history) - 1)
            axis_colors = ("#f87171", "#4ade80", "#60a5fa")
            for axis in range(3):
                points: list[float] = []
                for idx, sample in enumerate(history):
                    value = sample[axis]
                    x_pos = left + (right - left) * (idx / denom)
                    clamped = max(-amp_dps, min(amp_dps, value))
                    y_pos = center_y - clamped * pixels_per_dps
                    points.extend((x_pos, y_pos))
                c.create_line(*points, fill=axis_colors[axis], width=2, smooth=True)

            legend_x = right - 172
            legend_y = y0 + 4
            legend = (("X", "#f87171"), ("Y", "#4ade80"), ("Z", "#60a5fa"))
            for idx, (label, color) in enumerate(legend):
                x = legend_x + idx * 56
                c.create_line(x, legend_y + 8, x + 16, legend_y + 8, fill=color, width=3)
                c.create_text(x + 20, legend_y + 8, anchor="w", fill="#cbd5e1", font=("Consolas", 9), text=label)

        draw_plot(0, split, self._gyro_raw_history, "Raw Gyro (dps)")
        draw_plot(split, h, self._gyro_filtered_history, "Filtered Gyro (dps)")

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
            x_view = -y
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

        gx_source = snapshot.gx_filtered if snapshot.gyro_filter_available else snapshot.gx
        gy_source = snapshot.gy_filtered if snapshot.gyro_filter_available else snapshot.gy
        gz_source = snapshot.gz_filtered if snapshot.gyro_filter_available else snapshot.gz

        gx_dps = gx_source * IMU_GYRO_DPS_PER_LSB
        gy_dps = gy_source * IMU_GYRO_DPS_PER_LSB
        gz_dps = gz_source * IMU_GYRO_DPS_PER_LSB
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

        gx_source = snapshot.gx_filtered if snapshot.gyro_filter_available else snapshot.gx
        gy_source = snapshot.gy_filtered if snapshot.gyro_filter_available else snapshot.gy
        gz_source = snapshot.gz_filtered if snapshot.gyro_filter_available else snapshot.gz

        gx_dps = gx_source * IMU_GYRO_DPS_PER_LSB
        gy_dps = gy_source * IMU_GYRO_DPS_PER_LSB
        gz_dps = gz_source * IMU_GYRO_DPS_PER_LSB
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
                gx_filtered=self.state.gx_filtered,
                gy_filtered=self.state.gy_filtered,
                gz_filtered=self.state.gz_filtered,
                ax=self.state.ax,
                ay=self.state.ay,
                az=self.state.az,
                display_filter_cutoff_hz=self.state.display_filter_cutoff_hz,
                pressure_pa=self.state.pressure_pa,
                altitude_cm=self.state.altitude_cm,
                battery_mv=self.state.battery_mv,
                battery_voltage_v=self.state.battery_voltage_v,
                battery_adc_raw=self.state.battery_adc_raw,
                battery_adc_channel=self.state.battery_adc_channel,
                battery_adc_ready=self.state.battery_adc_ready,
                rc_link_ok=self.state.rc_link_ok,
                rc_frames=self.state.rc_frames,
                channels=list(self.state.channels),
                motor_mode=self.state.motor_mode,
                motors_armed=self.state.motors_armed,
                gui_test_active=self.state.gui_test_active,
                gyro_filter_available=self.state.gyro_filter_available,
                motors_us=list(self.state.motors_us),
                last_update_monotonic=self.state.last_update_monotonic,
                connected=self.state.connected,
                port_name=self.state.port_name,
                status_text=self.state.status_text,
            )

        age_ms = (time.monotonic() - snapshot.last_update_monotonic) * 1000.0 if snapshot.last_update_monotonic else -1.0
        link = "OK" if snapshot.rc_link_ok else "--"
        ser = "UP" if snapshot.connected else "DN"
        port = snapshot.port_name if snapshot.port_name else "n/a"
        motor_mode_text = "REAL" if snapshot.motor_mode == 1 else "SIM"
        motor_state_text = "ARM" if snapshot.motors_armed else "DIS"
        gui_test_text = "GUI_TEST" if snapshot.gui_test_active else "-"
        self.header_var.set(
            f"SEQ {snapshot.sequence:06d}   RC {link}   RF {snapshot.rc_frames:6d}   "
            f"MOT {motor_mode_text}/{motor_state_text}/{gui_test_text}   SER {ser} ({port})   Age {age_ms:6.1f} ms"
        )
        self.imu_var.set(
            f"GX {snapshot.gx:6d}  GY {snapshot.gy:6d}  GZ {snapshot.gz:6d}    "
            f"AX {snapshot.ax:6d}  AY {snapshot.ay:6d}  AZ {snapshot.az:6d}"
        )
        self.baro_var.set(
            f"BARO P {snapshot.pressure_pa:7d} Pa   ALT {snapshot.altitude_cm / 100.0:+7.2f} m"
        )
        if snapshot.battery_voltage_v is None:
            self.battery_var.set("BAT n/a (telemetry has no battery field/value yet)")
        elif (snapshot.battery_mv is not None) and (snapshot.battery_mv == 0):
            self.battery_var.set("BAT 0.00 V")
        else:
            self.battery_var.set(f"BAT {snapshot.battery_voltage_v:0.2f} V")
        self.filter_fw_var.set(f"Firmware cutoff: {snapshot.display_filter_cutoff_hz} Hz")
        fusion_label = "FusionGUI:ON" if self._fusion_ready else "FusionGUI:OFF"
        if (not self._fusion_ready) and imufusion_error:
            fusion_label = f"{fusion_label} ({imufusion_error})"
        self.footer_var.set(
            f"{snapshot.status_text} | {fusion_label} | Motor us: "
            f"M1 {snapshot.motors_us[0]:4d} M2 {snapshot.motors_us[1]:4d} "
            f"M3 {snapshot.motors_us[2]:4d} M4 {snapshot.motors_us[3]:4d} "
            f"| GUI test: {'ON' if snapshot.gui_test_active else 'OFF'}"
        )

        # Keep startup attitude aligned with firmware's calibrated level estimate.
        if snapshot.gyro_filter_available:
            fusion_quaternion = None
            source_pitch, source_roll, source_yaw = snapshot.pitch_deg, snapshot.roll_deg, snapshot.yaw_deg
        else:
            fusion_quaternion = self._compute_fusion_quaternion(snapshot)
            if fusion_quaternion is not None:
                source_pitch, source_roll, source_yaw = self._quaternion_to_euler_display(fusion_quaternion)
            else:
                source_pitch, source_roll, source_yaw = snapshot.pitch_deg, snapshot.roll_deg, snapshot.yaw_deg

        self._append_gyro_history(snapshot)
        self._draw_attitude(source_pitch, source_roll, source_yaw)
        self._draw_sticks(snapshot.channels)
        self._draw_board_3d(source_pitch, source_roll, source_yaw, fusion_quaternion)
        self._draw_gyro_graph(snapshot)
        self._draw_channels(snapshot.channels)
        self._draw_motors(snapshot.motor_mode, snapshot.motors_armed, snapshot.motors_us)

        self.root.after(33, self.update)


def main() -> int:
    parser = argparse.ArgumentParser(description="Graphical telemetry viewer for serial telemetry")
    parser.add_argument("--port", default="COM6", help="Serial port, for example COM6 or auto")
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
    app = TelemetryApp(root, state, lock, reader)

    def _on_close() -> None:
        app.on_close()
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
