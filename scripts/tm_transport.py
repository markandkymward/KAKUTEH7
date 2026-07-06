from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Optional

import serial


@dataclass
class TransportStats:
    rx_bytes: int = 0
    tx_bytes: int = 0
    rx_errors: int = 0
    tx_errors: int = 0


class BinaryTransport:
    def __init__(self) -> None:
        self._rx_queue: Queue[bytes] = Queue(maxsize=1024)
        self._tx_queue: Queue[bytes] = Queue(maxsize=1024)
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._ser: Optional[serial.Serial] = None
        self.stats = TransportStats()

    def connect_tcp(self, host: str, port: int, timeout_s: float = 2.0) -> None:
        self.close()
        sock = socket.create_connection((host, port), timeout=timeout_s)
        sock.setblocking(False)
        self._sock = sock
        self._start_threads()

    def connect_serial(self, port: str, baud: int = 921600, timeout_s: float = 0.05) -> None:
        self.close()
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
        self._start_threads()

    def _start_threads(self) -> None:
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_worker, daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def send(self, data: bytes) -> bool:
        try:
            self._tx_queue.put_nowait(data)
            return True
        except Exception:
            self.stats.tx_errors += 1
            return False

    def recv_nonblocking(self) -> list[bytes]:
        out: list[bytes] = []
        while True:
            try:
                out.append(self._rx_queue.get_nowait())
            except Empty:
                return out

    def _rx_worker(self) -> None:
        while not self._stop.is_set():
            try:
                if self._sock is not None:
                    try:
                        data = self._sock.recv(4096)
                    except BlockingIOError:
                        time.sleep(0.002)
                        continue
                    if not data:
                        time.sleep(0.01)
                        continue
                elif self._ser is not None:
                    data = self._ser.read(self._ser.in_waiting or 1)
                    if not data:
                        continue
                else:
                    return

                self.stats.rx_bytes += len(data)
                try:
                    self._rx_queue.put_nowait(data)
                except Exception:
                    self.stats.rx_errors += 1
            except Exception:
                self.stats.rx_errors += 1
                time.sleep(0.01)

    def _tx_worker(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._tx_queue.get(timeout=0.05)
            except Empty:
                continue

            try:
                if self._sock is not None:
                    self._sock.sendall(data)
                elif self._ser is not None:
                    self._ser.write(data)
                else:
                    continue
                self.stats.tx_bytes += len(data)
            except Exception:
                self.stats.tx_errors += 1
