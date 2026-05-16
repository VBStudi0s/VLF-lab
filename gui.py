#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial
from serial.tools import list_ports

PACKET_SYNC = b"\xA5\x5A"
PACKET_END_SYNC = b"\xAB\xBA"

# Структура: sync(2) + block_id(4) + dropped(4) + mean_raw(2) + min(2) + max(2) + sample_count(2) + end_sync(2) = 20
HEADER_FMT = "<I I H h h H"   # 4+4+2+2+2+2 = 16 байт после sync
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 16
PACKET_SIZE = 2 + HEADER_SIZE + 2          # 2+16+2=20

@dataclass
class Packet:
    block_id: int
    dropped_triggers: int
    mean_raw: int
    min_centered: int
    max_centered: int
    sample_count: int

def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("Нет доступных serial-портов.")
        return
    for i, p in enumerate(ports, 1):
        print(f"{i}. {p.device} — {p.description}")

def pick_port(port_arg: Optional[str]) -> str:
    if port_arg and port_arg.lower() != "auto":
        return port_arg
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("Не найдено ни одного serial-порта")
    if len(ports) == 1:
        return ports[0].device
    print("Выбери порт:")
    for i, p in enumerate(ports, 1):
        print(f"{i}. {p.device} — {p.description}")
    while True:
        try:
            idx = int(input("Номер: ").strip())
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device
        except ValueError:
            pass
        print("Неверный выбор.")

def adc_counts_to_volts(counts: int, full_scale_v: float = 1.2, adc_max: int = 4095) -> float:
    return (counts / adc_max) * full_scale_v

def parse_one_packet(buf: bytearray) -> Optional[Packet]:
    pos = buf.find(PACKET_SYNC)
    if pos < 0:
        if len(buf) > 1:
            del buf[:-1]
        return None
    if pos > 0:
        del buf[:pos]

    if len(buf) < PACKET_SIZE:
        return None

    # Распаковка заголовка (включая sample_count)
    header = struct.unpack_from(HEADER_FMT, buf, 2)
    block_id, dropped_triggers, mean_raw, min_centered, max_centered, sample_count = header

    # Проверка конца пакета
    if buf[PACKET_SIZE - 2:PACKET_SIZE] != PACKET_END_SYNC:
        del buf[:1]
        return None

    del buf[:PACKET_SIZE]
    return Packet(
        block_id=block_id,
        dropped_triggers=dropped_triggers,
        mean_raw=mean_raw,
        min_centered=min_centered,
        max_centered=max_centered,
        sample_count=sample_count,
    )

def run_receiver(port: str, baud: int, timeout: float) -> None:
    print(f"Открываю {port} @ {baud} ...")
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    ser.reset_input_buffer()
    buf = bytearray()
    frames = 0
    last_info = time.time()
    try:
        while True:
            waiting = ser.in_waiting
            chunk = ser.read(waiting if waiting > 0 else 1)
            if chunk:
                buf.extend(chunk)
            while True:
                pkt = parse_one_packet(buf)
                if pkt is None:
                    break
                frames += 1
                mean_v = adc_counts_to_volts(pkt.mean_raw)
                print(
                    f"#{pkt.block_id:08d} | "
                    f"mean={pkt.mean_raw:4d} ({mean_v:.3f} V) | "
                    f"min={pkt.min_centered:5d} | "
                    f"max={pkt.max_centered:5d} | "
                    f"dropped={pkt.dropped_triggers} | "
                    f"samples={pkt.sample_count}"
                )
            if time.time() - last_info >= 5.0:
                print(f"[info] frames={frames}, buffer={len(buf)} bytes")
                last_info = time.time()
    except KeyboardInterrupt:
        print("\nОстановка.")
    finally:
        ser.close()

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    args = parser.parse_args()
    if args.list:
        list_serial_ports()
        return
    port = pick_port(args.port)
    run_receiver(port, args.baud, args.timeout)

if __name__ == "__main__":
    main()
