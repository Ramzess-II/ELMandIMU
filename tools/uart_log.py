#!/usr/bin/env python3
"""Пишет лог платы с COM-порта в файл, каждая строка — с временем компьютера.

Плату не сбрасывает: линии DTR и RTS не трогаются. Если плата перезагрузилась и порт пропал,
открывает его заново. Пока скрипт работает, порт занят: прошивать плату в это время нельзя.

    python tools/uart_log.py COM15 uart.log --minutes 30
"""
import argparse
import datetime
import time

import serial


def open_port(name):
    ser = serial.Serial()
    ser.port = name
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("file")
    ap.add_argument("--minutes", type=float, default=30)
    args = ap.parse_args()

    end = time.time() + args.minutes * 60
    ser, buf = None, b""
    with open(args.file, "a", encoding="utf-8", newline="\n") as out:
        def put(text):
            stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            out.write(f"{stamp} {text}\n")
            out.flush()

        put(f"--- запись начата, порт {args.port} ---")
        while time.time() < end:
            try:
                if ser is None:
                    ser = open_port(args.port)
                buf += ser.read(4096)
            except Exception as e:
                if ser is not None:
                    put(f"--- порт пропал: {e} ---")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                time.sleep(0.5)
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                put(raw.decode("utf-8", "replace").rstrip("\r"))
        put("--- запись закончена ---")


if __name__ == "__main__":
    main()
