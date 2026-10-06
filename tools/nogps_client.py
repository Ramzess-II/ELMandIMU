#!/usr/bin/env python3
"""Проверка блока NoGPS с ноутбука, подключённого к его Wi-Fi (ТЗ, раздел 13).

Шлёт HELLO раз в секунду и печатает строки блока. Команды вводятся с клавиатуры:
  CAL_UP, CAL_BIAS, CAL_RESET, STATUS, EVENTS,1, IMU_TEST, SET,data_hz,20, REBOOT
По умолчанию NGD печатается раз в секунду, ключ --all печатает все.

    python tools/nogps_client.py
"""
import argparse
import socket
import sys
import threading
import time


def crc16(text: str) -> int:
    crc = 0xFFFF
    for b in text.encode("ascii"):
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def line(body: str) -> bytes:
    return f"${body}*{crc16(body):04X}\r\n".encode("ascii")


def parse(raw: str):
    raw = raw.strip()
    star = raw.rfind("*")
    if not raw.startswith("$") or star < 0 or len(raw) != star + 5:
        return None
    body = raw[1:star]
    if int(raw[star + 1:], 16) != crc16(body):
        return None
    return body.split(",")


FLAGS = ["IMU_OK", "BIAS_OK", "UP_OK", "FWD_OK", "OBD_OK", "STILL", "CALIBRATING",
         "MOUNT_MOVED", "REVERSE", "OBD_ABSENT", "ERROR"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--all", action="store_true", help="печатать каждую NGD")
    args = ap.parse_args()

    box = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    seq = [0]
    lock = threading.Lock()

    def send(cmd):
        with lock:
            seq[0] += 1
            sock.sendto(line(f"NGC,{seq[0]},{cmd}"), box)

    def hello():
        while True:
            send("HELLO")
            time.sleep(1)

    def keyboard():
        for cmd in sys.stdin:
            cmd = cmd.strip()
            if cmd:
                send(cmd)

    threading.Thread(target=hello, daemon=True).start()
    threading.Thread(target=keyboard, daemon=True).start()

    n_ngd, bad, t0, last_print = 0, 0, time.time(), 0.0
    while True:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            print("... нет данных")
            continue
        text = data.decode("ascii", "replace")
        f = parse(text)
        if f is None:
            bad += 1
            print("ПЛОХАЯ CRC:", text.strip())
            continue
        if f[0] == "NGD":
            n_ngd += 1
            now = time.time()
            if args.all or now - last_print >= 1:
                flags = int(f[9], 16)
                names = [n for i, n in enumerate(FLAGS) if flags >> i & 1]
                rate = n_ngd / (now - t0) if now > t0 else 0
                acc = ""
                if len(f) >= 14 and f[10] != "":
                    acc = (f" ускорение h1={int(f[10]) / 1000:+.2f} h2={int(f[11]) / 1000:+.2f} "
                           f"up={int(f[12]) / 1000:+.2f} встряска={int(f[13]) / 1000:.2f} м/с²")
                print(f"NGD #{f[2]} t={f[3]} курс={int(f[4]) / 1000:+.2f}° "
                      f"поворот={int(f[5]) / 1000:+.2f}°/с скорость={f[7]} "
                      f"[{' '.join(names)}] {rate:.1f}/с{acc}")
                last_print = now
        else:
            print(text.strip())


if __name__ == "__main__":
    main()
