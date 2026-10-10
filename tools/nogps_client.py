#!/usr/bin/env python3
"""Проверка блока NoGPS с ноутбука, подключённого к его Wi-Fi (ТЗ, раздел 13).

Шлёт HELLO раз в секунду и печатает строки блока. Команды вводятся с клавиатуры:
  CAL_UP, CAL_BIAS, CAL_RESET, STATUS, INFO, EVENTS,1, IMU_TEST, SET,data_hz,20, REBOOT,
  BT_PAIR (на 2 минуты разрешить привязку нового телефона по Bluetooth), BT_FORGET
По умолчанию NGD печатается раз в секунду, ключ --all печатает все.
То же по Bluetooth — tools/nogps_ble.py.

    python tools/nogps_client.py

Обновление прошивки по Wi-Fi работает только на плате для стола (SET,bench,1 и REBOOT):

    python tools/nogps_client.py --ota build/ELMandIMU.bin
"""
import argparse
import hashlib
import os
import socket
import struct
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
    try:
        if int(raw[star + 1:], 16) != crc16(body):
            return None
    except ValueError:
        return None
    return body.split(",")


FLAGS = ["IMU_OK", "BIAS_OK", "UP_OK", "FWD_OK", "OBD_OK", "STILL", "CALIBRATING",
         "MOUNT_MOVED", "REVERSE", "OBD_ABSENT", "ERROR"]


def ota(sock, box, send, path):
    """Отправить прошивку: OTA_BEGIN, куски '#'+смещение+данные в пределах окна, OTA_END."""
    data = open(path, "rb").read()
    sha = hashlib.sha256(data).hexdigest()
    name = os.path.basename(path).replace(",", "_").replace("*", "_")
    print(f"обновление: {path}, {len(data)} байт, SHA-256 {sha[:16]}…")

    def wait_reply(seq, timeout):
        """Ждёт NGA с этим номером; попутно запоминает последнюю NGO."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                f = parse(sock.recvfrom(1024)[0].decode("ascii", "replace"))
            except socket.timeout:
                continue
            if f and f[0] == "NGO":
                state["ngo"] = (int(f[2]), f[3])
            elif f and f[0] == "NGA" and int(f[1]) == seq and f[2] != "PROGRESS":
                return f[2:]
            elif f and f[0] == "NGE":
                print("  ", ",".join(f))
        return ["ERR", "нет ответа"]

    state = {"ngo": (0, "RECV")}
    r = wait_reply(send(f"OTA_BEGIN,{len(data)},{sha},{name}"), 5)
    if r[0] != "OK":
        print("блок отказал:", ",".join(r))
        return
    chunk, window = int(r[1]), int(r[2])
    print(f"кусок {chunk} байт, окно {window} байт")
    sock.settimeout(0.005)
    sent, rewound, t0, shown = 0, -1, time.time(), 0.0
    last_acked, progress_at = 0, time.time()
    while True:
        acked, st = state["ngo"]
        now = time.time()
        if acked != last_acked:
            last_acked, progress_at = acked, now
        if st not in ("RECV", "RESEND") or acked >= len(data):
            break
        if st == "RESEND" and acked != rewound:
            # Блок увидел разрыв: вернуться к месту, с которого он ждёт. Одну просьбу — один раз.
            sent = rewound = acked
        elif now - progress_at > 1 and sent > acked:
            # Подтверждённое не растёт уже секунду: последние куски потерялись, повторить.
            sent, progress_at = acked, now
        if now - shown > 1:
            shown = now
            print(f"  {acked * 100 // len(data)} %  {acked}/{len(data)}  "
                  f"{acked / 1024 / max(now - t0, 0.001):.1f} КБ/с")
        # Не уходить дальше чем на окно от подтверждённого.
        if sent < len(data) and sent - acked + chunk <= window:
            part = data[sent:sent + chunk]
            sock.sendto(b"#" + struct.pack("<I", sent) + part, box)
            sent += len(part)
            continue
        try:
            f = parse(sock.recvfrom(1024)[0].decode("ascii", "replace"))
            if f and f[0] == "NGO":
                state["ngo"] = (int(f[2]), f[3])
        except socket.timeout:
            pass
    sock.settimeout(0.5)
    if state["ngo"][1] not in ("RECV", "RESEND"):
        print("блок прервал обновление:", state["ngo"][1])
        return
    print(f"передано за {time.time() - t0:.1f} с, блок проверяет образ…")
    r = wait_reply(send("OTA_END"), 60)
    print("итог:", ",".join(r))
    if r[0] == "OK":
        print("блок перезагрузится в новую прошивку. Запустите скрипт снова в течение 10 минут: "
              "прошивка подтверждается, когда к ней подключились, иначе блок вернёт прежнюю.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--all", action="store_true", help="печатать каждую NGD")
    ap.add_argument("--ota", metavar="ФАЙЛ", help="отправить блоку прошивку (.bin) и выйти; "
                    "по Wi-Fi работает только на плате для стола (SET,bench,1)")
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
            return seq[0]

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
    if args.ota:
        time.sleep(0.3)
        ota(sock, box, send, args.ota)
        return
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
