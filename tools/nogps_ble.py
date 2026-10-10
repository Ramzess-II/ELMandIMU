#!/usr/bin/env python3
"""Проверка блока NoGPS по Bluetooth LE с ноутбука — то же, что nogps_client.py, но без Wi-Fi.

Ищет блок по UUID сервиса, подключается, шлёт HELLO раз в секунду и печатает строки блока.
Команды вводятся с клавиатуры: STATUS, INFO, CAL_UP, EVENTS,1, BT_PAIR, BT_FORGET, SET,bt_pin,654321 …
По умолчанию NGD печатается раз в секунду, ключ --all печатает все.

    pip install bleak
    python tools/nogps_ble.py
    python tools/nogps_ble.py --name NoGPS-1B00

Обновление прошивки (файл .bin должен быть подписан тем же ключом, что прошивка в блоке):

    python tools/nogps_ble.py --ota build/ELMandIMU.bin

Перед первым подключением ноутбук надо привязать к блоку средствами системы (Windows: Параметры →
Bluetooth → Добавить устройство; Linux: bluetoothctl pair). Система спросит код из шести цифр — это
настройка bt_pin, по умолчанию 123456. Блок принимает новую привязку только в «окне сопряжения»:
первые 2 минуты после подачи питания или после команды BT_PAIR, посланной по Wi-Fi либо с уже
привязанного телефона.
"""
import argparse
import asyncio
import hashlib
import os
import struct
import sys
import threading
import time

from bleak import BleakClient, BleakScanner

from nogps_client import FLAGS, line, parse

SVC = "02cb0001-c0c3-40d0-819c-5a85998dcd99"
TX = "02cb0002-c0c3-40d0-819c-5a85998dcd99"
RX = "02cb0003-c0c3-40d0-819c-5a85998dcd99"
OTA = "02cb0004-c0c3-40d0-819c-5a85998dcd99"


async def find(name):
    print("ищу блок…")
    found = await BleakScanner.discover(timeout=6, return_adv=True)
    for dev, adv in found.values():
        if SVC in [u.lower() for u in adv.service_uuids] and (not name or adv.local_name == name):
            # Данные производителя 0xFFFF: версия формата и флаги, бит 0 — окно сопряжения открыто.
            mfg = adv.manufacturer_data.get(0xFFFF, b"")
            window = "открыто" if len(mfg) >= 2 and mfg[1] & 1 else "закрыто"
            print(f"нашёл {adv.local_name or '?'} {dev.address}, уровень {adv.rssi} дБм, "
                  f"окно сопряжения {window}")
            return dev
    return None


class Link:
    """Соединение: HELLO раз в секунду, сборка строк из потока TX, ответы на команды."""

    def __init__(self, client, show_all):
        self.client = client
        self.show_all = show_all
        self.seq = 0
        self.buf = b""
        self.ngd = 0
        self.t0 = time.time()
        self.last = 0.0
        self.replies = {}       # номер команды -> поля ответа NGA
        self.ngo = None         # последняя строка NGO: (принято, состояние)
        self.quiet = False      # во время обновления NGS не печатаем

    # TX — поток байт: строка может прийти в нескольких уведомлениях. Собираем от $ до \n.
    def on_tx(self, _, data):
        self.buf += bytes(data)
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            start = raw.rfind(b"$")
            if start >= 0:
                self.show(raw[start:].decode("ascii", "replace"))

    def show(self, text):
        f = parse(text)
        if f is None:
            print("ПЛОХАЯ CRC:", text.strip())
            return
        if f[0] == "NGA" and len(f) >= 3 and f[2] != "PROGRESS":
            self.replies[int(f[1])] = f[2:]
        if f[0] == "NGO":
            self.ngo = (int(f[2]), f[3])
            return
        if f[0] != "NGD":
            if not (self.quiet and f[0] == "NGS"):
                print(text.strip())
            return
        self.ngd += 1
        now = time.time()
        if not self.show_all and now - self.last < 1:
            return
        self.last = now
        flags = int(f[9], 16)
        names = [n for i, n in enumerate(FLAGS) if flags >> i & 1]
        rate = self.ngd / (now - self.t0) if now > self.t0 else 0
        print(f"NGD #{f[2]} t={f[3]} курс={int(f[4]) / 1000:+.2f}° поворот={int(f[5]) / 1000:+.2f}°/с "
              f"скорость={f[7]} [{' '.join(names)}] {rate:.1f}/с")

    async def send(self, cmd, response=True):
        self.seq += 1
        await self.client.write_gatt_char(RX, line(f"NGC,{self.seq},{cmd}"), response=response)
        return self.seq

    async def command(self, cmd, timeout=10):
        """Команда и её окончательный ответ NGA (список полей после номера)."""
        seq = await self.send(cmd)
        end = time.time() + timeout
        while seq not in self.replies:
            if time.time() > end:
                raise TimeoutError(f"нет ответа на {cmd}")
            await asyncio.sleep(0.02)
        return self.replies.pop(seq)

    async def hello_loop(self):
        while self.client.is_connected:
            # Первая запись — с ответом: если связь ещё не зашифрована, блок откажет, и система сама
            # начнёт шифрование или сопряжение. Запись без ответа он молча отбросил бы.
            await self.send("HELLO", response=self.seq == 0)
            await asyncio.sleep(1)


async def ota(link, path):
    data = open(path, "rb").read()
    sha = hashlib.sha256(data).hexdigest()
    print(f"обновление: {path}, {len(data)} байт, SHA-256 {sha[:16]}…")
    name = os.path.basename(path).replace(",", "_").replace("*", "_")
    r = await link.command(f"OTA_BEGIN,{len(data)},{sha},{name}")
    if r[0] != "OK":
        print("блок отказал:", ",".join(r))
        return
    chunk, window = int(r[1]), int(r[2])
    print(f"кусок {chunk} байт, окно {window} байт")
    link.quiet = True
    link.ngo = (0, "RECV")
    sent = 0            # откуда слать следующий кусок
    rewound = -1        # куда уже возвращались по просьбе блока
    t0 = time.time()
    shown = 0.0
    last_acked, progress_at = 0, time.time()
    while True:
        acked, state = link.ngo
        now = time.time()
        if acked != last_acked:
            last_acked, progress_at = acked, now
        if state not in ("RECV", "RESEND") or acked >= len(data):
            break
        if state == "RESEND" and acked != rewound:
            # Блок увидел разрыв: вернуться к месту, с которого он ждёт. Одну просьбу — один раз.
            sent = rewound = acked
        elif now - progress_at > 2 and sent > acked:
            # Подтверждённое не растёт уже две секунды: повторить с подтверждённого места.
            sent, progress_at = acked, now
        if now - shown > 1:
            shown = now
            speed = acked / 1024 / max(now - t0, 0.001)
            print(f"  {acked * 100 // len(data)} %  {acked}/{len(data)}  {speed:.1f} КБ/с")
        # Не уходить дальше чем на окно от подтверждённого.
        if sent < len(data) and sent - acked + chunk <= window:
            part = data[sent:sent + chunk]
            await link.client.write_gatt_char(OTA, struct.pack("<I", sent) + part, response=False)
            sent += len(part)
        else:
            await asyncio.sleep(0.005)
    if link.ngo[1] in ("RECV", "RESEND"):
        print(f"передано за {time.time() - t0:.1f} с, блок проверяет образ…")
        r = await link.command("OTA_END", timeout=60)
        print("итог:", ",".join(r))
        if r[0] == "OK":
            print("блок перезагрузится в новую прошивку. Подключитесь снова в течение 10 минут: "
                  "иначе он вернёт прежнюю.")
    else:
        print("блок прервал обновление:", link.ngo[1])
    link.quiet = False


async def run(args):
    dev = await find(args.name)
    if dev is None:
        print("блок не найден: он рекламируется только при включённом радио (двигатель работает "
              "или первая минута после включения)")
        return

    loop = asyncio.get_running_loop()
    cmds = asyncio.Queue()

    def keyboard():
        for cmd in sys.stdin:
            if cmd.strip():
                loop.call_soon_threadsafe(cmds.put_nowait, cmd.strip())

    async with BleakClient(dev) as client:
        print(f"подключился, MTU {client.mtu_size}")
        link = Link(client, args.all)
        await client.start_notify(TX, link.on_tx)
        hello = asyncio.create_task(link.hello_loop())
        await asyncio.sleep(1.5)        # дать пройти первому HELLO и строке NGI
        if args.ota:
            await ota(link, args.ota)
            await asyncio.sleep(2)
        else:
            threading.Thread(target=keyboard, daemon=True).start()
            while client.is_connected:
                try:
                    cmd = await asyncio.wait_for(cmds.get(), timeout=0.5)
                except asyncio.TimeoutError:
                    continue
                await link.send(cmd)
        hello.cancel()
    print("связь закрыта")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", help="имя блока, например NoGPS-1B00; по умолчанию первый найденный")
    ap.add_argument("--all", action="store_true", help="печатать каждую NGD")
    ap.add_argument("--ota", metavar="ФАЙЛ", help="отправить блоку прошивку (.bin) и выйти")
    asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    main()
