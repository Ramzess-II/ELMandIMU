#!/usr/bin/env python3
"""Готовит файл выпуска из собранной прошивки.

Копирует build/ELMandIMU.bin в release/nogps-<чип>-<плата>-<версия>.bin и дописывает сведения о нём
в release/nogps-<версия>.json: версия, чип, плата, размер, SHA-256. Версия и чип берутся из самого
образа, плата — из sdkconfig (CONFIG_NOGPS_BOARD). Образ должен быть подписан: блок принимает по
радио только прошивку с той же подписью, что у работающей в нём.

    python tools/make_release.py
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import struct

CHIPS = {0x0009: "esp32s3", 0x0005: "esp32c3"}
APP_DESC_MAGIC = 0xABCD5432


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="build/ELMandIMU.bin")
    ap.add_argument("--sdkconfig", default="sdkconfig")
    ap.add_argument("--out", default="release")
    args = ap.parse_args()

    data = open(args.bin, "rb").read()
    # Заголовок образа — 24 байта, заголовок первого сегмента — 8, дальше описание приложения.
    chip_id = struct.unpack_from("<H", data, 12)[0]
    magic, = struct.unpack_from("<I", data, 32)
    if data[0] != 0xE9 or magic != APP_DESC_MAGIC or chip_id not in CHIPS:
        raise SystemExit(f"{args.bin}: не похоже на прошивку ESP32-S3 или ESP32-C3")
    version = data[32 + 16:32 + 48].split(b"\0")[0].decode()
    chip = CHIPS[chip_id]
    board = re.search(r'^CONFIG_NOGPS_BOARD="(.*)"', open(args.sdkconfig, encoding="utf-8").read(), re.M)
    if not board:
        raise SystemExit(f"в {args.sdkconfig} нет CONFIG_NOGPS_BOARD")
    board = board.group(1)

    os.makedirs(args.out, exist_ok=True)
    name = f"nogps-{chip}-{board}-{version}.bin"
    shutil.copyfile(args.bin, os.path.join(args.out, name))
    entry = {"file": name, "version": version, "chip": chip, "board": board, "size": len(data),
             "sha256": hashlib.sha256(data).hexdigest()}

    index = os.path.join(args.out, f"nogps-{version}.json")
    files = []
    if os.path.exists(index):
        files = [f for f in json.load(open(index, encoding="utf-8"))["files"] if f["file"] != name]
    files.append(entry)
    json.dump({"version": version, "files": files}, open(index, "w", encoding="utf-8"), indent=2)
    print(f"{os.path.join(args.out, name)}: {len(data)} байт, SHA-256 {entry['sha256']}")
    print(f"описание: {index}")


if __name__ == "__main__":
    main()
