#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Sends UART commands to the ESP32 without resetting it and prints what comes back.

    serialcmd.py --port COM14 [--seconds 5] [--logdir logs] [--wifi] "sync" ...

--wifi asks for the Wi-Fi name and password, so they stay out of the shell history and need no quoting.
The values of the wifi and token commands are not printed or logged.
Needs pyserial (esptool brings it along).
"""
import argparse
import getpass
import os
import time

import serial


def quoted(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    ap = argparse.ArgumentParser(description="Send UART commands to a wiiu-wake-esp32 board without reset.")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM14 or /dev/ttyUSB0")
    ap.add_argument("--seconds", type=float, default=5, help="how long to read after the last command")
    ap.add_argument("--logdir", help="also write serial_<time>.log and latest.log here")
    ap.add_argument("--wifi", action="store_true", help="ask for the Wi-Fi name and password and send them")
    ap.add_argument("commands", nargs="*")
    a = ap.parse_args()

    if a.wifi:
        ssid = input("Wi-Fi name: ")
        password = getpass.getpass("Wi-Fi password: ")
        a.commands.insert(0, f"wifi {quoted(ssid)} {quoted(password)}")
        if len(a.commands[0].encode()) > 119:
            ap.error("Wi-Fi name and password are too long for one command")

    files = []
    if a.logdir:
        os.makedirs(a.logdir, exist_ok=True)
        name = "serial_" + time.strftime("%Y%m%d_%H%M%S") + ".log"
        files = [open(os.path.join(a.logdir, n), "w", encoding="utf-8", newline="\n") for n in (name, "latest.log")]

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.05
    s.dtr = False  # keep DTR/RTS low, otherwise opening the port resets the ESP32
    s.rts = False
    s.open()
    t0 = time.time()
    buf = b""

    def emit(text):
        line = f"[{time.time() - t0:7.2f}] {time.strftime('%H:%M:%S')} {text}"
        print(line, flush=True)
        for f in files:
            f.write(line + "\n")
            f.flush()

    def pump(seconds):
        nonlocal buf
        end = time.time() + seconds
        while time.time() < end:
            buf += s.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                emit(line.decode("utf-8", "replace").rstrip())

    for cmd in a.commands:
        shown = cmd.split(" ", 1)[0] + " ***" if cmd.startswith(("wifi ", "token ")) else cmd
        emit("### TX " + shown)
        s.write(cmd.encode() + b"\n")
        pump(0.5)
    pump(a.seconds)
    s.close()


if __name__ == "__main__":
    main()
