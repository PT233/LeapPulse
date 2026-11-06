#!/usr/bin/env python3
"""Upload a LeapPulse application binary over an HC-05 SPP serial port."""

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

MAGIC = b"\xa5\x5a"
FRAME_SIZE = 20
PAYLOAD_SIZE = 14
START, DATA, FINISH, REPLY = 0x10, 0x11, 0x12, 0x80
STATUS = {
    1: "invalid frame",
    2: "missing or incomplete session",
    3: "invalid size or length",
    4: "unexpected sequence",
    5: "flash operation failed",
    6: "readback CRC32 mismatch; use --restart to erase the staging slot",
}


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value


def frame(kind, sequence, payload):
    body = bytes((kind, sequence, len(payload))) + payload.ljust(PAYLOAD_SIZE, b"\0")
    return MAGIC + body + bytes((crc8(body),))


class Link:
    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()

    def read_reply(self, kind, sequence, deadline):
        while time.monotonic() < deadline:
            self.buffer.extend(self.port.read(self.port.in_waiting or 1))
            while len(self.buffer) >= FRAME_SIZE:
                index = self.buffer.find(MAGIC)
                if index < 0:
                    self.buffer[:] = self.buffer[-1:]
                    break
                del self.buffer[:index]
                if len(self.buffer) < FRAME_SIZE:
                    break
                packet = self.buffer[:FRAME_SIZE]
                if crc8(packet[2:19]) != packet[19]:
                    del self.buffer[:1]
                    continue
                del self.buffer[:FRAME_SIZE]
                if tuple(packet[2:6]) == (REPLY, sequence, 10, kind):
                    return struct.unpack_from("<BII", packet, 6)
        return None

    def command(self, kind, sequence, payload, timeout):
        packet = frame(kind, sequence, payload)
        for _ in range(4):  # first attempt plus up to three retries
            if self.port.write(packet) != len(packet):
                raise RuntimeError("partial serial write")
            result = self.read_reply(kind, sequence, time.monotonic() + timeout)
            if result is None:
                continue
            status, offset, capacity = result
            if status == 0:
                return offset, capacity
            if status not in (1, 4):
                raise RuntimeError(STATUS.get(status, f"device returned status {status}"))
        raise RuntimeError(f"no successful ACK for command 0x{kind:02x} after three retries")


def upload(link, image, restart):
    size = len(image)
    if size == 0 or size > 0xFFFFFFFF:
        raise RuntimeError("image size is outside the protocol range")
    manifest = struct.pack("<II", size, zlib.crc32(image)) + (b"\x01" if restart else b"")
    offset, capacity = link.command(START, 0, manifest, timeout=20.0)
    if size > capacity or offset > size or (offset != size and offset % PAYLOAD_SIZE != 0):
        raise RuntimeError("device returned an invalid starting offset or capacity")
    print(f"Staging at byte {offset} of {size} ({capacity} bytes available).")
    while offset < size:
        chunk = image[offset : offset + PAYLOAD_SIZE]
        expected = offset + len(chunk)
        offset, _ = link.command(DATA, (offset // PAYLOAD_SIZE) & 0xFF, chunk, timeout=0.5)
        if offset != expected:
            raise RuntimeError(f"device acknowledged {offset} bytes, expected {expected}")
    offset, _ = link.command(FINISH, 0, b"", timeout=10.0)
    if offset != size:
        raise RuntimeError(f"device finished at byte {offset}, expected {size}")
    print("Device verified the staged image and scheduled activation.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="HC-05 SPP port, for example COM7 or /dev/rfcomm0")
    parser.add_argument("image", type=Path, help="raw application .bin linked for the run slot")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud (default: 115200)")
    parser.add_argument("--restart", action="store_true", help="erase staging and restart at zero")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        parser.exit(1, "Install pySerial with python -m pip install pyserial\n")
    try:
        image = args.image.read_bytes()
        with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
            upload(Link(port), image, args.restart)
    except (OSError, RuntimeError) as error:
        print(f"Upload failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
