#!/usr/bin/env python3
"""Preflight checks for the DOFBOT I2C control bus.

The default checks only open the device and select the target slave address.
They do not send servo command frames and should not move the arm.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import re
import shutil
import subprocess
import sys


I2C_SLAVE = 0x0703


def parse_address(value: str) -> int:
    try:
        address = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid I2C address: {value}") from exc
    if address < 0 or address > 0x7F:
        raise argparse.ArgumentTypeError("I2C address must be between 0x00 and 0x7f")
    return address


def bus_number_from_device(device: str) -> str:
    match = re.fullmatch(r"/dev/i2c-(\d+)", device)
    if not match:
        raise ValueError(f"device must look like /dev/i2c-N, got {device}")
    return match.group(1)


def check_open_and_select(device: str, address: int) -> bool:
    if not os.path.exists(device):
        print(f"FAIL device missing: {device}")
        return False

    if not os.access(device, os.R_OK | os.W_OK):
        print(f"FAIL no read/write permission for {device}")
        print("HINT try sudo for this check, or add the user to the i2c group")
        return False

    try:
        fd = os.open(device, os.O_RDWR)
    except OSError as exc:
        print(f"FAIL cannot open {device}: {exc}")
        return False

    try:
        fcntl.ioctl(fd, I2C_SLAVE, address)
    except OSError as exc:
        print(f"FAIL cannot select address 0x{address:02x} on {device}: {exc}")
        return False
    finally:
        os.close(fd)

    print(f"OK opened {device} and selected address 0x{address:02x}")
    return True


def vendor_probe(bus_number: int) -> bool:
    try:
        from Arm_Lib import Arm_Device
    except Exception as exc:
        print(f"WARN Arm_Lib unavailable, skip vendor probe: {exc}")
        return True

    try:
        arm = Arm_Device(bus_number)
    except Exception as exc:
        print(f"FAIL Arm_Lib could not open bus {bus_number}: {exc}")
        return False

    try:
        version = arm.Arm_get_hardversion()
        ping = arm.Arm_ping_servo(1)
    except Exception as exc:
        print(f"FAIL vendor probe failed: {exc}")
        return False

    print(f"OK Arm_Lib bus={arm.get_i2c_bus_num()} version={version} ping1={ping}")
    if version is None or ping != 0xDA:
        print("FAIL vendor probe did not receive the expected board response")
        return False
    return True


def scan_bus(device: str, address: int) -> bool:
    i2cdetect = shutil.which("i2cdetect")
    if not i2cdetect:
        print("FAIL i2cdetect not found")
        return False

    try:
        bus = bus_number_from_device(device)
    except ValueError as exc:
        print(f"FAIL {exc}")
        return False

    result = subprocess.run(
        [i2cdetect, "-y", bus],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.stdout:
        print(result.stdout.rstrip())
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr)
        print(f"FAIL i2cdetect exited with {result.returncode}")
        return False

    visible = address_visible_in_scan(result.stdout, address)
    if visible:
        print(f"OK detected address 0x{address:02x} or an occupied address marker")
    else:
        print(
            f"WARN address 0x{address:02x} was not visible in i2cdetect output; "
            "treating vendor probe as the authoritative check"
        )
    return True


def address_visible_in_scan(scan_output: str, address: int) -> bool:
    needle = f"{address:02x}"
    for line in scan_output.lower().splitlines():
        line = line.strip()
        if not re.match(r"^[0-7]0:", line):
            continue

        row_label, *tokens = line.replace(":", "").split()
        row_base = int(row_label, 16)
        for column, token in enumerate(tokens[:16]):
            current_address = row_base + column
            if current_address == address and token in {needle, "uu"}:
                return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/i2c-7")
    parser.add_argument("--address", type=parse_address, default=parse_address("0x15"))
    parser.add_argument(
        "--scan",
        action="store_true",
        help="also run i2cdetect on the bus; this probes the I2C bus but does not send servo frames",
    )
    args = parser.parse_args()

    ok = check_open_and_select(args.device, args.address)
    try:
        bus_number = int(bus_number_from_device(args.device))
    except ValueError as exc:
        print(f"FAIL {exc}")
        return 1

    ok = vendor_probe(bus_number) and ok
    if args.scan:
        ok = scan_bus(args.device, args.address) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
