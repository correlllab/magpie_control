#!/usr/bin/env python3
"""Utilities to discover and configure AX-12 Dynamixel motors.

This helper is intended for first-time bring-up of fresh motors.
"""

from __future__ import annotations

import argparse
import glob
from dataclasses import dataclass
from typing import Iterable

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

AX12_PROTOCOL_VERSION = 1.0
AX12_ADDR_ID = 3
AX12_ADDR_BAUD = 4

# Common baud-rate settings for AX-12A.
# Formula: baud = 2_000_000 / (value + 1)
AX12_BAUD_TO_REG = {
    1_000_000: 1,
    500_000: 3,
    400_000: 4,
    250_000: 7,
    200_000: 9,
    115_200: 16,
    57_600: 34,
    19_200: 103,
    9_600: 207,
}


@dataclass(frozen=True)
class Detection:
    port: str
    baudrate: int
    motor_id: int
    model_number: int


def candidate_ports() -> list[str]:
    """Return likely serial ports for USB Dynamixel adapters on Linux."""
    ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    return ports


def _open_port(port: str, baudrate: int) -> tuple[PortHandler, PacketHandler] | tuple[None, None]:
    port_handler = PortHandler(port)
    packet_handler = PacketHandler(AX12_PROTOCOL_VERSION)

    if not port_handler.openPort():
        return None, None
    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        return None, None

    # Some USB-serial adapters can block indefinitely on writes when hardware
    # is disconnected or unpowered. Keep timeouts short for bring-up scans.
    if hasattr(port_handler, "ser") and port_handler.ser is not None:
        port_handler.ser.timeout = 0.2
        port_handler.ser.write_timeout = 0.2

    return port_handler, packet_handler


def ping_id(port: str, baudrate: int, motor_id: int) -> Detection | None:
    """Ping a specific ID on one port/baud and return detection info if found."""
    handlers = _open_port(port, baudrate)
    if handlers == (None, None):
        return None

    port_handler, packet_handler = handlers
    try:
        model_number, comm_result, dxl_error = packet_handler.ping(port_handler, motor_id)
        if comm_result == COMM_SUCCESS and dxl_error == 0:
            return Detection(port, baudrate, motor_id, model_number)
        return None
    finally:
        port_handler.closePort()


def scan(
    ports: Iterable[str],
    baudrates: Iterable[int],
    id_min: int,
    id_max: int,
) -> list[Detection]:
    """Scan ids on all provided ports/bauds."""
    detections: list[Detection] = []
    for port in ports:
        for baud in baudrates:
            handlers = _open_port(port, baud)
            if handlers == (None, None):
                continue

            port_handler, packet_handler = handlers
            try:
                for motor_id in range(id_min, id_max + 1):
                    try:
                        model_number, comm_result, dxl_error = packet_handler.ping(port_handler, motor_id)
                    except Exception:
                        # Continue scanning if one ID transaction fails.
                        continue

                    if comm_result == COMM_SUCCESS and dxl_error == 0:
                        detections.append(Detection(port, baud, motor_id, model_number))
            finally:
                port_handler.closePort()
    return detections


def set_id(port: str, baudrate: int, current_id: int, new_id: int) -> None:
    if not (0 <= current_id <= 253 and 0 <= new_id <= 253):
        raise ValueError("IDs must be in [0, 253].")

    handlers = _open_port(port, baudrate)
    if handlers == (None, None):
        raise RuntimeError(f"Could not open {port} at {baudrate} bps")

    port_handler, packet_handler = handlers
    try:
        _, comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, current_id, AX12_ADDR_ID, new_id
        )
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(packet_handler.getTxRxResult(comm_result))
        if dxl_error != 0:
            raise RuntimeError(packet_handler.getRxPacketError(dxl_error))
    finally:
        port_handler.closePort()


def set_baud(port: str, baudrate: int, motor_id: int, new_baud: int) -> None:
    if new_baud not in AX12_BAUD_TO_REG:
        supported = ", ".join(str(v) for v in sorted(AX12_BAUD_TO_REG))
        raise ValueError(f"Unsupported new baudrate {new_baud}. Supported: {supported}")

    handlers = _open_port(port, baudrate)
    if handlers == (None, None):
        raise RuntimeError(f"Could not open {port} at {baudrate} bps")

    port_handler, packet_handler = handlers
    try:
        _, comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, AX12_ADDR_BAUD, AX12_BAUD_TO_REG[new_baud]
        )
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(packet_handler.getTxRxResult(comm_result))
        if dxl_error != 0:
            raise RuntimeError(packet_handler.getRxPacketError(dxl_error))
    finally:
        port_handler.closePort()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Discover/configure AX-12 motors")
    sub = parser.add_subparsers(dest="cmd", required=True)

    scan_parser = sub.add_parser("scan", help="Scan ports, bauds, and IDs")
    scan_parser.add_argument(
        "--ports",
        nargs="*",
        default=None,
        help="Ports to scan. Default: auto-detect /dev/ttyACM* and /dev/ttyUSB*",
    )
    scan_parser.add_argument(
        "--bauds",
        nargs="+",
        type=int,
        default=[1_000_000, 57_600],
        help="Baudrates to scan (default: 1000000 57600)",
    )
    scan_parser.add_argument("--id-min", type=int, default=0, help="Minimum ID to scan")
    scan_parser.add_argument("--id-max", type=int, default=30, help="Maximum ID to scan")

    set_id_parser = sub.add_parser("set-id", help="Change a motor ID")
    set_id_parser.add_argument("--port", required=True, help="Serial port, e.g., /dev/ttyACM0")
    set_id_parser.add_argument("--baud", type=int, default=1_000_000, help="Current baudrate")
    set_id_parser.add_argument("--current-id", type=int, required=True, help="Current motor ID")
    set_id_parser.add_argument("--new-id", type=int, required=True, help="New motor ID")

    set_baud_parser = sub.add_parser("set-baud", help="Change motor baudrate")
    set_baud_parser.add_argument("--port", required=True, help="Serial port, e.g., /dev/ttyACM0")
    set_baud_parser.add_argument("--baud", type=int, default=1_000_000, help="Current baudrate")
    set_baud_parser.add_argument("--id", type=int, required=True, help="Motor ID")
    set_baud_parser.add_argument("--new-baud", type=int, required=True, help="New baudrate")

    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    if args.cmd == "scan":
        ports = args.ports if args.ports else candidate_ports()
        if not ports:
            print("No candidate serial ports found (/dev/ttyACM* or /dev/ttyUSB*)")
            return 1

        print(f"Scanning ports={ports}, bauds={args.bauds}, ids={args.id_min}..{args.id_max}")
        found = scan(ports=ports, baudrates=args.bauds, id_min=args.id_min, id_max=args.id_max)
        if not found:
            print("No motors found.")
            return 2

        print("Detected motors:")
        for d in found:
            print(
                f"  port={d.port} baud={d.baudrate} id={d.motor_id} model={d.model_number}"
            )
        return 0

    if args.cmd == "set-id":
        print(
            "Changing motor ID. Make sure only ONE motor with the current ID is connected to the bus."
        )
        set_id(
            port=args.port,
            baudrate=args.baud,
            current_id=args.current_id,
            new_id=args.new_id,
        )
        verify = ping_id(args.port, args.baud, args.new_id)
        if verify is None:
            print(
                "ID write sent, but verification ping failed at the same baud. "
                "If you also changed baud previously, rescan with that baud."
            )
            return 3
        print(f"Verified new ID={args.new_id} on {args.port} @ {args.baud}")
        return 0

    if args.cmd == "set-baud":
        old_baud = args.baud
        print(
            "Changing baudrate. Verification is done by pinging at the new baud."
        )
        set_baud(port=args.port, baudrate=old_baud, motor_id=args.id, new_baud=args.new_baud)
        verify = ping_id(args.port, args.new_baud, args.id)
        if verify is None:
            print("Baud write sent, but verification ping at new baud failed.")
            return 3
        print(f"Verified ID={args.id} on {args.port} @ {args.new_baud}")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
