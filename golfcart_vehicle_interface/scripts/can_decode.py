#!/usr/bin/env python3
"""Live CAN decoder for CAX_ADS_CAN.dbc.

Reads frames from a SocketCAN interface and pretty-prints them using the
package's DBC file. Intended for bench debugging — not a runtime dependency
of the interface node.

Dependencies (host install):
    pip install --user cantools python-can

Examples:
    ./can_decode.py                              # default: can0, all known IDs
    ./can_decode.py -i can1 --raw                # also show payload hex
    ./can_decode.py --filter-id 0x75 -f 257      # only these IDs
    ./can_decode.py --quiet                      # skip undecoded frames
    ./can_decode.py --log raw.log                # decode a candump -L file
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

try:
    import cantools
except ImportError:
    sys.exit("missing dep: pip install --user cantools python-can")

DEFAULT_DBC = Path(__file__).resolve().parent.parent / "CAX_ADS_CAN.dbc"


def parse_id(value: str) -> int:
    value = value.strip()
    return int(value, 16) if value.lower().startswith("0x") else int(value)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("-i", "--interface", default="can0", help="SocketCAN iface (default: can0)")
    p.add_argument("--bustype", default="socketcan", help="python-can bustype (default: socketcan)")
    p.add_argument(
        "--dbc",
        default=os.environ.get("CAX_ADS_DBC", str(DEFAULT_DBC)),
        help=f"DBC file (default: {DEFAULT_DBC})",
    )
    p.add_argument(
        "-f", "--filter-id", action="append", type=parse_id, default=[],
        help="Only show this CAN ID (hex 0x.. or decimal). Repeatable.",
    )
    p.add_argument("--raw", action="store_true", help="Also print payload bytes as hex")
    p.add_argument("--quiet", action="store_true", help="Suppress undecoded frames")
    p.add_argument("--log", help="Decode a candump -L logfile instead of live bus")
    return p.parse_args()


def fmt_payload(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def fmt_decoded(decoded: dict) -> str:
    return ", ".join(f"{k}={v}" for k, v in decoded.items())


def decode_one(db, ts: float, can_id: int, data: bytes, args) -> None:
    if args.filter_id and can_id not in args.filter_id:
        return
    try:
        msg = db.get_message_by_frame_id(can_id)
        decoded = db.decode_message(can_id, data)
        line = f"[{ts:10.3f}] id=0x{can_id:03X} ({can_id}) {msg.name}: {fmt_decoded(decoded)}"
    except (KeyError, cantools.database.DecodeError) as exc:
        if args.quiet:
            return
        line = f"[{ts:10.3f}] id=0x{can_id:03X} ({can_id}) <undecoded: {exc}>"
    if args.raw:
        line += f"  [{fmt_payload(data)}]"
    print(line, flush=True)


def replay_log(db, args) -> None:
    """Parse a `candump -L` logfile: '(t) iface ID#HEXBYTES'."""
    with open(args.log) as f:
        for raw in f:
            raw = raw.strip()
            if not raw or not raw.startswith("("):
                continue
            try:
                ts_part, _iface, frame_part = raw.split(" ", 2)
                ts = float(ts_part.strip("()"))
                id_str, payload = frame_part.split("#", 1)
                # Strip optional flags after `##` (FD frames).
                payload = payload.split(" ")[0].split("##")[-1]
                can_id = int(id_str, 16)
                data = bytes.fromhex(payload)
            except ValueError:
                continue
            decode_one(db, ts, can_id, data, args)


def listen_live(db, args) -> None:
    try:
        import can
    except ImportError:
        sys.exit("missing dep: pip install --user python-can")

    bus = can.Bus(channel=args.interface, interface=args.bustype)
    print(
        f"can_decode: listening on {args.interface} "
        f"(filter_ids={args.filter_id or 'all'}, raw={args.raw}, quiet={args.quiet}). "
        f"Ctrl-C to stop.",
        file=sys.stderr,
    )
    start = time.monotonic()
    try:
        for frame in bus:
            if frame.is_error_frame or frame.is_remote_frame:
                continue
            ts = time.monotonic() - start
            decode_one(db, ts, frame.arbitration_id, bytes(frame.data), args)
    finally:
        bus.shutdown()


def main() -> int:
    args = parse_args()
    if not Path(args.dbc).exists():
        sys.exit(f"DBC not found: {args.dbc} (set CAX_ADS_DBC or pass --dbc)")
    db = cantools.database.load_file(args.dbc)

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    if args.log:
        replay_log(db, args)
    else:
        listen_live(db, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
