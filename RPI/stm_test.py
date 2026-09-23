#!/usr/bin/env python3
"""
Standalone STM32 single-command tester.

Sends ONE motor command over /dev/ttyACM0, waits for the matching ACK, and
reports how long it took. Use it to measure each primitive in isolation:

    python3 stm_test.py turnr     # :1/MOTOR/TURN90R/90/90;   expect dx=30, dy=30
    python3 stm_test.py turnl     # :1/MOTOR/TURN90L/90/90;   expect dx=30, dy=30
    python3 stm_test.py fwd       # :1/MOTOR/FWD/40/30;       expect 30cm travel

This is a debug tool. It does NOT touch ctrl_center, the algorithm server, the
image server, Bluetooth or the tablet -- it only opens the STM serial port. It
is not referenced by the Makefile, so it cannot affect the ctrl_center build.

IMPORTANT: ctrl_center must NOT be running. Both programs would hold
/dev/ttyACM0 and steal each other's ACKs. The script refuses to start if it
sees ctrl_center alive.
"""

import argparse
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing.  Install it with:  python3 -m pip install --user pyserial")

DEVICE = "/dev/ttyACM0"
BAUD = 115200

# name -> (motor command, speed, value)   speeds match rpi_hal.c defaults
PRESETS = {
    "turnr": ("TURN90R", 90, 90),
    "turnl": ("TURN90L", 90, 90),
    "fwd":   ("FWD",     40, 30),
    "rev":   ("REVS",    40, 10),
    "revl":  ("REVL",    90, 90),
    "revr":  ("REVR",    90, 90),
}

EXPECTED = {
    "turnr": "dx = +30 cm, dy = +30 cm  (a 3-cell arc)",
    "turnl": "dx = -30 cm, dy = +30 cm  (a 3-cell arc)",
    "fwd":   "30 cm travel, no lateral drift",
    "rev":   "10 cm travel backwards",
    "revl":  "dx/dy per the reverse-left arc",
    "revr":  "dx/dy per the reverse-right arc",
}


def ctrl_center_running():
    """True if a ctrl_center process is alive (it would fight us for the port)."""
    try:
        subprocess.check_output(["pgrep", "-f", "ctrl_center"])
        return True
    except subprocess.CalledProcessError:
        return False
    except FileNotFoundError:
        # No pgrep -- can't check, let the caller decide.
        return False


def read_messages(ser, deadline):
    """Yield ';'-terminated messages from the STM until the deadline passes."""
    buf = ""
    while time.time() < deadline:
        chunk = ser.read(64)
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while ";" in buf:
            msg, buf = buf.split(";", 1)
            msg = msg.strip()
            if msg:
                yield msg + ";"


def run_command(cmd_str, cmd_id, timeout, device):
    print(f"Opening {device} at {BAUD} baud...")
    try:
        ser = serial.Serial(device, BAUD, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {device}: {e}")

    with ser:
        time.sleep(0.2)          # let the port settle
        ser.reset_input_buffer()  # drop anything stale so we only see this command's reply

        print(f"TX  {cmd_str}")
        ser.write(cmd_str.encode("ascii"))
        ser.flush()
        sent_at = time.time()

        want = f"!{cmd_id}/"
        deadline = sent_at + timeout

        for msg in read_messages(ser, deadline):
            dt = time.time() - sent_at
            print(f"RX  {msg}    (+{dt:.2f}s)")
            if msg.startswith(want) or msg.startswith(f"!{cmd_id};"):
                print(f"\nACK for command {cmd_id} after {dt:.2f}s")
                return True
            if "/ERROR/" in msg:
                print("\nSTM reported an ERROR -- see the message above.")
                return False

        print(f"\nTIMEOUT: no ACK for command {cmd_id} within {timeout}s.")
        print("If the robot moved anyway, the STM ran the command but did not ACK it.")
        return False


def main():
    p = argparse.ArgumentParser(
        description="Send one command to the STM32 and wait for its ACK.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="presets: " + ", ".join(PRESETS),
    )
    p.add_argument("preset", nargs="?", choices=sorted(PRESETS),
                   help="which primitive to test")
    p.add_argument("--raw", metavar="STR",
                   help='send a literal string instead, e.g. ":1/MOTOR/FWD/40/20;"')
    p.add_argument("--speed", type=int, help="override the preset speed")
    p.add_argument("--value", type=int, help="override the preset distance/angle")
    p.add_argument("--id", type=int, default=1, help="command id (default 1)")
    p.add_argument("--timeout", type=float, default=20.0,
                   help="seconds to wait for the ACK (default 20)")
    p.add_argument("--device", default=DEVICE, help=f"serial device (default {DEVICE})")
    p.add_argument("--force", action="store_true",
                   help="run even if ctrl_center appears to be running")
    args = p.parse_args()

    if not args.preset and not args.raw:
        p.error("give a preset (e.g. 'turnr') or --raw")

    if ctrl_center_running() and not args.force:
        sys.exit(
            "ctrl_center is running -- it already owns " + args.device + ".\n"
            "Stop it first:   sudo pkill -f ctrl_center\n"
            "(or pass --force if you really know what you are doing)"
        )

    if args.raw:
        cmd_str = args.raw
        # Pull the id out of ":<id>/..." so we can match the ACK.
        try:
            cmd_id = int(cmd_str.lstrip(":").split("/", 1)[0])
        except (ValueError, IndexError):
            cmd_id = args.id
        label = None
    else:
        motor, speed, value = PRESETS[args.preset]
        if args.speed is not None:
            speed = args.speed
        if args.value is not None:
            value = args.value
        cmd_id = args.id
        cmd_str = f":{cmd_id}/MOTOR/{motor}/{speed}/{value};"
        label = args.preset

    print("=" * 60)
    if label:
        print(f"TEST: {label}      expect {EXPECTED[label]}")
    print("Measure the SAME reference point on the car before and after")
    print("(chassis centre is easiest). Mark the floor before you start.")
    print("=" * 60)

    ok = run_command(cmd_str, cmd_id, args.timeout, args.device)

    print("\nRecord your measurement:")
    print(f"  command : {cmd_str}")
    print("  start   : x = ______ cm   y = ______ cm")
    print("  end     : x = ______ cm   y = ______ cm")
    print("  delta   : dx = ______ cm  dy = ______ cm")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
