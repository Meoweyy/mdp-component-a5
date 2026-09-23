#!/usr/bin/env python3
"""
A.5 -- "Navigating around the obstacle".

Point the car at the block from anywhere in front of it and run:

    python3 a5.py

It will:
  1. drive straight at the block until the ultrasonic reads the standoff
     (FWDUSH -- holds heading with the gyro, so it arrives square);
  2. photograph the face and send it to the image server;
  3. if the answer is the Bullseye marker, loop round to the next face
     and photograph that one;
  4. stop the moment any other valid image id comes back.

Nothing else needs to be running on the Pi -- no ctrl_center, no
and_center. The image server on the laptop does need to be up.

Options:
    python3 a5.py --standoff 280     stop distance, MILLIMETRES (OLED "d:")
    python3 a5.py --faces 4          how many faces to try
    python3 a5.py --align 22         the alignment knob, see MOVES below
    python3 a5.py --dry-run          print the commands, drive nothing
"""

import argparse
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  python3 -m pip install --user pyserial")

try:
    import requests
except ImportError:
    sys.exit("requests missing:  python3 -m pip install --user requests")

DEVICE = "/dev/ttyACM0"
BAUD = 115200

IMAGE_SERVER = "http://192.168.23.11:4000/detect"
CAPTURE_FILE = "/tmp/a5_capture.jpg"

BULLSEYE_ID = 41          # the marker; never a valid answer
APPROACH_SPEED = 35       # speed units (parser multiplies by 71, max 101)
MOVE_SPEED = 40


def send(ser, cmd_str, cmd_id, timeout=25.0, dry=False):
    """Send one command and wait for its '!<id>/...' reply."""
    print(f"  TX  {cmd_str}")
    if dry:
        return True

    ser.reset_input_buffer()
    ser.write(cmd_str.encode("ascii"))
    ser.flush()

    deadline = time.time() + timeout
    buf = ""
    want = f"!{cmd_id}/"
    while time.time() < deadline:
        chunk = ser.read(64)
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while ";" in buf:
            msg, buf = buf.split(";", 1)
            msg = msg.strip() + ";"
            if msg.startswith("!"):
                print(f"  RX  {msg}")
            if msg.startswith(want):
                if "/ERROR/" in msg:
                    print("  !! STM reported an error")
                    return False
                return True

    print(f"  !! no reply to command {cmd_id} within {timeout}s")
    return False


def approach(ser, cmd_id, standoff_mm, dry=False):
    """Drive until the ultrasonic reads `standoff_mm`, holding heading."""
    cmd = f":{cmd_id}/MOTOR/FWDUSH/{APPROACH_SPEED}/{standoff_mm};"
    return send(ser, cmd, cmd_id, timeout=30.0, dry=dry)


def snapshot(object_id, dry=False):
    """Photograph the face and ask the image server what it is.

    Returns the img_id, or None if nothing was recognised.
    """
    print("  [cam] capturing...")
    if dry:
        return None

    rc = subprocess.call(
        ["raspistill", "-n", "-t", "200", "-w", "640", "-h", "480",
         "-q", "75", "-o", CAPTURE_FILE]
    )
    if rc != 0:
        print("  !! raspistill failed")
        return None

    try:
        with open(CAPTURE_FILE, "rb") as fh:
            r = requests.post(
                IMAGE_SERVER,
                files={"image": fh},
                data={"object_id": str(object_id)},
                timeout=20,
            )
    except Exception as e:
        print(f"  !! image server unreachable: {e}")
        return None

    if r.status_code != 200:
        print(f"  !! image server returned {r.status_code}")
        return None

    data = r.json()
    objs = data.get("objects") or []
    if not objs:
        print("  [cam] nothing detected")
        return None

    best = objs[0]
    print(f"  [cam] {best.get('class_label')}  img_id={best.get('img_id')}"
          f"  conf={best.get('confidence', 0):.2f}")
    return best.get("img_id")


def next_face(ser, cmd_id, standoff_mm, align, swing, dry=False):
    """Loop from the current face round to the next one, anticlockwise.

    SPACE. The two 90-degree arcs each throw the car 30cm sideways, so
    after both of them its centre is 60cm from the block centre and its
    outer edge 69.5cm -- about 65cm clear of the block face. That is the
    floor; no amount of reversing first reduces it. A forward leg adds to
    it 1:1, which is why --swing defaults to 0.

    THE TWO KNOBS

      --align  the reverse between the last two turns. Sets how far ALONG
               the block the car ends up, and is the only number that
               normally needs tuning. Raise it if the car overshoots the
               next face, lower it if it stops short, roughly 1cm per 1cm.

      --swing  a forward leg after the first turn, pushing the loop
               further out. 0 fits 65cm of clearance; every extra cm
               needs another cm of space. Only raise it if you have room
               and want more margin on the final approach.

    Note the default align of 35 is derived for standoff 190 with no
    swing, not measured. The 4cm offset measured on 23 Sep was for
    standoff 280 with swing 30 -- different geometry, so expect to
    re-tune on the first run.
    """
    moves = [
        (f":{{id}}/MOTOR/REVS/{MOVE_SPEED}/25;",      "back off, clear the corner"),
        (f":{{id}}/MOTOR/TURN90R/90/90;",             "swing out"),
    ]
    if swing > 0:
        moves.append(
            (f":{{id}}/MOTOR/FWD/{MOVE_SPEED}/{swing};", "run past the block  <-- --swing")
        )
    moves += [
        (f":{{id}}/MOTOR/TURN90L/90/90;",             "turn alongside"),
        (f":{{id}}/MOTOR/REVS/{MOVE_SPEED}/{align};", "line up  <-- --align"),
        (f":{{id}}/MOTOR/TURN90L/90/90;",             "turn in to face it"),
    ]

    for template, why in moves:
        print(f"  ({why})")
        if not send(ser, template.format(id=cmd_id), cmd_id, dry=dry):
            return None
        cmd_id += 1

    # Final leg by ultrasonic, so the standoff is right however far the
    # loop drifted on the way round.
    print("  (close in on the new face)")
    if not approach(ser, cmd_id, standoff_mm, dry=dry):
        return None
    return cmd_id + 1


def main():
    p = argparse.ArgumentParser(description="A.5 obstacle face sweep.")
    p.add_argument("--standoff", type=int, default=190,
                   help="stop distance in MILLIMETRES (default 190)")
    p.add_argument("--faces", type=int, default=4,
                   help="how many faces to try (default 4)")
    p.add_argument("--align", type=int, default=35,
                   help="reverse between the last two turns, cm (default 35)")
    p.add_argument("--swing", type=int, default=0,
                   help="forward leg after the first turn, cm. 0 fits 65cm of "
                        "clearance; each extra cm needs another cm of space")
    p.add_argument("--object-id", type=int, default=1,
                   help="id reported to the image server (default 1)")
    p.add_argument("--device", default=DEVICE)
    p.add_argument("--dry-run", action="store_true",
                   help="print the commands without driving")
    args = p.parse_args()

    try:
        subprocess.check_output(["pgrep", "-f", "_center"])
        sys.exit("ctrl_center/and_center is running and owns the serial port.\n"
                 "Stop it first:  sudo pkill -f _center")
    except subprocess.CalledProcessError:
        pass
    except FileNotFoundError:
        pass

    ser = None
    if not args.dry_run:
        try:
            ser = serial.Serial(args.device, BAUD, timeout=0.2)
        except serial.SerialException as e:
            sys.exit(f"Could not open {args.device}: {e}")
        time.sleep(0.2)

    cmd_id = 1
    print("=" * 60)
    print(f"A.5 sweep -- standoff {args.standoff}mm, up to {args.faces} faces")
    print("=" * 60)

    print("\n[1] Approaching the block")
    if not approach(ser, cmd_id, args.standoff, dry=args.dry_run):
        sys.exit("Approach failed.")
    cmd_id += 1

    for face in range(1, args.faces + 1):
        print(f"\n[2] Face {face} of {args.faces}")
        img_id = snapshot(args.object_id, dry=args.dry_run)

        if img_id is not None and img_id != BULLSEYE_ID:
            print(f"\n*** Valid image found on face {face}: img_id={img_id} ***")
            if ser:
                ser.close()
            return 0

        if img_id == BULLSEYE_ID:
            print("  -> Bullseye marker. Trying the next face.")
        else:
            print("  -> nothing valid here. Trying the next face.")

        if face == args.faces:
            break

        print(f"\n[3] Looping to face {face + 1}")
        nxt = next_face(ser, cmd_id, args.standoff, args.align, args.swing,
                        dry=args.dry_run)
        if nxt is None:
            sys.exit("Move to the next face failed.")
        cmd_id = nxt

    print("\nAll faces checked, no valid image found.")
    if ser:
        ser.close()
    return 1


if __name__ == "__main__":
    sys.exit(main())
