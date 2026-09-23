# MDP Component A.5 — Navigating around the obstacle

Checklist item A.5: the robot approaches a block placed at an arbitrary
distance, photographs the face, and if it sees the Bullseye marker it
navigates around the block scanning the other faces, stopping only once a
valid image is found.

## Layout

```
RPI/         runs on the Raspberry Pi
Image/       runs on the laptop — YOLOv8 detection server
firmware/    STM32F407VET6 sources (drop into MDP/Core/ of the CubeIDE project)
```

The Pi takes the photo (`raspistill`, inside `a5.py`) and POSTs it to the
laptop; the laptop runs the model and answers with the image id. Both halves
are in this repo — neither works alone.

## Running it

### 1. Laptop — image server

```bash
cd Image
python -m venv .venv
.venv\Scripts\activate            # Windows;  source .venv/bin/activate on Mac/Linux
pip install -r requirements.txt
python object_detection_server.py
```

Listens on port **4000**. Do not commit or copy a `.venv` between machines —
a virtualenv hardcodes absolute paths to the Python that built it and will
fail with `No Python at '...'` on anyone else's computer. Build your own.

Browse `http://localhost:4000/` for the detection gallery, or run
`python live_view.py` for the live viewer on port **4100**.

The Bullseye marker needs `INFERENCE_SIZE = 960`. Measured on a real snap
photo it is missed entirely at 416 and 640, and found at 0.64 confidence at
960 — about 150 ms instead of 30 ms. Symbols detect fine at every size, so
960 costs a little latency and loses nothing. Do not lower it.

### 2. Pi — the run

```bash
sudo pkill -f _center        # release /dev/ttyACM0
python3 a5.py --align 28
```

### Network

Both machines must be on the robot network (192.168.23.x). `a5.py` has the
server address hardcoded at the top:

```python
IMAGE_SERVER = "http://192.168.23.11:4000/detect"
```

If the laptop running the server picks up a different address, edit that line.
`DISPLAY_IP` in `object_detection_server.py` is cosmetic — it only affects the
"Running on" banner.

### Options

| Flag | Default | Meaning |
|---|---|---|
| `--standoff` | 190 | stop distance in **millimetres**, matches the OLED `d:` line |
| `--faces` | 4 | how many faces to try |
| `--align` | 35 | reverse between the last two turns, cm — the alignment knob |
| `--swing` | 0 | forward leg after the first turn, cm; each extra cm needs another cm of clearance |
| `--dry-run` | — | print the commands without driving |

`--align` is the only number that normally needs tuning. Raise it if the car
stops short of the next face, lower it if it overshoots, roughly 1 cm per 1 cm.
Judge it on the **first** loop only — the error is systematic, so it repeats
rather than drifting randomly.

## The loop, one face to the next

Anticlockwise, six moves:

```
REVS 25      back off, clear the corner
TURN90R      swing out
(FWD swing)  only if --swing > 0
TURN90L      turn alongside
REVS align   line up          <-- the knob
TURN90L      turn in to face it
FWDUSH       close in by ultrasonic
```

The final leg is always ultrasonic, so the standoff is correct however far
the loop drifted on the way round — errors do not compound from face to face.

Each 90° arc throws the car about 30 cm sideways, so after two of them its
outer edge sits roughly 65 cm from the block. That is the floor; reversing
first does not reduce it. Plan for 65 cm of clearance on all sides.

## Firmware additions

Additive only — nothing existing was modified.

- **`APPROACHUS`** in `robot_types.h`, wire name **`FWDUSH`**. Drives forward
  until the ultrasonic reads the standoff **while holding heading with the
  gyro**, so the car arrives square. Implemented as `motorApproachUntil()` in
  `main.c`.
- **Gyro heading hold** on the servo in `motorPidForward` / `motorPidReverse`,
  gains `MTN_YAW_KP` and `MTN_YAW_KP_REV` in `motion.h`. The target is snapped
  to the nearest 90° at boot rather than locked to whatever heading the car
  happened to start at.
- Ultrasonic task: the settle delay now runs **after** the trigger, not before,
  which was returning a one-cycle-stale reading.

### Wire protocol

```
:<id>/MOTOR/<CMD>/<speed>/<dist_or_angle>;   ->   !<id>/DONE;
```

The speed field is multiplied by 71 by `rxSerialParse` and rejected above
7199, so the usable range is 1..101. Passing a raw PWM value such as 2500
returns `INVALID_SPEED`.

## Gotchas

- **Power-cycle the car on the floor with hands off for 3 seconds.** The gyro
  bias is calibrated during boot; moving the car then poisons it and the yaw
  will drift at tens of degrees per second.
- `ctrl_center` / `and_center` own `/dev/ttyACM0`. `a5.py` refuses to start
  while either is alive.
- Pressing reset on the STM32 can drop the SSH session. Reconnect and re-run
  from the start.

## Known unfixed

- `REVR` lands about 7 cm long; the servo is saturated at ~225 so PWM makes no
  difference. Worked around with a 7 cm forward nudge on the Pi side.
- The pivot turns `TURNR` / `TURNL` have never been properly measured.
- On the last run the approach did not track straight over 60–70 cm; `FWDUSH`
  heading hold still needs investigation.
