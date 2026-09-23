#!/usr/bin/env python3
"""
Standalone live viewer for MDP detections.

Watches the ./detections folder and shows each snapshot as it lands, newest
first. Completely independent of object_detection_server.py -- it only reads
the image files that server writes, so it cannot affect detection or the robot.

Run it alongside the detection server:

    python live_view.py            # http://localhost:4100
    python live_view.py --port 5500

The detection server keeps port 4000; this uses 4100 by default.
"""

import argparse
import os
import re

from flask import Flask, jsonify, send_from_directory

app = Flask(__name__)

DETECTIONS_DIR = "./detections"

# object_detection_server.py writes:
#   detection_<object_id>.jpg           -> annotated, only when something was found
#   original_detection_<object_id>.jpg  -> the raw frame, always
ANNOTATED_RE = re.compile(r"^detection_(.+)\.jpg$", re.IGNORECASE)
ORIGINAL_RE = re.compile(r"^original_detection_(.+)\.jpg$", re.IGNORECASE)


def scan_detections():
    """One entry per obstacle, newest first, flagged detected / not detected."""
    if not os.path.isdir(DETECTIONS_DIR):
        return []

    shots = {}
    for name in os.listdir(DETECTIONS_DIR):
        path = os.path.join(DETECTIONS_DIR, name)
        if not os.path.isfile(path):
            continue

        m_ann = ANNOTATED_RE.match(name)
        m_org = ORIGINAL_RE.match(name)
        if m_ann:
            obstacle, kind = m_ann.group(1), "annotated"
        elif m_org:
            obstacle, kind = m_org.group(1), "original"
        else:
            continue

        entry = shots.setdefault(obstacle, {"obstacle": obstacle})
        entry[kind] = name
        entry[kind + "_mtime"] = os.path.getmtime(path)

    out = []
    for obstacle, e in shots.items():
        ann_t = e.get("annotated_mtime", 0)
        org_t = e.get("original_mtime", 0)
        # The annotated file is only rewritten on a successful detection. If it
        # is older than the raw frame, the most recent snap found nothing.
        detected = "annotated" in e and ann_t >= org_t
        out.append(
            {
                "obstacle": obstacle,
                "detected": detected,
                "file": e["annotated"] if detected else e.get("original"),
                "mtime": max(ann_t, org_t),
            }
        )

    out = [o for o in out if o["file"]]
    out.sort(key=lambda o: o["mtime"], reverse=True)
    return out


@app.route("/api/shots")
def api_shots():
    shots = scan_detections()
    return jsonify(
        {
            "count": len(shots),
            "hits": sum(1 for s in shots if s["detected"]),
            "shots": shots,
        }
    )


@app.route("/img/<path:filename>")
def img(filename):
    return send_from_directory(DETECTIONS_DIR, filename)


@app.route("/")
def index():
    return send_from_directory(".", "live_view.html")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Live viewer for MDP detections.")
    p.add_argument("--port", type=int, default=4100)
    p.add_argument("--dir", default=DETECTIONS_DIR, help="detections folder to watch")
    args = p.parse_args()

    DETECTIONS_DIR = args.dir
    print(f" * Watching {os.path.abspath(DETECTIONS_DIR)}")
    print(f" * Open http://localhost:{args.port}")
    app.run(host="0.0.0.0", port=args.port, debug=False)
