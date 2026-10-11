#!/usr/bin/env python3
"""Convert motion-capture exports into RiftPhys reality logs (crates/riftphys-reality, log.rs).

Output: JSON lines in RiftPhys conventions - metres, seconds, y up, quaternions [x, y, z, w]
(body to world). A `break` event is written wherever samples stop for longer than --break-gap
(a lost track, the end of a take) and between input files, so replay windows never span them.

  OptiTrack Motive CSV export (rigid bodies, quaternion rotation):
    python mocap_to_reality.py motive take1.csv take2.csv --out balls.jsonl

  Generic CSV (libsurvive, Crazyflie cfclient logs, anything with columns):
    python mocap_to_reality.py csv flight.csv --body cf1 --time timestamp --time-scale 0.001 \
        --pos stateEstimate.x,stateEstimate.y,stateEstimate.z --up z --out cf.jsonl
    (optional --quat qx,qy,qz,qw; --units mm; --cmd motor.m1,motor.m2,motor.m3,motor.m4 --cmd-scale 1.52590219e-5)

  python mocap_to_reality.py selftest

Motive's export layout (header rows naming each column's body, quantity and axis) is parsed by
content, not position, and was written from its documented format: check the first real
export (`--verbose` prints what each column was read as).
"""
import argparse
import csv
import json
import math
import sys

VERSION = "0.1"


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# Rotation of -90 degrees about x: maps a z-up frame onto y-up ((x, y, z) -> (x, z, -y))
_S = math.sqrt(0.5)
Z_TO_Y = (-_S, 0.0, 0.0, _S)
Z_TO_Y_INV = (_S, 0.0, 0.0, _S)


def to_y_up(pos, quat, up):
    if up == "y":
        return pos, quat
    x, y, z = pos
    p = (x, z, -y)
    q = qmul(qmul(Z_TO_Y, quat), Z_TO_Y_INV) if quat is not None else None
    return p, q


def normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 1e-9 else None


class Writer:
    def __init__(self, source):
        self.source = source
        self.lines = []
        self.bodies = []
        self.rate = 0.0
        self.offset = 0.0
        self.last_t = None
        self.last_body_t = {}

    def pose(self, t, body, pos, quat, gap):
        t += self.offset
        if body not in self.bodies:
            self.bodies.append(body)
        prev = self.last_body_t.get(body)
        if prev is not None and t - prev > gap:
            self.event(prev + 0.5 * (t - prev), "break")
        self.last_body_t[body] = t
        q = normalize(quat) if quat is not None else (0.0, 0.0, 0.0, 1.0)
        if q is None:
            return
        self.lines.append({"t": "pose", "time": round(t, 6), "body": body, "pos": [round(c, 6) for c in pos], "rot": [round(c, 6) for c in q]})
        self.last_t = t if self.last_t is None else max(self.last_t, t)

    def cmd(self, t, who, act):
        self.lines.append({"t": "cmd", "time": round(t + self.offset, 6), "who": who, "act": act})

    def event(self, t, e):
        self.lines.append({"t": "event", "time": round(t, 6), "e": e})

    def next_file(self):
        """Later files continue 1 s after the previous one ends, separated by a break."""
        if self.last_t is not None:
            self.event(self.last_t + 0.5, "break")
            self.offset = self.last_t + 1.0
            self.last_body_t = {}

    def save(self, path, notes):
        meta = {"t": "meta", "v": VERSION, "source": self.source, "rate_hz": self.rate, "bodies": self.bodies, "notes": notes}
        self.lines.sort(key=lambda l: l["time"])
        with open(path, "w", newline="\n") as f:
            f.write(json.dumps(meta) + "\n")
            for l in self.lines:
                f.write(json.dumps(l) + "\n")
        print(f"wrote {path}: {len(self.lines)} lines, bodies {self.bodies}, {self.rate} Hz")


def motive(paths, out, gap, verbose):
    w = Writer("optitrack")
    for path in paths:
        w.next_file()
        with open(path, newline="", encoding="utf-8-sig") as f:
            rows = list(csv.reader(f))
        meta = rows[0]
        info = {meta[i].strip(): meta[i + 1].strip() for i in range(0, len(meta) - 1, 2)}
        if info.get("Rotation Type", "Quaternion") != "Quaternion":
            sys.exit(f"{path}: export rotations as Quaternion (got {info.get('Rotation Type')})")
        scale = {"Meters": 1.0, "Centimeters": 0.01, "Millimeters": 0.001}.get(info.get("Length Units", "Meters"))
        if scale is None:
            sys.exit(f"{path}: unknown Length Units {info.get('Length Units')}")
        rate = float(info.get("Export Frame Rate") or info.get("Capture Frame Rate") or 0)
        w.rate = w.rate or rate
        axis_row = next(i for i, r in enumerate(rows) if r and r[0].strip() == "Frame")
        type_row = next(i for i, r in enumerate(rows[:axis_row]) if any(c.strip() == "Rigid Body" for c in r))
        types, names, quantities, axes = rows[type_row], rows[type_row + 1], rows[axis_row - 1], rows[axis_row]
        time_col = next(j for j, c in enumerate(axes) if c.strip().startswith("Time"))
        cols = {}  # body -> {("Rotation","X"): j, ...}
        for j in range(len(axes)):
            if j < len(types) and types[j].strip() == "Rigid Body" and quantities[j].strip() in ("Rotation", "Position"):
                cols.setdefault(names[j].strip(), {})[(quantities[j].strip(), axes[j].strip())] = j
        if verbose:
            for b, c in cols.items():
                print(f"{path}: body {b!r} columns {sorted(c.items(), key=lambda kv: kv[1])}")
        need = [("Position", a) for a in "XYZ"] + [("Rotation", a) for a in "XYZW"]
        for b, c in cols.items():
            missing = [k for k in need if k not in c]
            if missing:
                sys.exit(f"{path}: body {b} lacks columns {missing}")
        for r in rows[axis_row + 1:]:
            if len(r) <= time_col or not r[time_col].strip():
                continue
            t = float(r[time_col])
            for b, c in cols.items():
                try:
                    pos = tuple(float(r[c[("Position", a)]]) * scale for a in "XYZ")
                    quat = tuple(float(r[c[("Rotation", a)]]) for a in "XYZW")
                except (ValueError, IndexError):
                    continue  # occluded this frame
                w.pose(t, b, pos, quat, gap)
    w.save(out, f"Motive CSV: {', '.join(paths)}")


def generic(paths, out, a):
    w = Writer(a.source)
    w.rate = a.rate
    scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[a.units]
    pos_cols = a.pos.split(",")
    quat_cols = a.quat.split(",") if a.quat else None
    cmd_cols = a.cmd.split(",") if a.cmd else None
    for path in paths:
        w.next_file()
        with open(path, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                try:
                    t = float(r[a.time]) * a.time_scale
                    pos = tuple(float(r[c]) * scale for c in pos_cols)
                    quat = tuple(float(r[c]) for c in quat_cols) if quat_cols else None
                except (ValueError, KeyError, TypeError):
                    continue
                p, q = to_y_up(pos, quat, a.up)
                w.pose(t, a.body, p, q, a.break_gap)
                if cmd_cols:
                    try:
                        w.cmd(t, a.body, [float(r[c]) * a.cmd_scale for c in cmd_cols])
                    except (ValueError, KeyError, TypeError):
                        pass
    w.save(out, f"generic CSV ({a.source}): {', '.join(paths)}")


def selftest():
    import os
    import tempfile
    d = tempfile.mkdtemp()
    # A Motive-style export: a ball in mm falling for 0.1 s, then lost for 0.5 s, then seen again
    path = os.path.join(d, "take.csv")
    with open(path, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["Format Version", "1.23", "Take Name", "test", "Capture Frame Rate", "100.000000", "Export Frame Rate", "100",
                     "Rotation Type", "Quaternion", "Length Units", "Millimeters", "Coordinate Space", "Global"])
        cw.writerow([])
        cw.writerow(["", "Type"] + ["Rigid Body"] * 8)
        cw.writerow(["", "Name"] + ["ball"] * 8)
        cw.writerow(["", "ID"] + ["1"] * 8)
        cw.writerow(["", ""] + ["Rotation"] * 4 + ["Position"] * 3 + ["Mean Marker Error"])
        cw.writerow(["Frame", "Time (Seconds)", "X", "Y", "Z", "W", "X", "Y", "Z", ""])
        for i in list(range(11)) + list(range(60, 66)):
            t = i * 0.01
            cw.writerow([i, f"{t:.6f}", 0, 0, 0, 1, 0, f"{1000 - 4905 * t * t:.4f}", 0, 0.1])
    out = os.path.join(d, "out.jsonl")
    motive([path], out, gap=0.1, verbose=False)
    lines = [json.loads(l) for l in open(out)]
    poses = [l for l in lines if l["t"] == "pose"]
    breaks = [l for l in lines if l["t"] == "event"]
    assert lines[0]["t"] == "meta" and lines[0]["bodies"] == ["ball"] and lines[0]["rate_hz"] == 100.0
    assert len(poses) == 17 and abs(poses[0]["pos"][1] - 1.0) < 1e-9, poses[0]
    assert len(breaks) == 1 and 0.1 < breaks[0]["time"] < 0.6, breaks
    # z-up conversion: a point 1 m up in z becomes 1 m up in y; identity rotation stays identity
    p, q = to_y_up((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), "z")
    assert abs(p[1] - 1.0) < 1e-9 and abs(q[3] - 1.0) < 1e-9, (p, q)
    # a yaw about z-up becomes a yaw about y-up
    s = math.sin(0.25)
    _, q = to_y_up((0, 0, 0), (0.0, 0.0, s, math.cos(0.25)), "z")
    assert abs(q[1] - s) < 1e-9 and abs(q[0]) < 1e-9 and abs(q[2]) < 1e-9, q
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    m = sub.add_parser("motive")
    m.add_argument("files", nargs="+")
    m.add_argument("--out", required=True)
    m.add_argument("--break-gap", type=float, default=0.1)
    m.add_argument("--verbose", action="store_true")
    g = sub.add_parser("csv")
    g.add_argument("files", nargs="+")
    g.add_argument("--out", required=True)
    g.add_argument("--body", required=True)
    g.add_argument("--time", required=True)
    g.add_argument("--time-scale", type=float, default=1.0)
    g.add_argument("--pos", required=True, help="x,y,z column names")
    g.add_argument("--quat", help="qx,qy,qz,qw column names")
    g.add_argument("--cmd", help="command column names, in actuator order")
    g.add_argument("--cmd-scale", type=float, default=1.0)
    g.add_argument("--units", choices=["m", "cm", "mm"], default="m")
    g.add_argument("--up", choices=["y", "z"], default="y")
    g.add_argument("--rate", type=float, default=0.0)
    g.add_argument("--source", default="csv")
    g.add_argument("--break-gap", type=float, default=0.1)
    sub.add_parser("selftest")
    a = ap.parse_args()
    if a.mode == "motive":
        motive(a.files, a.out, a.break_gap, a.verbose)
    elif a.mode == "csv":
        generic(a.files, a.out, a)
    else:
        selftest()


if __name__ == "__main__":
    main()
