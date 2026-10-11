#!/usr/bin/env python3
"""Convert a Blackbird dataset flight (MIT AERA quadrotor dataset) into a RiftPhys reality log.

Reads, without ROS installed:
  - rosbag.bag: a ROS1 bag (v2.0), topics /blackbird/state (geometry_msgs/PoseStamped, mocap
    pose) and /blackbird/rotor_rpm (blackbird/MotorRPM: Header, time[] sample_stamp,
    float32[] rpm). Chunks may be uncompressed or bz2 (lz4 needs the `lz4` module).
  - or the flight's CSVs: groundTruthPoses.csv + csv/blackbird_slash_rotor_rpm.csv
    (column layout detected from the header; check with --verbose).

Writes metres / seconds / y up. Blackbird's mocap frame is NED (x north, y east, z down) and its
body frame FRD; both map onto RiftPhys's (x forward, y up, z right) by a +90 degree rotation
about x: (x, y, z) -> (x, -z, y). Rotor speeds become `cmd` lines (rpm, rotor order as logged).

  python blackbird_to_reality.py bag  rosbag.bag --out flight.jsonl
  python blackbird_to_reality.py csv  <flight_dir> --out flight.jsonl
  python blackbird_to_reality.py selftest

Then: riftphys-fit --log flight.jsonl --scene crates/riftphys-reality/scenes/blackbird.json
(the scene's geometry is a placeholder until set from the Blackbird paper).
"""
import argparse
import bz2
import csv
import io
import json
import math
import os
import struct
import sys

VERSION = "0.1"
POSE_TOPIC = "/blackbird/state"
RPM_TOPIC = "/blackbird/rotor_rpm"

_S = math.sqrt(0.5)
NED_TO_YUP = (_S, 0.0, 0.0, _S)        # +90 degrees about x
NED_TO_YUP_INV = (-_S, 0.0, 0.0, _S)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def ned_to_yup(pos, quat_xyzw):
    x, y, z = pos
    q = qmul(qmul(NED_TO_YUP, quat_xyzw), NED_TO_YUP_INV)
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return (x, -z, y), tuple(c / n for c in q)


# ---------------------------------------------------------------- ROS1 bag v2.0

def _fields(buf):
    out, i = {}, 0
    while i + 4 <= len(buf):
        (n,) = struct.unpack_from("<I", buf, i)
        i += 4
        f = buf[i:i + n]
        i += n
        k, _, v = f.partition(b"=")
        out[k.decode()] = v
    return out


def _records(buf):
    i = 0
    while i + 4 <= len(buf):
        (hl,) = struct.unpack_from("<I", buf, i)
        header = _fields(buf[i + 4:i + 4 + hl])
        i += 4 + hl
        (dl,) = struct.unpack_from("<I", buf, i)
        data = buf[i + 4:i + 4 + dl]
        i += 4 + dl
        yield header, data


def _decompress(kind, data, size):
    if kind == "none":
        return data
    if kind == "bz2":
        return bz2.decompress(data)
    if kind == "lz4":
        try:
            import lz4.frame
        except ImportError:
            sys.exit("this bag uses lz4 chunks: pip install lz4 (into a separate environment)")
        return lz4.frame.decompress(data)
    sys.exit(f"unknown bag compression {kind}")


def read_bag(path, topics):
    """Yields (topic, type, raw message bytes) for every message on `topics`, in file order."""
    with open(path, "rb") as f:
        raw = f.read()
    magic = b"#ROSBAG V2.0\n"
    if not raw.startswith(magic):
        sys.exit(f"{path}: not a ROS1 v2.0 bag")
    conns = {}

    def handle(header, data):
        op = header.get("op", b"\x00")[0]
        if op == 0x07:  # connection
            info = _fields(data)
            conns[struct.unpack("<I", header["conn"])[0]] = (header["topic"].decode(), info.get("type", b"").decode())
        elif op == 0x02:  # message data
            c = conns.get(struct.unpack("<I", header["conn"])[0])
            if c and c[0] in topics:
                yield c[0], c[1], data
        elif op == 0x05:  # chunk
            inner = _decompress(header["compression"].decode(), data, struct.unpack("<I", header["size"])[0])
            for h, d in _records(inner):
                yield from handle(h, d)

    for header, data in _records(raw[len(magic):]):
        yield from handle(header, data)


class Reader:
    def __init__(self, b):
        self.b, self.i = b, 0

    def take(self, fmt):
        v = struct.unpack_from("<" + fmt, self.b, self.i)
        self.i += struct.calcsize("<" + fmt)
        return v

    def header(self):
        _seq, secs, nsecs = self.take("III")
        (n,) = self.take("I")
        self.i += n
        return secs + nsecs * 1e-9


def parse_pose_stamped(b):
    r = Reader(b)
    t = r.header()
    px, py, pz, qx, qy, qz, qw = r.take("7d")
    return t, (px, py, pz), (qx, qy, qz, qw)


def parse_motor_rpm(b):
    r = Reader(b)
    t = r.header()
    (n,) = r.take("I")
    r.i += 8 * n  # sample_stamp: time[] (secs, nsecs)
    (m,) = r.take("I")
    return t, list(r.take(f"{m}f"))


def from_bag(path):
    poses, rpms = [], []
    for topic, _type, data in read_bag(path, {POSE_TOPIC, RPM_TOPIC}):
        if topic == POSE_TOPIC:
            poses.append(parse_pose_stamped(data))
        else:
            rpms.append(parse_motor_rpm(data))
    return poses, rpms


# ---------------------------------------------------------------- CSVs

def _time_scale(ts):
    """Seconds per unit, chosen so the median sample spacing is between 0.1 ms and 0.1 s."""
    d = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
    if not d:
        return 1.0
    med = d[len(d) // 2]
    for s in (1.0, 1e-3, 1e-6, 1e-9):
        if 1e-4 <= med * s <= 0.1:
            return s
    return 1.0


def _find(cols, *names):
    low = [c.strip().lower() for c in cols]
    for n in names:
        if n in low:
            return low.index(n)
    return None


def from_csv(flight_dir, verbose):
    gt = os.path.join(flight_dir, "groundTruthPoses.csv")
    rp = os.path.join(flight_dir, "csv", "blackbird_slash_rotor_rpm.csv")
    rows = list(csv.reader(open(gt, newline="")))
    head = [c.lstrip("#").strip() for c in rows[0]]
    numeric = all(_is_num(c) for c in head if c)
    if numeric:
        # No header: assume time, px, py, pz, qw, qx, qy, qz (LCM order)
        cols = {"t": 0, "px": 1, "py": 2, "pz": 3, "qw": 4, "qx": 5, "qy": 6, "qz": 7}
        data = rows
        print("groundTruthPoses.csv has no header: assuming time,px,py,pz,qw,qx,qy,qz - CHECK", file=sys.stderr)
    else:
        cols = {
            "t": _find(head, "timestamp", "time", "t", "utime", "stamp"),
            "px": _find(head, "px", "p_x", "x", "position.x", "pos_x"),
            "py": _find(head, "py", "p_y", "y", "position.y", "pos_y"),
            "pz": _find(head, "pz", "p_z", "z", "position.z", "pos_z"),
            "qw": _find(head, "qw", "q_w", "w", "orientation.w"),
            "qx": _find(head, "qx", "q_x", "orientation.x"),
            "qy": _find(head, "qy", "q_y", "orientation.y"),
            "qz": _find(head, "qz", "q_z", "orientation.z"),
        }
        if cols["t"] is None:
            cols["t"] = 0
        missing = [k for k, v in cols.items() if v is None]
        if missing:
            sys.exit(f"{gt}: could not find columns {missing} in header {head}")
        data = rows[1:]
    if verbose:
        print(f"{gt}: columns {cols}")
    raw = [(float(r[cols["t"]]), r) for r in data if r and _is_num(r[cols["t"]])]
    s = _time_scale([t for t, _ in raw])
    poses = [(t * s, (float(r[cols["px"]]), float(r[cols["py"]]), float(r[cols["pz"]])),
              (float(r[cols["qx"]]), float(r[cols["qy"]]), float(r[cols["qz"]]), float(r[cols["qw"]]))) for t, r in raw]

    rows = list(csv.reader(open(rp, newline="")))
    head = [c.strip() for c in rows[0]]
    ci = _find(head, "rpm")
    si, ni, bi = _find(head, "secs"), _find(head, "nsecs"), _find(head, "rosbagtimestamp")
    if ci is None:
        sys.exit(f"{rp}: no rpm column in {head}")
    rpms = []
    for r in rows[1:]:
        try:
            t = int(r[si]) + int(r[ni]) * 1e-9 if si is not None and ni is not None else int(r[bi]) * 1e-9
            vals = [float(v) for v in r[ci].strip("[] ").split(",") if v.strip()]
        except (ValueError, IndexError, TypeError):
            continue
        rpms.append((t, vals))
    if verbose:
        print(f"{rp}: rpm column {ci}, time from {'secs/nsecs' if si is not None else 'rosbagTimestamp'}")
    return poses, rpms


def _is_num(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# ---------------------------------------------------------------- write

def write(poses, rpms, out, source, notes, body="blackbird"):
    if not poses:
        sys.exit("no poses found")
    t0 = min([p[0] for p in poses] + [r[0] for r in rpms])
    lines = []
    for t, pos, q in poses:
        p, qr = ned_to_yup(pos, q)
        lines.append({"t": "pose", "time": round(t - t0, 6), "body": body, "pos": [round(c, 6) for c in p], "rot": [round(c, 7) for c in qr]})
    for t, vals in rpms:
        lines.append({"t": "cmd", "time": round(t - t0, 6), "who": body, "act": [round(v, 2) for v in vals]})
    lines.sort(key=lambda l: l["time"])
    span = poses[-1][0] - poses[0][0]
    rate = round((len(poses) - 1) / span, 1) if span > 0 else 0.0
    meta = {"t": "meta", "v": VERSION, "source": source, "rate_hz": rate, "bodies": [body], "notes": notes}
    with open(out, "w", newline="\n") as f:
        f.write(json.dumps(meta) + "\n")
        for l in lines:
            f.write(json.dumps(l) + "\n")
    print(f"wrote {out}: {len(poses)} poses (~{rate} Hz), {len(rpms)} rotor-speed samples, {span:.1f} s")


# ---------------------------------------------------------------- self-test

def _bag_record(header_fields, data):
    h = b"".join(struct.pack("<I", len(k) + 1 + len(v)) + k + b"=" + v for k, v in header_fields)
    return struct.pack("<I", len(h)) + h + struct.pack("<I", len(data)) + data


def _header_bytes(t, frame=b"mocap"):
    secs = int(t)
    return struct.pack("<III", 0, secs, int(round((t - secs) * 1e9))) + struct.pack("<I", len(frame)) + frame


def selftest():
    import tempfile
    d = tempfile.mkdtemp()
    path = os.path.join(d, "rosbag.bag")
    def conn(c, topic, typ):
        info = b"".join(struct.pack("<I", len(k) + 1 + len(v)) + k + b"=" + v for k, v in [(b"topic", topic), (b"type", typ), (b"md5sum", b"0")])
        return _bag_record([(b"op", b"\x07"), (b"conn", struct.pack("<I", c)), (b"topic", topic)], info)

    msgs_a, msgs_b = [], []
    for i in range(10):
        t = 1600000000.0 + i / 360.0
        # NED: hovering 1.5 m up (z = -1.5), yawed 90 degrees about down (z_ned)
        s = math.sin(math.pi / 4)
        pose = _header_bytes(t) + struct.pack("<7d", 0.5, 0.25, -1.5, 0.0, 0.0, s, s)
        msgs_a.append(_bag_record([(b"op", b"\x02"), (b"conn", struct.pack("<I", 0)), (b"time", struct.pack("<II", int(t), 0))], pose))
        rpm = _header_bytes(t) + struct.pack("<I", 4) + struct.pack("<8I", *([int(t), 0] * 4)) + struct.pack("<I", 4) + struct.pack("<4f", 15000, 15100, 15200, 15300)
        msgs_b.append(_bag_record([(b"op", b"\x02"), (b"conn", struct.pack("<I", 1)), (b"time", struct.pack("<II", int(t), 0))], rpm))
    chunk1 = conn(0, POSE_TOPIC.encode(), b"geometry_msgs/PoseStamped") + b"".join(msgs_a[:5])
    chunk2 = conn(1, RPM_TOPIC.encode(), b"blackbird/MotorRPM") + b"".join(msgs_b) + b"".join(msgs_a[5:])
    body = _bag_record([(b"op", b"\x03")], b"")
    body += _bag_record([(b"op", b"\x05"), (b"compression", b"none"), (b"size", struct.pack("<I", len(chunk1)))], chunk1)
    body += _bag_record([(b"op", b"\x05"), (b"compression", b"bz2"), (b"size", struct.pack("<I", len(chunk2)))], bz2.compress(chunk2))
    with open(path, "wb") as f:
        f.write(b"#ROSBAG V2.0\n" + body)
    poses, rpms = from_bag(path)
    assert len(poses) == 10 and len(rpms) == 10, (len(poses), len(rpms))
    assert rpms[0][1] == [15000, 15100, 15200, 15300]
    out = os.path.join(d, "flight.jsonl")
    write(poses, rpms, out, "blackbird", "selftest")
    lines = [json.loads(l) for l in open(out)]
    p = next(l for l in lines if l["t"] == "pose")
    # NED (0.5 N, 0.25 E, -1.5 D) -> y-up (x 0.5, y 1.5 up, z 0.25)
    assert all(abs(a - b) < 1e-6 for a, b in zip(p["pos"], [0.5, 1.5, 0.25])), p["pos"]
    # A yaw about NED down is a rotation about RiftPhys -y: body forward (x) turns toward +z (east)
    qx, qy, qz, qw = p["rot"]
    fwd = (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy))
    assert abs(fwd[2] - 1.0) < 1e-6, fwd
    assert abs(lines[0]["rate_hz"] - 360.0) < 1.0, lines[0]
    print("selftest ok (bag with plain + bz2 chunks, NED -> y-up)")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    b = sub.add_parser("bag")
    b.add_argument("bag")
    b.add_argument("--out", required=True)
    c = sub.add_parser("csv")
    c.add_argument("flight_dir")
    c.add_argument("--out", required=True)
    c.add_argument("--verbose", action="store_true")
    sub.add_parser("selftest")
    a = ap.parse_args()
    if a.mode == "bag":
        poses, rpms = from_bag(a.bag)
        write(poses, rpms, a.out, "blackbird", f"Blackbird bag {os.path.basename(a.bag)}")
    elif a.mode == "csv":
        poses, rpms = from_csv(a.flight_dir, a.verbose)
        write(poses, rpms, a.out, "blackbird", f"Blackbird CSVs {a.flight_dir}")
    else:
        selftest()


if __name__ == "__main__":
    main()
