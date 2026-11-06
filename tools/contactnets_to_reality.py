#!/usr/bin/env python3
"""Convert the ContactNets cube-toss dataset into a RiftPhys reality log (crates/riftphys-reality).

The dataset (DAIRLab/contact-nets, BSD-3-Clause; Pfrommer, Halm & Posa, CoRL 2020) holds 570 real
tosses of a 10.48 cm acrylic cube onto a table, tracked with AprilTags at 148 Hz. Acosta, Yang &
Posa ("Validating Robotics Simulators on Real-World Impacts", RA-L 2022) fit Drake, MuJoCo and
Bullet to the same tosses, so RiftPhys can be scored on a published scale.

Each `data/tosses_processed/<n>.pt` is one toss saved with PyTorch's legacy (pre-1.6) format: a
tensor (1, frames, 19) of rows position (3), quaternion (4, real part first), velocity (3), angular
velocity (3), control (6, zeros). Positions and velocities are in cube half-widths (0.0524 m); z
is up.

The files are pickles. They are read here with a restricted unpickler that only rebuilds tensors
from raw storage and refuses every other global, so a file cannot run code. torch is not needed.

Output (metres, seconds, y up, quaternions [x, y, z, w]): one body `cube`, a `break` event between
tosses, and at each toss's first frame an event `state` whose data holds the toss index, the
measured velocity and angular velocity (world frame, y up).

  python contactnets_to_reality.py DATA_DIR --out cube_tosses.jsonl
  python contactnets_to_reality.py DATA_DIR --check     (prints frame conventions, writes nothing)
"""
import argparse
import array
import io
import json
import math
import os
import pickle
import struct
import sys

HALF_WIDTH = 0.0524
RATE_HZ = 148.0
LEGACY_MAGIC = 0x1950A86A20F9469CFC6C
STORAGE_TYPES = {"DoubleStorage": ("d", 8), "FloatStorage": ("f", 4)}


class _Storage:
    def __init__(self, kind, numel):
        self.kind, self.numel, self.data = kind, numel, None


class _Tensor:
    def __init__(self, storage, offset, size, stride):
        self.storage, self.offset, self.size, self.stride = storage, offset, size, stride

    def rows(self):
        """The tensor as a list of rows (last dimension), leading dimensions flattened."""
        *lead, cols = self.size
        n = 1
        for d in lead:
            n *= d
        out = []
        for i in range(n):
            # Index of row i in the leading dimensions
            idx, rem = [], i
            for d in reversed(lead):
                idx.append(rem % d)
                rem //= d
            idx.reverse()
            base = self.offset + sum(k * s for k, s in zip(idx, self.stride[:-1]))
            out.append([self.storage.data[base + c * self.stride[-1]] for c in range(cols)])
        return out


def _rebuild_tensor_v2(storage, offset, size, stride, requires_grad=False, hooks=None, *rest):
    return _Tensor(storage, offset, tuple(size), tuple(stride))


class _Ordered(dict):
    pass


class SafeUnpickler(pickle.Unpickler):
    """Allows only what a saved tensor needs; anything else in the file is refused."""

    def __init__(self, f, storages):
        super().__init__(f)
        self.storages = storages

    def find_class(self, module, name):
        if (module, name) == ("torch._utils", "_rebuild_tensor_v2"):
            return _rebuild_tensor_v2
        if module == "torch" and name in STORAGE_TYPES:
            return name
        if (module, name) == ("collections", "OrderedDict"):
            return _Ordered
        raise pickle.UnpicklingError(f"refused global {module}.{name}")

    def persistent_load(self, pid):
        if not (isinstance(pid, tuple) and pid and pid[0] == "storage"):
            raise pickle.UnpicklingError(f"refused persistent id {pid!r}")
        _, kind, key, _location, numel = pid[:5]
        if kind not in STORAGE_TYPES:
            raise pickle.UnpicklingError(f"refused storage type {kind!r}")
        if key not in self.storages:
            self.storages[key] = _Storage(kind, numel)
        return self.storages[key]


class _Plain(pickle.Unpickler):
    def find_class(self, module, name):
        raise pickle.UnpicklingError(f"refused global {module}.{name}")


def load_legacy_pt(path):
    with open(path, "rb") as fh:
        f = io.BytesIO(fh.read())
    if _Plain(f).load() != LEGACY_MAGIC:
        raise ValueError(f"{path}: not a legacy PyTorch file")
    _Plain(f).load()  # protocol version
    _Plain(f).load()  # sys info
    storages = {}
    obj = SafeUnpickler(f, storages).load()
    keys = _Plain(f).load()
    for key in keys:
        st = storages[key]
        code, size = STORAGE_TYPES[st.kind]
        (numel,) = struct.unpack("<q", f.read(8))
        data = array.array(code)
        data.frombytes(f.read(numel * size))
        if sys.byteorder != "little":
            data.byteswap()
        st.data = data
    if not isinstance(obj, _Tensor):
        raise ValueError(f"{path}: expected one tensor")
    return obj.rows()


def to_y_up_vec(v):
    x, y, z = v
    return [x, z, -y]


def to_y_up_quat_wxyz(q):
    """ContactNets quaternion (w, x, y, z), z up -> RiftPhys [x, y, z, w], y up."""
    w, x, y, z = q
    s = math.sqrt(0.5)
    # R = Rx(-90) * q * Rx(-90)^-1, with Rx(-90) = (x: -s, w: s)
    def mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)
    r = mul(mul((-s, 0.0, 0.0, s), (x, y, z, w)), (s, 0.0, 0.0, s))
    n = math.sqrt(sum(c * c for c in r))
    return [c / n for c in r]


def qrot(q, v):
    """Rotate v by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)]


def frame_check(files):
    """Compares the stored angular velocity with the quaternion's finite difference in the world
    and in the body frame, and the stored velocity with the position's finite difference."""
    errs = {"world": 0.0, "body": 0.0, "vel": 0.0, "rest_z": []}
    n = 0
    for path in files:
        rows = load_legacy_pt(path)
        errs["rest_z"].append(rows[-1][2])
        for a, b in zip(rows[5:40], rows[6:41]):
            qa, qb = a[3:7], b[3:7]
            if sum(x * y for x, y in zip(qa, qb)) < 0:
                qb = [-c for c in qb]
            dq = [(y - x) * RATE_HZ for x, y in zip(qa, qb)]
            qm = [(x + y) / 2 for x, y in zip(qa, qb)]
            w, x, y, z = qm
            conj = (w, -x, -y, -z)
            def mul(p, r):
                pw, px, py, pz = p
                rw, rx, ry, rz = r
                return (pw * rw - px * rx - py * ry - pz * rz, pw * rx + px * rw + py * rz - pz * ry,
                        pw * ry - px * rz + py * rw + pz * rx, pw * rz + px * ry - py * rx + pz * rw)
            ww = [2 * c for c in mul(dq, conj)[1:]]
            wb = [2 * c for c in mul(conj, dq)[1:]]
            om = [(p + q) / 2 for p, q in zip(a[10:13], b[10:13])]
            errs["world"] += sum((p - q) ** 2 for p, q in zip(ww, om))
            errs["body"] += sum((p - q) ** 2 for p, q in zip(wb, om))
            vfd = [(q - p) * RATE_HZ for p, q in zip(a[0:3], b[0:3])]
            vm = [(p + q) / 2 for p, q in zip(a[7:10], b[7:10])]
            errs["vel"] += sum((p - q) ** 2 for p, q in zip(vfd, vm))
            n += 1
    for k in ("world", "body", "vel"):
        errs[k] = math.sqrt(errs[k] / n)
    return errs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir", help="the contact-nets repository's data/ folder (or tosses_processed/ itself)")
    ap.add_argument("--out")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--omega-frame", choices=["world", "body"], default=None,
                    help="frame of the stored angular velocity (default: detected with --check's test)")
    args = ap.parse_args()

    d = args.data_dir
    if os.path.isdir(os.path.join(d, "tosses_processed")):
        d = os.path.join(d, "tosses_processed")
    files = sorted((os.path.join(d, f) for f in os.listdir(d) if f.endswith(".pt")),
                   key=lambda p: int(os.path.splitext(os.path.basename(p))[0]))
    if not files:
        sys.exit(f"no .pt files in {d}")

    chk = frame_check(files[:40])
    detected = "world" if chk["world"] < chk["body"] else "body"
    if args.check or not args.out:
        rz = sorted(chk["rest_z"])
        print(f"{len(files)} tosses. Angular velocity vs quaternion finite difference, RMS (rad/s):")
        print(f"  world frame {chk['world']:.3f}, body frame {chk['body']:.3f} -> stored in the {detected} frame")
        print(f"  velocity vs position finite difference RMS {chk['vel']:.3f} (half-widths/s)")
        print(f"  final height (half-widths): median {rz[len(rz) // 2]:.3f}, range {rz[0]:.3f}..{rz[-1]:.3f}  (1.0 = resting on a face, z up)")
        return
    frame = args.omega_frame or detected

    out = open(args.out, "w", encoding="utf-8", newline="\n")
    meta = {"t": "meta", "v": "0.1", "source": "contactnets", "rate_hz": RATE_HZ, "bodies": ["cube"],
            "notes": f"DAIRLab/contact-nets data/tosses_processed ({len(files)} tosses, BSD-3-Clause); "
                     f"acrylic cube, half-width {HALF_WIDTH} m, mass 0.37 kg; angular velocity stored in the {frame} frame"}
    out.write(json.dumps(meta) + "\n")
    t = 0.0
    dt = 1.0 / RATE_HZ
    frames = 0
    for i, path in enumerate(files):
        rows = load_legacy_pt(path)
        if i > 0:
            out.write(json.dumps({"t": "event", "time": round(t, 6), "e": "break"}) + "\n")
            t += 1.0
        first = rows[0]
        q0 = first[3:7]
        om = first[10:13]
        if frame == "body":
            om = qrot(q0, om)
        state = {"toss": int(os.path.splitext(os.path.basename(path))[0]),
                 "vel": [round(c * HALF_WIDTH, 6) for c in to_y_up_vec(first[7:10])],
                 "ang": [round(c, 6) for c in to_y_up_vec(om)]}
        out.write(json.dumps({"t": "event", "time": round(t, 6), "e": "state", "data": state}) + "\n")
        for k, r in enumerate(rows):
            pos = [round(c * HALF_WIDTH, 6) for c in to_y_up_vec(r[0:3])]
            rot = [round(c, 7) for c in to_y_up_quat_wxyz(r[3:7])]
            out.write(json.dumps({"t": "pose", "time": round(t + k * dt, 6), "body": "cube", "pos": pos, "rot": rot}) + "\n")
        t += len(rows) * dt
        frames += len(rows)
    out.close()
    print(f"{len(files)} tosses, {frames} frames -> {args.out} (angular velocity: {frame} frame)")


if __name__ == "__main__":
    main()
