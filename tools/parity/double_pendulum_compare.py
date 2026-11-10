#!/usr/bin/env python3
"""Compares riftphys-double-pendulum with MuJoCo on the real double pendulum (PARITY_SPEC 3.3,
"Method for the final numbers"), from their --dump and --energy files.

Per horizon: the mean paired difference RiftPhys - MuJoCo over the same held-out windows, with a
95% block-bootstrap interval (blocks of 10 consecutive windows within a trial), and each held-out
trial alone. Energy: each run's energy above rest against the measured, from the reality log.

    python double_pendulum_compare.py FITS_DIR LOG.jsonl [--tag h4_e200]

FITS_DIR holds <engine>_<tag>_s<seed>.{txt,csv} and energy_<engine>_<tag>_s<seed>.csv. Each engine's
headline run is its seed with the lowest training objective.

Needs: numpy.
"""
import argparse
import glob
import json
import math
import os
import re

import numpy as np

HORIZONS = (0.1, 0.25, 0.5, 1.0)
BLOCK = 10
RESAMPLES = 10000
HELD = (1, 3)
TIMES = (5, 10, 20, 30, 45, 60)
G = 9.81
HALF = 5  # +-10 ms at 500 Hz


def runs(root, engine, tag):
    out = []
    for path in sorted(glob.glob(os.path.join(root, f"{engine}_{tag}_s*.txt"))):
        text = open(path, encoding="utf-8").read()
        obj = float(re.search(r"training objective ([0-9.eE+-]+)", text).group(1))
        params = re.search(r"params (\S+)", text).group(1)
        held = re.search(r"\(held\)\s+\d+ windows \| deg: (.*)", text).group(1)
        seed = int(re.search(r"_s(\d+)\.txt$", path).group(1))
        out.append({"seed": seed, "objective": obj, "params": params, "held": held, "csv": path[:-4] + ".csv"})
    return out


def load_errors(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        next(f)
        for line in f:
            v = line.strip().split(",")
            rows.append((int(v[0]), float(v[1]), [float(x) for x in v[2:6]]))
    return rows


def block_ci(diffs_by_trial, rng):
    """95% interval of the mean, resampling blocks of consecutive windows within each trial."""
    blocks = []
    for d in diffs_by_trial:
        for i in range(0, len(d), BLOCK):
            b = d[i:i + BLOCK]
            if len(b) > 0:
                blocks.append(b)
    n = len(blocks)
    means = np.empty(RESAMPLES)
    for r in range(RESAMPLES):
        pick = rng.integers(0, n, n)
        means[r] = np.concatenate([blocks[i] for i in pick]).mean()
    return np.percentile(means, 2.5), np.percentile(means, 97.5)


def paired(a, b, name):
    print(f"\nRiftPhys minus {name}, degrees (held-out trials {HELD}):")
    rng = np.random.default_rng(1)
    # Times are compared loosely: the engines parse the log's decimal times to the nearest ulp or so
    assert len(a) == len(b) and all(ra[0] == rb[0] and abs(ra[1] - rb[1]) < 1e-6 for ra, rb in zip(a, b)), "the two runs have different windows"
    for k, h in enumerate(HORIZONS):
        by = {}
        for ra, rb in zip(a, b):
            if ra[0] in HELD:
                by.setdefault(ra[0], []).append(math.degrees(ra[2][k] - rb[2][k]))
        arrays = [np.array(by[t]) for t in HELD]
        all_d = np.concatenate(arrays)
        lo, hi = block_ci(arrays, rng)
        each = "; ".join(f"trial {t} {np.array(by[t]).mean():+.2f}" for t in HELD)
        verdict = "no worse" if lo <= 0 else "WORSE"
        print(f"  {h:.2f} s: {all_d.mean():+.2f} [{lo:+.2f}, {hi:+.2f}] {verdict} | {each} | n {len(all_d)}")


def measured_energy(path):
    rig, rate, starts, tracks = None, None, {}, {"upper": [], "lower": []}
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            if o["t"] == "meta":
                rate = o["rate_hz"]
            elif o["t"] == "pose":
                q = o["rot"]
                tracks[o["body"]].append((o["time"], 2.0 * math.atan2(q[2], q[3])))
            elif o["t"] == "event" and o["e"] == "rig":
                rig = o["data"]
            elif o["t"] == "event" and o["e"] == "state":
                starts.setdefault(o["data"]["trial"], o["time"])
    t = np.array([x[0] for x in tracks["upper"]])
    p1 = np.unwrap(np.array([x[1] for x in tracks["upper"]]))
    p2 = np.unwrap(np.array([x[1] for x in tracks["lower"]]))
    x = np.arange(HALF, -HALF - 1, -1) / rate
    kernel = x / (x * x).sum()
    u, l, ll = rig["upper"], rig["lower"], rig["l"]
    out = {}
    for trial, t0 in starts.items():
        i0 = int(np.searchsorted(t, t0 - 1e-6))
        i1 = min(len(t), i0 + int(61 * rate))
        a, b = p1[i0 - HALF:i1 + HALF], p2[i0 - HALF:i1 + HALF]
        w1, w2 = np.convolve(a, kernel, mode="valid"), np.convolve(b, kernel, mode="valid")
        a, b = a[HALF:-HALF], b[HALF:-HALF]
        vx = ll * np.cos(a) * w1 + l["c"] * np.cos(b) * w2
        vy = ll * np.sin(a) * w1 + l["c"] * np.sin(b) * w2
        e = 0.5 * u["m"] * (u["c"] * w1) ** 2 + 0.5 * u["i"] * w1 ** 2 + 0.5 * l["m"] * (vx * vx + vy * vy) + 0.5 * l["i"] * w2 ** 2
        e += u["m"] * G * u["c"] * (1 - np.cos(a)) + l["m"] * G * (ll * (1 - np.cos(a)) + l["c"] * (1 - np.cos(b)))
        window = int(rate)
        smooth = np.convolve(e, np.ones(window) / window, mode="same")
        out[trial] = (t[i0:i0 + len(e)] - t0, smooth, float(np.mean(e[:25])))
    return out


def milestones(t, e, e0):
    at = []
    for s in TIMES:
        i = int(np.searchsorted(t, s))
        at.append(float(e[min(i, len(e) - 1)]))
    def when(frac):
        # From 0.5 s: the moving mean of the measured energy starts half empty
        below = np.nonzero((e < frac * e0) & (t >= 0.5))[0]
        return float(t[below[0]]) if len(below) else float("nan")
    return at, when(0.5), when(0.1)


def energy_table(root, tag, best, log):
    meas = measured_energy(log)
    print(f"\nEnergy above rest (mJ) from each trial's first window, at {TIMES} s; t50 and t10 in s:")
    for trial in sorted(meas):
        t, e, e0 = meas[trial]
        at, t50, t10 = milestones(t, e, e0)
        print(f"  trial {trial}: start {1000 * e0:.1f}")
        print(f"    {'measured':<14} " + " ".join(f"{1000 * v:7.1f}" for v in at) + f"  t50 {t50:6.1f} t10 {t10:6.1f}")
        for engine, seed in best.items():
            rows = np.loadtxt(os.path.join(root, f"energy_{engine}_{tag}_s{seed}.csv"), delimiter=",", skiprows=1)
            rows = rows[rows[:, 0] == trial]
            a, t50, t10 = milestones(rows[:, 1], rows[:, 2], rows[0, 2])
            print(f"    {engine:<14} " + " ".join(f"{1000 * v:7.1f}" for v in a) + f"  t50 {t50:6.1f} t10 {t10:6.1f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("log")
    ap.add_argument("--tag", default="h4_e200")
    args = ap.parse_args()
    engines = ("riftphys", "mujocoEuler", "mujocoRK4")
    best, errs = {}, {}
    print(f"objective tag {args.tag}")
    for e in engines:
        rs = runs(args.root, e, args.tag)
        print(f"\n{e}: {len(rs)} seeds")
        for r in rs:
            print(f"  seed {r['seed']}: objective {r['objective']:.5f} | params {r['params']} | held: {r['held']}")
        top = min(rs, key=lambda r: r["objective"])
        print(f"  headline: seed {top['seed']}")
        best[e] = top["seed"]
        errs[e] = load_errors(top["csv"])
    print("\nHeld-out mean error (deg), headline seeds:")
    for e in engines:
        held = [r for r in errs[e] if r[0] in HELD]
        print(f"  {e:<14} " + "  ".join(f"{h:.2f} s {np.mean([math.degrees(r[2][k]) for r in held]):6.2f}" for k, h in enumerate(HORIZONS)))
    paired(errs["riftphys"], errs["mujocoEuler"], "MuJoCo Euler")
    paired(errs["riftphys"], errs["mujocoRK4"], "MuJoCo RK4")
    energy_table(args.root, args.tag, best, args.log)


if __name__ == "__main__":
    main()
