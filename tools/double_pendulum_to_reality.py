#!/usr/bin/env python3
"""Converts the Myers et al. double-pendulum data (HardwareX 2020, Mendeley 10.17632/7yd2ntbh3w.1)
into a reality log for riftphys-double-pendulum (PARITY_SPEC W3).

Input: the dataset's Video_Tracking_Data (both links' angles at 500 Hz, degrees, unwrapped, 0
hanging straight down, about the same axis). The encoders measure only the upper link, so they
are a cross-check, not an input.

What the recordings hold, found by reading them:
- Tracking starts with the pendulum hanging. A hand lifts it to inverted over about 2 s, holds it,
  and lets go at about t = 5 s. Only the motion after the release is free, so each trial's log
  starts where a link has left the held pose by more than `LEFT_HOLD` degrees.
- The video clock matches the encoders' to 2 parts in 10^4 (fit over 35 s, all trials).

Output, in RiftPhys conventions (metres, seconds, y up, [x, y, z, w]):
- poses of bodies `upper` and `lower`: rotation by the link's angle about +z, which swings a link
  hanging along -y to (sin phi, -cos phi);
- one `rig` event: each link's mass, centre and inertia, summed from the paper's Table 6;
- `state` events, one per window: the angles, and rates from a +-10 ms quadratic fit (as the
  cube-toss start states); `break` events between trials.

    python -I tools/double_pendulum_to_reality.py DATASET_ROOT --out double_pendulum.jsonl

Needs: numpy. Reads the arrays with allow_pickle=False.
"""
import argparse
import json
import math
import os

import numpy as np

RATE_HZ = 500.0
TRIALS = (1, 2, 3)
# Degrees: a link this far from its held pose has been let go
LEFT_HOLD = 20.0
# A window every second from the release, while the lower link's smoothed rate peaks above this (rad/s)
WINDOW_S = 1.0
MIN_RATE = 1.0
FIT_HALF = 5  # samples either side: +-10 ms at 500 Hz
_X = np.arange(FIT_HALF, -FIT_HALF - 1, -1) / RATE_HZ
SLOPE = _X / (_X * _X).sum()  # convolution kernel: the fit's slope at the centre sample
TRIAL_GAP = 100.0  # seconds between trials in the log's clock

L = 0.1719  # top to bottom pivot, m

# Table 6, per item: mass (g), distance from the link's pivot along it (mm), inertia about its own
# centre (g mm^2). Table 6 gives the upper arm as one 132 g, 470000 g mm^2 part, but the upper link
# is two plates with the lower link between them. The measured small-swing modes (1.098 and 2.175
# Hz at 18-40 degrees) say two: the sum below gives 1.108 and 2.271 Hz; one plate gives 1.068 and
# 2.488. Parts at the lower pivot whose position follows the upper link only (shaft, pin, races,
# balls) are masses on the upper link; the outer race turns with the lower link, so its inertia
# goes there. Bearing balls: 0.3 g each, 7 in a 608 (assumed). The trackers' and tape's offsets
# aren't tabulated; they are placed at the arm's centre (under 1 g each).
PIVOT = L * 1000.0
UPPER = [
    (0.0, 0.0, 2000.0),      # encoder rotor
    (22.3, 0.0, 1100.0),     # flex coupling
    (2.9, 0.0, 103.0),       # top bearing inner race
    (2.3, 0.0, 40.0),        # top bearing spacer and balls
    (30.0, 4.0, 242.0),      # upper shaft
    (264.0, 86.0, 940000.0), # upper arm: two plates
    (3.8, PIVOT, 12.0),      # lower shaft
    (2.7, PIVOT, 61.0),      # dowel pin
    (2.9, PIVOT, 103.0),     # bottom bearing inner race
    (0.2, PIVOT, 2.5),       # bottom bearing spacer
    (2.1, PIVOT, 0.0),       # bottom bearing balls
    (6.5, PIVOT, 0.0),       # bottom bearing outer race (its position)
    (0.2, 86.0, 4.0),        # upper tracker
    (0.7, 86.0, 2500.0),     # upper arm tape
]
LOWER = [
    (116.0, 71.5, 245000.0), # lower arm
    (0.0, 0.0, 544.0),       # bottom bearing outer race (its rotation)
    (0.3, 71.5, 9.0),        # lower tracker
    (0.6, 71.5, 1300.0),     # lower arm tape
]


def link(items):
    """Mass (kg), centre from the pivot (m), inertia about the centre (kg m^2)."""
    mass = sum(m for m, _, _ in items)
    centre = sum(m * r for m, r, _ in items) / mass
    inertia = sum(i + m * (r - centre) ** 2 for m, r, i in items)
    return {"m": mass * 1e-3, "c": centre * 1e-3, "i": inertia * 1e-9}


def wrap180(a):
    return np.mod(a + 180.0, 360.0) - 180.0


def release_index(p1, p2):
    """The first sample after the hand's hold at which a link has left the held pose."""
    tracked = int(np.nonzero(np.abs(p1) > 1e-9)[0][0])
    hold = int(0.4 * RATE_HZ)
    upright = (np.abs(wrap180(p1 + 180.0)) < 25.0) & (np.abs(wrap180(p2 + 180.0)) < 25.0)
    for i in range(tracked, len(p1) - hold):
        if upright[i:i + hold].all() and np.ptp(p1[i:i + hold]) < 6.0 and np.ptp(p2[i:i + hold]) < 12.0:
            break
    else:
        raise ValueError("no held pose found")
    held1, held2 = np.median(p1[i:i + hold]), np.median(p2[i:i + hold])
    left = np.nonzero((np.abs(p1[i:] - held1) > LEFT_HOLD) | (np.abs(p2[i:] - held2) > LEFT_HOLD))[0]
    return i + int(left[0])


def rate(phi, k):
    """Rate (per second) at sample k from a quadratic through +-FIT_HALF samples."""
    x = np.arange(-FIT_HALF, FIT_HALF + 1) / RATE_HZ
    return float(np.polyfit(x, phi[k - FIT_HALF:k + FIT_HALF + 1], 2)[1])


def quat_z(phi):
    return [0.0, 0.0, math.sin(phi / 2.0), math.cos(phi / 2.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    upper, lower = link(UPPER), link(LOWER)
    print(f"upper {upper}\nlower {lower}")
    n_windows = 0
    with open(args.out, "w", encoding="utf-8") as f:
        def put(o):
            f.write(json.dumps(o, separators=(",", ":")) + "\n")
        put({"t": "meta", "v": "0.1", "source": "myers2020-video", "rate_hz": RATE_HZ, "bodies": ["upper", "lower"],
             "notes": "Myers et al. HardwareX 2020 double pendulum; angles as rotations about +z; 0 hanging"})
        put({"t": "event", "time": 0.0, "e": "rig", "data": {"upper": upper, "lower": lower, "l": L}})
        for trial in TRIALS:
            d = os.path.join(args.root, "Video_Tracking_Data", f"Trial{trial}")
            a = np.load(os.path.join(d, "DPmean_data_RB0.npy"), allow_pickle=False)
            b = np.load(os.path.join(d, "DPmean_data_RB1.npy"), allow_pickle=False)
            times, p1, p2 = a[0], np.radians(a[1]), np.radians(b[1])
            k0 = release_index(a[1], b[1])
            base = TRIAL_GAP * (trial - 1)
            step = int(WINDOW_S * RATE_HZ)
            kept = []
            for k in range(k0 + FIT_HALF, len(times) - step - FIT_HALF, step):
                # The +-10 ms smoothed rate: a raw sample difference reads 3-4 rad/s from tracker noise
                peak = np.abs(np.convolve(p2[k - FIT_HALF:k + step + FIT_HALF], SLOPE, mode="valid")).max()
                if peak >= MIN_RATE:
                    kept.append(k)
            states = {k: {"trial": trial, "phi": [float(p1[k]), float(p2[k])], "rate": [rate(p1, k), rate(p2, k)]} for k in kept}
            for k in range(k0, len(times)):
                t = base + float(times[k])
                if k in states:
                    put({"t": "event", "time": t, "e": "state", "data": states[k]})
                put({"t": "pose", "time": t, "body": "upper", "pos": [0.0, 0.0, 0.0], "rot": quat_z(float(p1[k]))})
                put({"t": "pose", "time": t, "body": "lower", "pos": [0.0, 0.0, 0.0], "rot": quat_z(float(p2[k]))})
            put({"t": "event", "time": base + float(times[-1]) + 0.001, "e": "break"})
            n_windows += len(kept)
            print(f"trial {trial}: released at t {times[k0]:.3f} s, phi {np.degrees(p1[k0]):.1f}, {np.degrees(p2[k0]):.1f} deg; {len(kept)} windows")
    print(f"{n_windows} windows -> {args.out}")


if __name__ == "__main__":
    main()
