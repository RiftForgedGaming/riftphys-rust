#!/usr/bin/env python3
"""Replays the real double pendulum in MuJoCo and scores it exactly as riftphys-double-pendulum
does (PARITY_SPEC W3): the same log, windows, rig, damping model, fit split and metric. Its held-out
error is the reference for criterion P3.

Input: the reality log written by tools/double_pendulum_to_reality.py.

  <venv>/python mujoco_double_pendulum.py LOG.jsonl [--hz 240] [--integrator Euler|RK4|implicitfast]
                                          [--params c1,f1,c2,f2] [--fit [--evals 200] [--seed 1] [--fit-horizons 4]]
                                          [--dump errors.csv] [--energy energy.csv] [--half-step-start]

Needs: mujoco (tested 3.17.0), numpy. The fit is the same CMA-ES as riftphys-reality's in outline
(population, weights and step-size rules of Hansen's tutorial), over the same box.
"""
import argparse
import json
import math

import mujoco
import numpy as np

HORIZONS = (0.1, 0.25, 0.5, 1.0)
FIT_TRIAL = 2
G = 9.81
HALF_STEP = False  # --half-step-start: for the semi-implicit integrators (Euler), as RiftPhys


def wrap(a):
    w = (a + math.pi) % (2.0 * math.pi) - math.pi
    return w + 2.0 * math.pi if w <= -math.pi else w


def angle_z(q):
    return wrap(2.0 * math.atan2(q[2], q[3]))


def load(path):
    rig, rate, states, times, upper, lower = None, None, [], [], [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            t = o["t"]
            if t == "meta":
                rate = o["rate_hz"]
            elif t == "pose":
                if o["body"] == "upper":
                    times.append(o["time"])
                    upper.append(angle_z(o["rot"]))
                else:
                    lower.append(angle_z(o["rot"]))
            elif t == "event" and o["e"] == "rig":
                rig = o["data"]
            elif t == "event" and o["e"] == "state":
                states.append((o["time"], o["data"]))
    times = np.array(times)
    windows = []
    for t0, d in states:
        k0 = int(np.searchsorted(times, t0 - 1e-6))
        ks = [k0 + round(h * rate) for h in HORIZONS]
        if ks[-1] >= len(times):
            continue
        windows.append({"trial": d["trial"], "time": t0, "phi": d["phi"], "rate": d["rate"], "at": [(upper[k], lower[k]) for k in ks]})
    return rig, windows


def model_xml(rig, c, f, hz, integrator):
    u, l = rig["upper"], rig["lower"]
    return f"""
<mujoco>
  <option timestep="{1.0 / hz:.10f}" gravity="0 {-G} 0" integrator="{integrator}"/>
  <worldbody>
    <body name="upper">
      <joint name="shoulder" type="hinge" axis="0 0 1" pos="0 0 0" damping="{c[0]:.12g}" frictionloss="{f[0]:.12g}"/>
      <inertial pos="0 {-u['c']:.10f} 0" mass="{u['m']:.10f}" diaginertia="{u['i']:.12g} {u['i']:.12g} {u['i']:.12g}"/>
      <body name="lower" pos="0 {-rig['l']:.10f} 0">
        <joint name="elbow" type="hinge" axis="0 0 1" pos="0 0 0" damping="{c[1]:.12g}" frictionloss="{f[1]:.12g}"/>
        <inertial pos="0 {-l['c']:.10f} 0" mass="{l['m']:.10f}" diaginertia="{l['i']:.12g} {l['i']:.12g} {l['i']:.12g}"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def start(model, data, w):
    """The measured state; with HALF_STEP, the rate less half of the first step's change."""
    def put(v):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = [w["phi"][0], w["phi"][1] - w["phi"][0]]
        data.qvel[:] = v
        mujoco.mj_forward(model, data)
    v0 = np.array([w["rate"][0], w["rate"][1] - w["rate"][0]])
    put(v0)
    if HALF_STEP:
        mujoco.mj_step(model, data)
        put(v0 - 0.5 * (np.array(data.qvel) - v0))


def replay(model, data, w, hz):
    start(model, data, w)
    step, out = 0, []
    for h in HORIZONS:
        target = round(h * hz)
        while step < target:
            mujoco.mj_step(model, data)
            step += 1
        q1, q2 = float(data.qpos[0]), float(data.qpos[0] + data.qpos[1])
        out.append((q1, q2))
    return out


def errors(rig, c, f, windows, hz, integrator):
    model = mujoco.MjModel.from_xml_string(model_xml(rig, c, f, hz, integrator))
    data = mujoco.MjData(model)
    rows = []
    for w in windows:
        sim = replay(model, data, w, hz)
        rows.append([math.sqrt((wrap(s[0] - a[0]) ** 2 + wrap(s[1] - a[1]) ** 2) / 2.0) for s, a in zip(sim, w["at"])])
    return np.array(rows)


def summary(rows):
    n = len(rows)
    return [(math.degrees(rows[:, k].mean()), math.degrees(rows[:, k].std() / math.sqrt(n))) for k in range(len(HORIZONS))]


def show(name, rows):
    cells = " | ".join(f"{h:.2f} s {m:6.2f} +-{se:5.2f}" for h, (m, se) in zip(HORIZONS, summary(rows)))
    print(f"{name:<26} {len(rows)} windows | deg: {cells}")


def energy(rig, phi, rate):
    """Energy above the hanging rest pose, as riftphys-double-pendulum computes it."""
    u, l, ll = rig["upper"], rig["lower"], rig["l"]
    s1, c1, s2, c2 = math.sin(phi[0]), math.cos(phi[0]), math.sin(phi[1]), math.cos(phi[1])
    vx, vy = ll * c1 * rate[0] + l["c"] * c2 * rate[1], ll * s1 * rate[0] + l["c"] * s2 * rate[1]
    kinetic = 0.5 * u["m"] * (u["c"] * rate[0]) ** 2 + 0.5 * u["i"] * rate[0] ** 2 + 0.5 * l["m"] * (vx * vx + vy * vy) + 0.5 * l["i"] * rate[1] ** 2
    return kinetic + u["m"] * G * u["c"] * (1 - c1) + l["m"] * G * (ll * (1 - c1) + l["c"] * (1 - c2))


def energy_run(rig, c, f, w, seconds, hz, integrator):
    model = mujoco.MjModel.from_xml_string(model_xml(rig, c, f, hz, integrator))
    data = mujoco.MjData(model)
    start(model, data, w)
    every = round(hz / 10.0)
    out = [(0.0, energy(rig, w["phi"], w["rate"]))]
    for i in range(1, round(seconds * hz) + 1):
        mujoco.mj_step(model, data)
        if i % every == 0:
            phi = [float(data.qpos[0]), float(data.qpos[0] + data.qpos[1])]
            rate = [float(data.qvel[0]), float(data.qvel[0] + data.qvel[1])]
            out.append((i / hz, energy(rig, phi, rate)))
    return out


def from_unit(x):
    lg = lambda v, lo, hi: 10.0 ** (lo + (hi - lo) * min(1.0, max(0.0, v)))
    return [lg(x[0], -6, -2), lg(x[2], -6, -2)], [lg(x[1], -5, -1), lg(x[3], -5, -1)]


def to_unit(c, f):
    n = lambda v, lo, hi: min(1.0, max(0.0, (math.log10(max(v, 1e-12)) - lo) / (hi - lo)))
    return [n(c[0], -6, -2), n(f[0], -5, -1), n(c[1], -6, -2), n(f[1], -5, -1)]


def cmaes(fn, x0, sigma, max_evals, seed):
    rng = np.random.default_rng(seed)
    n = len(x0)
    lam = 4 + int(3 * math.log(n))
    mu = lam // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    w /= w.sum()
    mueff = 1.0 / (w ** 2).sum()
    cc, cs = (4 + mueff / n) / (n + 4 + 2 * mueff / n), (mueff + 2) / (n + mueff + 5)
    c1, cmu = 2 / ((n + 1.3) ** 2 + mueff), min(1 - 2 / ((n + 1.3) ** 2 + mueff), 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0.0, math.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chi = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
    mean, C, pc, ps, evals = np.array(x0, float), np.eye(n), np.zeros(n), np.zeros(n), 0
    best, best_x = float("inf"), mean.copy()
    while evals < max_evals:
        D2, B = np.linalg.eigh((C + C.T) / 2)
        D = np.sqrt(np.maximum(D2, 1e-20))
        z = rng.standard_normal((lam, n))
        ys = (B @ (D * z).T).T
        xs = np.clip(mean + sigma * ys, 0.0, 1.0)
        fs = np.array([fn(x) for x in xs])
        evals += lam
        order = np.argsort(fs)
        if fs[order[0]] < best:
            best, best_x = float(fs[order[0]]), xs[order[0]].copy()
        y_sel = (xs[order[:mu]] - mean) / sigma
        yw = w @ y_sel
        mean = mean + sigma * yw
        cinv = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * cinv @ yw
        hs = np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * evals / lam)) / chi < 1.4 + 2 / (n + 1)
        pc = (1 - cc) * pc + (math.sqrt(cc * (2 - cc) * mueff) * yw if hs else 0)
        C = (1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (0 if hs else cc * (2 - cc) * C)) + cmu * (y_sel.T * w) @ y_sel
        sigma *= math.exp((cs / damps) * (np.linalg.norm(ps) / chi - 1))
    return best_x, evals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--hz", type=float, default=240.0)
    ap.add_argument("--integrator", default="Euler")
    ap.add_argument("--params", default="2e-5,1e-3,5e-6,3e-4")
    ap.add_argument("--fit", action="store_true")
    ap.add_argument("--evals", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fit-horizons", type=int, default=4)
    ap.add_argument("--dump")
    ap.add_argument("--energy")
    ap.add_argument("--half-step-start", action="store_true")
    args = ap.parse_args()
    global HALF_STEP
    HALF_STEP = args.half_step_start
    rig, windows = load(args.log)
    v = [float(x) for x in args.params.split(",")]
    c, f = [v[0], v[2]], [v[1], v[3]]
    train = [w for w in windows if w["trial"] == FIT_TRIAL]
    held = [w for w in windows if w["trial"] != FIT_TRIAL]
    print(f"MuJoCo {mujoco.__version__}, {args.hz:.0f} Hz, {args.integrator}, {len(windows)} windows")
    show("start: all", errors(rig, c, f, windows, args.hz, args.integrator))
    if args.fit:
        def objective(x):
            cc, ff = from_unit(x)
            return float(errors(rig, cc, ff, train, args.hz, args.integrator)[:, :args.fit_horizons].sum(axis=1).mean())
        x, n = cmaes(objective, to_unit(c, f), 0.2, args.evals, args.seed)
        c, f = from_unit(x)
        c, f = [float(v) for v in c], [float(v) for v in f]
        print(f"fitted after {n} evaluations: c {c}, f {f}; training objective {objective(x):.6f}")
        show("fitted: trial 2 (train)", errors(rig, c, f, train, args.hz, args.integrator))
        show("fitted: trials 1, 3 (held)", errors(rig, c, f, held, args.hz, args.integrator))
        show("fitted: all", errors(rig, c, f, windows, args.hz, args.integrator))
    print(f"params {c[0]},{f[0]},{c[1]},{f[1]}")
    if args.dump:
        rows = errors(rig, c, f, windows, args.hz, args.integrator)
        with open(args.dump, "w", encoding="utf-8") as out:
            out.write("trial,time,e0,e1,e2,e3\n")
            for w, r in zip(windows, rows):
                out.write(f"{w['trial']},{w['time']},{r[0]},{r[1]},{r[2]},{r[3]}\n")
    if args.energy:
        with open(args.energy, "w", encoding="utf-8") as out:
            out.write("trial,t,energy\n")
            for trial in (1, 2, 3):
                w = next((x for x in windows if x["trial"] == trial), None)
                if w is None:
                    continue
                for t, e in energy_run(rig, c, f, w, 60.0, args.hz, args.integrator):
                    out.write(f"{trial},{t},{e}\n")


if __name__ == "__main__":
    main()
