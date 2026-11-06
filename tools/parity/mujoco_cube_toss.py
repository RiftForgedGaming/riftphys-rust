#!/usr/bin/env python3
"""Replay the ContactNets cube tosses in MuJoCo and score them exactly as riftphys-cube-toss does
(Acosta, Yang & Posa, RA-L 2022, Eq. 3), to check that RiftPhys's harness reproduces the published
MuJoCo row and to run MuJoCo side by side with RiftPhys.

Input: the reality log written by tools/contactnets_to_reality.py (y up, metres, [x, y, z, w]).
Defaults: the paper's fitted MuJoCo contact (Table IV: stiffness 3300, damping 45, friction 0.22),
1480 Hz, the cube of Table I (0.37 kg, half-width 0.0524 m, inertia 8.1e-4 kg m^2), the table at
the median rest height less a half-width (as riftphys-cube-toss).

  <venv>/python mujoco_cube_toss.py LOG.jsonl [--stiffness 3300] [--damping 45] [--friction 0.22]
                                             [--hz 1480] [--fit] [--out results.json]

Needs: mujoco (tested 3.17.0), numpy.
"""
import argparse
import json
import math

import mujoco
import numpy as np

HALF, MASS, INERTIA = 0.0524, 0.37, 8.1e-4
PUBLISHED_MUJOCO = (25.1, 10.8, 21.7, 21.4, 0.38, 0.63)


def load_tosses(path):
    tosses, cur, rate = [], None, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            t = o["t"]
            if t == "meta":
                rate = o["rate_hz"]
            elif t == "event" and o["e"] == "state":
                cur = {"index": o["data"]["toss"], "vel": o["data"]["vel"], "ang": o["data"]["ang"], "pos": [], "rot": []}
                tosses.append(cur)
            elif t == "event" and o["e"] == "break":
                cur = None
            elif t == "pose" and cur is not None:
                cur["pos"].append(o["pos"])
                cur["rot"].append(o["rot"])
    return tosses, 1.0 / rate


def model_xml(table_y, stiffness, damping, friction, hz):
    # y up: gravity along -y; the table plane's normal (its local z) is turned onto +y
    return f"""
<mujoco>
  <option timestep="{1.0 / hz:.10f}" gravity="0 -9.81 0" integrator="Euler"/>
  <worldbody>
    <geom name="table" type="plane" size="5 5 0.1" pos="0 {table_y:.6f} 0" quat="0.70710678 -0.70710678 0 0"
          friction="{friction} 0.005 0.0001" solref="{-stiffness} {-damping}"/>
    <body name="cube">
      <freejoint/>
      <inertial pos="0 0 0" mass="{MASS}" diaginertia="{INERTIA} {INERTIA} {INERTIA}"/>
      <geom type="box" size="{HALF} {HALF} {HALF}" friction="{friction} 0.005 0.0001" solref="{-stiffness} {-damping}"/>
    </body>
  </worldbody>
</mujoco>"""


def quat_xyzw_to_wxyz(q):
    return [q[3], q[0], q[1], q[2]]


def angle_between(qa_wxyz, qb_wxyz):
    d = abs(sum(a * b for a, b in zip(qa_wxyz, qb_wxyz)))
    return 2.0 * math.acos(min(1.0, d))


def replay(model, data, toss, frame_dt, hz):
    sub = max(1, round(frame_dt * hz))
    mujoco.mj_resetData(model, data)
    q0 = quat_xyzw_to_wxyz(toss["rot"][0])
    data.qpos[:3] = toss["pos"][0]
    data.qpos[3:7] = q0
    data.qvel[:3] = toss["vel"]
    # Free-joint angular velocity is in the body frame: w_body = R^T w_world
    rmat = np.zeros(9)
    mujoco.mju_quat2Mat(rmat, np.array(q0, dtype=float))
    data.qvel[3:6] = rmat.reshape(3, 3).T @ np.array(toss["ang"])
    mujoco.mj_forward(model, data)
    pos_sum = rot_sum = eq_sum = 0.0
    n = len(toss["pos"]) - 1
    for k in range(1, n + 1):
        for _ in range(sub):
            mujoco.mj_step(model, data)
        dp = math.dist(data.qpos[:3], toss["pos"][k])
        ang = angle_between(list(data.qpos[3:7]), quat_xyzw_to_wxyz(toss["rot"][k]))
        pos_sum += dp / (2 * HALF)
        rot_sum += ang
        eq_sum += dp * dp / HALF + ang * ang  # Eq. 3 as printed: (2/l) |dp|^2 (metres) + angle^2
    return 100.0 * pos_sum / n, math.degrees(rot_sum / n), eq_sum / n


def summary(rows):
    a = np.array(rows)
    return [(float(a[:, i].mean()), float(a[:, i].std())) for i in range(3)]


def run(tosses, frame_dt, table_y, stiffness, damping, friction, hz):
    model = mujoco.MjModel.from_xml_string(model_xml(table_y, stiffness, damping, friction, hz))
    data = mujoco.MjData(model)
    return [replay(model, data, t, frame_dt, hz) for t in tosses]


def fmt(name, s):
    return f"{name:<34}{s[0][0]:>7.1f} +-{s[0][1]:>5.1f}{s[1][0]:>8.1f} +-{s[1][1]:>5.1f}{s[2][0]:>7.2f} +-{s[2][1]:>5.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--stiffness", type=float, default=3300.0)
    ap.add_argument("--damping", type=float, default=45.0)
    ap.add_argument("--friction", type=float, default=0.22)
    ap.add_argument("--hz", type=float, default=1480.0)
    ap.add_argument("--out")
    args = ap.parse_args()

    tosses, frame_dt = load_tosses(args.log)
    rest = sorted(t["pos"][-1][1] - HALF for t in tosses)
    table_y = rest[len(rest) // 2]
    print(f"MuJoCo {mujoco.__version__}: {len(tosses)} tosses, table at y = {table_y:.4f} m, {args.hz:.0f} Hz, "
          f"stiffness {args.stiffness}, damping {args.damping}, friction {args.friction}")
    rows = run(tosses, frame_dt, table_y, args.stiffness, args.damping, args.friction, args.hz)
    s = summary(rows)
    p = PUBLISHED_MUJOCO
    print(f"\n{'':<34}{'position % width':>16}{'rotation deg':>16}{'e_q':>15}")
    print(fmt("MuJoCo (published, Table V)", [(p[0], p[1]), (p[2], p[3]), (p[4], p[5])]))
    print(fmt("MuJoCo (this harness), all", s))
    held = summary([r for t, r in zip(tosses, rows) if t["index"] % 2 == 1])
    print(fmt("MuJoCo (this harness), held-out", held))
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"mujoco": mujoco.__version__, "table_y": table_y, "params": vars(args), "all": s, "held_out": held,
                       "per_toss": [{"toss": t["index"], "pos_pct": r[0], "rot_deg": r[1], "e_q": r[2]} for t, r in zip(tosses, rows)]}, f)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
