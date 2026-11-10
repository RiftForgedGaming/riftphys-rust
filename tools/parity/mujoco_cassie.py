"""The MuJoCo reference for P4 (PARITY_SPEC 3.4e): Agility's Cassie MJCF replaying the DAIR Lab
jumping-impact dataset from the same data states and inputs as RiftPhys, scored with the paper's
loss, fitted per integrator and rate on the same folds.

    python -I tools/parity/mujoco_cassie.py --xml cassie.xml --starts cassie_starts.json
        --check                         the mapping's round trip, start residuals, the loss port
        --replay --hz 240 --integrator euler [--k 1600 --d 270 --mu 0.43]
        --fit --hz 240 --integrator rk4 --evals 200 --out mujoco_rk4_240hz.json

The starts file is written by `riftphys-cassie --export-starts`: each trajectory's loop-consistent
start row (the data state both engines use), times, recorded states and inputs, and RiftPhys's own
replay of trajectory 0 to check this loss port against the Rust one.

Conventions (3.4e): shared joints map one to one; the pelvis's angular velocity is world-frame in a
row and body-frame in MuJoCo's ball joint, so it is rotated; each achilles rod's ball rotation and
each foot crank and plantar rod are solved so both loops close in position and velocity; the
actuators are motor-side (ctrl = joint torque / gear), unclipped; each step's control is the
first-order hold's mean over the step on the real sample times.
"""
import argparse
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mujoco_double_pendulum import cmaes  # noqa: E402  (the Python CMA-ES used in W3)

# The 16 joints of a row, in column order (positions 7..22, rates 29..44), and their MJCF names
JOINTS = ["hip_roll_left", "hip_roll_right", "hip_yaw_left", "hip_yaw_right", "hip_pitch_left", "hip_pitch_right",
          "knee_left", "knee_right", "knee_joint_left", "knee_joint_right", "ankle_joint_left", "ankle_joint_right",
          "ankle_spring_joint_left", "toe_left", "ankle_spring_joint_right", "toe_right"]
MJ = {"hip_roll_left": "left-hip-roll", "hip_roll_right": "right-hip-roll", "hip_yaw_left": "left-hip-yaw",
      "hip_yaw_right": "right-hip-yaw", "hip_pitch_left": "left-hip-pitch", "hip_pitch_right": "right-hip-pitch",
      "knee_left": "left-knee", "knee_right": "right-knee", "knee_joint_left": "left-shin", "knee_joint_right": "right-shin",
      "ankle_joint_left": "left-tarsus", "ankle_joint_right": "right-tarsus", "ankle_spring_joint_left": "left-heel-spring",
      "ankle_spring_joint_right": "right-heel-spring", "toe_left": "left-foot", "toe_right": "right-foot"}
# The dataset's input order and the MJCF actuators' names
INPUTS = ["hip_roll_left", "hip_roll_right", "hip_yaw_left", "hip_yaw_right", "hip_pitch_left", "hip_pitch_right",
          "knee_left", "knee_right", "toe_left", "toe_right"]
# The redundant coordinates per side: the achilles rod's ball joint, the foot crank, the plantar rod
REDUNDANT = {"left": ("left-achilles-rod", "left-foot-crank", "left-plantar-rod"),
             "right": ("right-achilles-rod", "right-foot-crank", "right-plantar-rod")}
INTEGRATORS = {"euler": 0, "rk4": 1}
LAGGED_RATES = [(15, 37), (16, 38), (17, 39), (18, 40)]


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_from_rotvec(v):
    a = np.linalg.norm(v)
    if a < 1e-15:
        return np.array([1.0, *(0.5 * v)])
    return np.array([math.cos(a / 2), *(math.sin(a / 2) * v / a)])


def rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Cassie:
    """The MJCF loaded once, with its joint addresses; contact parameters, integrator and timestep set per run."""

    def __init__(self, xml):
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        m = self.m
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qadr = {j: m.jnt_qposadr[jid(MJ[j])] for j in JOINTS}
        self.vadr = {j: m.jnt_dofadr[jid(MJ[j])] for j in JOINTS}
        self.red = {side: (m.jnt_qposadr[jid(a)], m.jnt_dofadr[jid(a)], m.jnt_qposadr[jid(c)], m.jnt_dofadr[jid(c)],
                           m.jnt_qposadr[jid(r)], m.jnt_dofadr[jid(r)]) for side, (a, c, r) in REDUNDANT.items()}
        # The ankle springs: unmeasured on the robot (its estimator solved them by closing the
        # achilles loop in the URDF's geometry), so they are re-solved here in the MJCF's (3.4e)
        self.heel = {side: (m.jnt_qposadr[jid(side + "-heel-spring")], m.jnt_dofadr[jid(side + "-heel-spring")]) for side in ("left", "right")}
        self.last_heel_change = (0.0, 0.0)
        self.act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, MJ[n]) for n in INPUTS]
        self.gear = np.array([m.actuator_gear[a][0] for a in self.act])
        m.actuator_ctrllimited[:] = 0  # (d) no clipping: the data exceeds the motors' peaks
        self.base_solref = m.geom_solref.copy()
        self.base_friction = m.geom_friction.copy()

    def set_params(self, k, dmp, mu, hz, integrator):
        m = self.m
        m.geom_solref[:, 0] = -k  # direct stiffness and damping (negative solref), every geom, as dairlib
        m.geom_solref[:, 1] = -dmp
        m.geom_friction[:, 0] = mu
        m.opt.timestep = 1.0 / hz
        m.opt.integrator = INTEGRATORS[integrator]

    # ---- the mapping ----
    def to_mujoco(self, row):
        """A dataset row to qpos, qvel, closing both loops in position and velocity. Returns the
        equality residuals (position m, velocity m/s)."""
        m, d = self.m, self.d
        q = np.zeros(m.nq)
        v = np.zeros(m.nv)
        q[0:3] = row[4:7]
        q[3:7] = row[0:4] / np.linalg.norm(row[0:4])
        v[0:3] = row[26:29]
        v[3:6] = rot(q[3:7]).T @ np.array(row[23:26])  # (a) world-frame w into the ball joint's frame
        for k, j in enumerate(JOINTS):
            q[self.qadr[j]] = row[7 + k]
            v[self.vadr[j]] = row[29 + k]
        # (b) the redundant positions. Start: each achilles rod pointed at its heel-spring anchor in
        # closed form (the anchor depends only on the leg's joints), the foot crank and plantar rod
        # from the toe (dairlib's guess); then Levenberg-Marquardt on the equality residuals
        for side, (aq, _, cq, _, rq, _) in self.red.items():
            toe = row[7 + JOINTS.index("toe_" + side)]
            q[aq:aq + 4] = [1.0, 0.0, 0.0, 0.0]
            q[cq] = toe
            q[rq] = -toe
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        for e in range(m.neq):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e)
            if "achilles" not in name:
                continue
            side = "left" if name.startswith("left") else "right"
            aq = self.red[side][0]
            rod, heel = m.eq_obj1id[e], m.eq_obj2id[e]
            target = d.xpos[heel] + d.xmat[heel].reshape(3, 3) @ m.eq_data[e][3:6]
            local = np.array(m.eq_data[e][0:3])  # the anchor on the rod, in the rod's frame
            want = d.xmat[rod].reshape(3, 3).T @ (target - d.xpos[rod])  # at identity rotation, in the rod's frame
            a, b = local / np.linalg.norm(local), want / np.linalg.norm(want)
            axis = np.cross(a, b)
            ang = math.atan2(np.linalg.norm(axis), float(np.dot(a, b)))
            q[aq:aq + 4] = quat_from_rotvec(axis / max(np.linalg.norm(axis), 1e-12) * ang)

        def residual(q):
            d.qpos[:] = q
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            eq = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
            return d.efc_pos[eq].copy()

        def apply(q, x):
            q = q.copy()
            for s, side in enumerate(("left", "right")):
                aq, _, cq, _, rq, _ = self.red[side]
                q[aq:aq + 4] = quat_mul(q[aq:aq + 4], quat_from_rotvec(x[6 * s:6 * s + 3]))
                q[aq:aq + 4] /= np.linalg.norm(q[aq:aq + 4])
                q[cq] += x[6 * s + 3]
                q[rq] += x[6 * s + 4]
                q[self.heel[side][0]] += x[6 * s + 5]
            return q

        heel0 = [q[self.heel[side][0]] for side in ("left", "right")]

        lam = 1e-6
        for _ in range(100):
            r = residual(q)
            if np.max(np.abs(r)) < 1e-10:
                break
            jac = np.zeros((len(r), 12))
            for i in range(12):
                e = np.zeros(12)
                e[i] = 1e-7
                jac[:, i] = (residual(apply(q, e)) - r) / 1e-7
            jtj, jtr = jac.T @ jac, jac.T @ r
            while True:
                step = np.linalg.solve(jtj + lam * np.eye(12), -jtr)
                trial = apply(q, step)
                if np.sum(residual(trial) ** 2) < np.sum(r ** 2) or lam > 1e6:
                    q, lam = trial, max(lam / 10, 1e-12)
                    break
                lam *= 10
        pos_res = np.max(np.abs(residual(q)))
        heel_dq = [q[self.heel[side][0]] - heel0[k] for k, side in enumerate(("left", "right"))]
        # (b) the redundant rates: J_u v_u = -J_k v_k on the equality rows
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        eq = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        jmat = d.efc_J.reshape(d.nefc, m.nv)[eq]
        unknown = []
        for side in ("left", "right"):
            _, av, _, cv, _, rv = self.red[side]
            unknown += [av, av + 1, av + 2, cv, rv, self.heel[side][1]]
        heel_v0 = [v[self.heel[side][1]] for side in ("left", "right")]
        known = [i for i in range(m.nv) if i not in unknown]
        v[unknown] = np.linalg.lstsq(jmat[:, unknown], -jmat[:, known] @ v[known], rcond=None)[0]
        vel_res = np.max(np.abs(jmat @ v))
        self.last_heel_change = (max(abs(x) for x in heel_dq), max(abs(v[self.heel[side][1]] - heel_v0[k]) for k, side in enumerate(("left", "right"))))
        return q, v, pos_res, vel_res

    def to_row(self, q, v):
        row = np.zeros(45)
        row[0:4] = q[3:7]
        if row[0] < 0:
            row[0:4] = -row[0:4]
        row[4:7] = q[0:3]
        row[23:26] = rot(q[3:7]) @ v[3:6]  # back to world-frame w
        row[26:29] = v[0:3]
        for k, j in enumerate(JOINTS):
            row[7 + k] = q[self.qadr[j]]
            row[29 + k] = v[self.vadr[j]]
        return row


def input_mean(t, u, a, b):
    """The first-order hold's mean over [a, b] on the real sample times (held at the ends), exact
    for a piecewise-linear input."""
    def at(time):
        if time <= t[0]:
            return u[0]
        if time >= t[-1]:
            return u[-1]
        k = np.searchsorted(t, time, side="right") - 1
        s = (time - t[k]) / (t[k + 1] - t[k])
        return u[k] * (1 - s) + u[k + 1] * s
    pts = [a] + [x for x in t if a < x < b] + [b]
    total = sum(0.5 * (at(p0) + at(p1)) * (p1 - p0) for p0, p1 in zip(pts[:-1], pts[1:]))
    return total / (b - a)


def run(c, traj, start_row, hz, steps_out, trial=False, contact=True):
    """Steps from `start_row`; returns the rows after each step (with the start first) and times."""
    m, d = c.m, c.d
    q, v, pres, vres = c.to_mujoco(start_row)
    d.qpos[:] = q
    d.qvel[:] = v
    d.act[:] = 0
    d.time = 0.0
    if not contact:
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    t = np.array(traj["t"])
    u = np.array(traj["u"])
    t0 = t[0]
    h = 1.0 / hz
    rows = [c.to_row(d.qpos, d.qvel)]
    times = [0.0]
    for k in range(steps_out):
        a, b = t0 + k * h, t0 + (k + 1) * h
        d.ctrl[c.act] = input_mean(t, u, a, b) / c.gear
        mujoco.mj_step(m, d)
        rows.append(c.to_row(d.qpos, d.qvel))
        times.append(b - t0)
    if not contact:
        m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return np.array(rows), np.array(times), pres, vres


def replay(c, traj, start_row, hz, integrator, line="main"):
    """The decided start (3.4e): Euler takes the gated half-step correction from its own trial
    steps, RK4 none; then the run, interpolated to the sample times."""
    t = np.array(traj["t"])
    start = np.array(start_row, dtype=float)
    if integrator == "euler":
        (r1, _, _, _) = run(c, traj, start, hz, 1, contact=False)
        rf = run(c, traj, start, 2000.0, 1, contact=False)[0] if hz != 2000.0 else r1
        c.m.opt.timestep = 1.0 / hz
        h, hf = 1.0 / hz, 1.0 / 2000.0
        for i in range(23, 45):
            dc, df = r1[1][i] - start[i], rf[1][i] - start[i]
            if abs(df) > 1e-12 and abs((dc / df) / (h / hf) - 1.0) <= 0.25:
                start[i] = start[i] - 0.5 * dc
    if line == "sensitivity":
        start[41] = 0.0
        start[43] = 0.0
    end = t[-1] - t[0]
    steps = int(math.ceil(end * hz)) + 1
    rows, times, pres, vres = run(c, traj, start, hz, steps)
    out = []
    for tk in t - t[0]:
        i = min(max(np.searchsorted(times, tk, side="right"), 1), len(times) - 1)
        s = min(max((tk - times[i - 1]) / (times[i] - times[i - 1]), 0.0), 1.0)
        a, b = rows[i - 1], rows[i]
        sign = -1.0 if np.dot(a[0:4], b[0:4]) < 0 else 1.0
        r = a * (1 - s) + b * s
        r[0:4] = a[0:4] * (1 - s) + sign * b[0:4] * s
        r[0:4] /= np.linalg.norm(r[0:4])
        out.append(r)
    return np.array(out), pres, vres


VEL_W = np.array([100.0, 100.0, 100.0, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0, 0.0, 0.01, 0.0, 0.01])


def quat_angle(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return 2.0 * math.acos(min(abs(float(np.dot(a, b))), 1.0))


def loss_terms(sim, data):
    """The paper's loss as `cassie_loss_utils.py` computes it (and as `cassie::loss_terms` does)."""
    sim, data = np.asarray(sim), np.asarray(data)
    n = min(len(sim), len(data))
    s, x = sim[:n], data[:n]
    pos = 10.0 * np.sum((s[:, 4:23] - x[:, 4:23]) ** 2) / n
    vel = np.sum(VEL_W * (s[:, 26:45] - x[:, 26:45]) ** 2) / n
    om = 5.0 * np.sum((s[:, 23:26] - x[:, 23:26]) ** 2) / n
    quat = sum(quat_angle(s[k, 0:4], x[k, 0:4]) ** 2 for k in range(n)) / n
    return [pos, vel, om, quat]


def loss(sim, data):
    return sum(loss_terms(sim, data))


def check(c, doc):
    """The round trip, the start residuals per jump, and the loss port against the Rust one."""
    worst = np.zeros(4)
    for tr in doc["trajectories"]:
        row = np.array(tr["start"])
        q, v, pres, vres = c.to_mujoco(row)
        back = c.to_row(q, v)
        a = np.array(row[0:4]) / np.linalg.norm(row[0:4])
        keep = [i for i in range(4, 45) if i not in (19, 21, 41, 43)]  # the ankle springs are re-solved
        pk = [i for i in keep if i < 23]
        rk = [i for i in keep if i >= 29]
        e = [quat_angle(a, back[0:4]), np.max(np.abs(row[pk] - back[pk])), np.max(np.abs(row[23:29] - back[23:29])),
             np.max(np.abs(row[rk] - back[rk]))]
        worst = np.maximum(worst, e)
        hq, hv = c.last_heel_change
        print(f"traj {tr['index']:2}: start equality residuals {pres:.1e} m, {vres:.1e} m/s; ankle springs moved {hq * 1e3:.2f} mrad, {hv:.3f} rad/s")
    print(f"round trip (the ankle springs apart), worst: orientation {worst[0]:.1e} rad, positions {worst[1]:.1e}, pelvis rates {worst[2]:.1e}, joint rates {worst[3]:.1e}")
    lc = doc["loss_check"]
    data = np.array(doc["trajectories"][lc["trajectory"]]["x"])
    mine = loss_terms(np.array(lc["states"]), data)
    print(f"the loss port on RiftPhys's trajectory-0 replay: {sum(mine):.6f} (Rust {lc['loss']:.6f}); terms {np.round(mine, 6)} vs {np.round(lc['terms'], 6)}")


def mean_loss(c, doc, jumps, k, dmp, mu, hz, integrator, line="main"):
    c.set_params(k, dmp, mu, hz, integrator)
    total = 0.0
    for j in jumps:
        tr = doc["trajectories"][j]
        try:
            sim, _, _ = replay(c, tr, tr["start"], hz, integrator, line)
            val = loss(sim, tr["x"])
            total += val if math.isfinite(val) else 1e6
        except Exception:
            total += 1e6
    return total / len(jumps)


def from_unit(x):
    x = np.clip(x, 0.0, 1.0)
    return 10.0 ** (1.0 + 5.0 * x[0]), 1000.0 * x[1], 0.01 + 0.99 * x[2]


def to_unit(k, dmp, mu):
    return np.array([(math.log10(k) - 1.0) / 5.0, dmp / 1000.0, (mu - 0.01) / 0.99])


def fit_all(c, doc, hz, integrator, evals, out):
    paper = [tr["index"] for tr in doc["trajectories"] if tr["paper"]]
    folds = doc["folds"]
    start = (1600.0, 270.0, 0.43)
    rows, fold_params = [], []
    for f, held in enumerate(folds):
        train = [j for j in paper if j not in held]
        x, _ = cmaes(lambda x: mean_loss(c, doc, train, *from_unit(x), hz, integrator), to_unit(*start), 0.2, evals, 1)
        k, dmp, mu = from_unit(x)
        fbest = mean_loss(c, doc, train, k, dmp, mu, hz, integrator)
        print(f"fold {f + 1} (held out {held}): stiffness {k:.0f}, damping {dmp:.1f}, mu {mu:.3f}; training loss {fbest:.2f}", flush=True)
        fold_params.append({"fold": f + 1, "held_out": held, "stiffness": k, "damping": dmp, "mu": mu, "train_loss": fbest})
        c.set_params(k, dmp, mu, hz, integrator)
        for j in held:
            tr = doc["trajectories"][j]
            sim, pres, vres = replay(c, tr, tr["start"], hz, integrator, "main")
            sens, _, _ = replay(c, tr, tr["start"], hz, integrator, "sensitivity")
            rows.append({"jump": j, "fold": f + 1, "main": loss(sim, tr["x"]), "sensitivity": loss(sens, tr["x"]),
                         "terms": loss_terms(sim, tr["x"]), "start_residuals": [pres, vres]})
    mean = lambda key: sum(r[key] for r in rows) / len(rows)
    print(f"{integrator} {hz} Hz, held out, mean over the 22: main {mean('main'):.2f}, sensitivity {mean('sensitivity'):.2f}")
    x, _ = cmaes(lambda x: mean_loss(c, doc, paper, *from_unit(x), hz, integrator), to_unit(*start), 0.2, evals, 1)
    k, dmp, mu = from_unit(x)
    in_loss = mean_loss(c, doc, paper, k, dmp, mu, hz, integrator)
    print(f"in-sample, all 22: stiffness {k:.0f}, damping {dmp:.1f}, mu {mu:.3f}; loss {in_loss:.2f}")
    json.dump({"engine": "mujoco " + mujoco.__version__, "integrator": integrator, "hz": hz, "evals": evals, "seed": 1,
               "folds": fold_params, "jumps": rows, "in_sample": {"stiffness": k, "damping": dmp, "mu": mu, "loss": in_loss}},
              open(out, "w"), indent=1)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True)
    ap.add_argument("--starts", required=True)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--fit", action="store_true")
    ap.add_argument("--hz", type=float, default=240.0)
    ap.add_argument("--integrator", choices=list(INTEGRATORS), default="euler")
    ap.add_argument("--k", type=float, default=1600.0)
    ap.add_argument("--d", type=float, default=270.0)
    ap.add_argument("--mu", type=float, default=0.43)
    ap.add_argument("--evals", type=int, default=200)
    ap.add_argument("--out", default="mujoco_cassie_fit.json")
    a = ap.parse_args()
    doc = json.load(open(a.starts))
    c = Cassie(a.xml)
    if a.check:
        check(c, doc)
    if a.replay:
        paper = [tr["index"] for tr in doc["trajectories"] if tr["paper"]]
        for line in ("main", "sensitivity"):
            print(f"{a.integrator} {a.hz} Hz, {line} line, k {a.k} d {a.d} mu {a.mu}: mean loss over the 22 "
                  f"{mean_loss(c, doc, paper, a.k, a.d, a.mu, a.hz, a.integrator, line):.2f}")
    if a.fit:
        fit_all(c, doc, a.hz, a.integrator, a.evals, a.out)


if __name__ == "__main__":
    main()
