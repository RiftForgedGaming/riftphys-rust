"""The P4 pairing as stated before any fit (PARITY_SPEC 3.4a, 3.4e, D2, D9): per jump, RiftPhys's
held-out loss minus the stricter MuJoCo integrator's (the stable one with the lower held-out mean at
that rate), the standard error of the mean difference by a bootstrap over the 22 jumps (10,000
resamples, seed 1), and the verdict: no worse when the mean difference is at most 2 SE. Main line,
then the sensitivity line.

    python -I tools/parity/cassie_pair.py --dir riftphys-parity-data/cassie_results
"""
import argparse
import json
import os

import numpy as np

JAN_27 = {0, 1, 2, 3, 6, 7, 8, 9}


def load(path):
    if not os.path.exists(path):
        return None
    doc = json.load(open(path))
    return {r["jump"]: r for r in doc["jumps"]}, doc


def stable(rows):
    return rows is not None and all(np.isfinite(r["main"]) and r["main"] < 1e5 for r in rows[0].values())


def pair(rift, ref, key, label):
    jumps = sorted(rift)
    d = np.array([rift[j][key] - ref[j][key] for j in jumps])
    rng = np.random.default_rng(1)
    boots = np.array([d[rng.integers(0, len(d), len(d))].mean() for _ in range(10000)])
    se = boots.std(ddof=1)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    verdict = "no worse (pass)" if d.mean() <= 2 * se else "worse (fail)"
    better = "better" if d.mean() + 2 * se < 0 else ("level" if d.mean() <= 2 * se else "behind")
    jan = np.mean([x for j, x in zip(jumps, d) if j in JAN_27])
    feb = np.mean([x for j, x in zip(jumps, d) if j not in JAN_27])
    print(f"  {label}: RiftPhys {np.mean([rift[j][key] for j in jumps]):.2f}, MuJoCo {np.mean([ref[j][key] for j in jumps]):.2f}; "
          f"difference {d.mean():+.2f} (SE {se:.2f}, 95% [{lo:+.2f}, {hi:+.2f}]); {verdict}, {better}; "
          f"by day: Jan 27 {jan:+.2f}, Feb 12 {feb:+.2f}; RiftPhys lower on {int((d < 0).sum())} of {len(d)}")
    return d.mean(), se


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    a = ap.parse_args()
    for hz in (240, 2000):
        rift = load(os.path.join(a.dir, f"riftphys_{hz}hz.json"))
        cands = {}
        for ig in ("euler", "rk4"):
            m = load(os.path.join(a.dir, f"mujoco_{ig}_{hz}hz.json"))
            if stable(m):
                cands[ig] = m
            else:
                print(f"{hz} Hz: MuJoCo {ig} {'not run' if m is None else 'not stable'}; not a candidate")
        if rift is None or not cands:
            print(f"{hz} Hz: nothing to pair")
            continue
        means = {ig: np.mean([r["main"] for r in m[0].values()]) for ig, m in cands.items()}
        ref_ig = min(means, key=means.get)
        print(f"{hz} Hz{' (the pass)' if hz == 240 else ''}: held-out means {', '.join(f'{k} {v:.2f}' for k, v in means.items())}; "
              f"stricter: {ref_ig}")
        for key, label in (("main", "main line"), ("sensitivity", "sensitivity line")):
            pair(rift[0], cands[ref_ig][0], key, label)


if __name__ == "__main__":
    main()
