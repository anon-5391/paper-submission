"""Post-sweep quality flags for every results/<prefix>_*.csv row:
  - CONTACT : two parts of the deformed centreline that are >2 body-diameters
              apart along the arc come closer than one outer diameter
              (the scene has no self-collision, so the body passes through itself)
  - TIMEOUT : the per-task log (logs/sweep/<id>_c<k>_<label>.log) shows the pull
              was force-logged at SETTLE_MAX_STEPS_PULL rather than settled.
usage: python3 sweep_check.py <test_dir> [prefix]"""
import csv, glob, json, os, re, sys
import numpy as np

root = sys.argv[1]
prefix = sys.argv[2] if len(sys.argv) > 2 else "RD"
SCENE_N = 1000.0


def min_gap(P, D):
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    best = np.inf
    for i in range(len(P)):
        far = (s - s[i]) > 2 * D
        if far.any():
            best = min(best, np.linalg.norm(P[far] - P[i], axis=1).min())
    return best


tot = n_contact = n_timeout = 0
print(f"{'design':>7} {'cable':>5} {'N':>5} {'bend':>6} {'gap':>6}  flags")
for path in sorted(glob.glob(os.path.join(root, "results", f"{prefix}_*.csv")),
                   key=lambda p: float(re.search(r"_([\d.]+)\.csv", p).group(1))):
    did = os.path.basename(path)[:-4]
    dims = json.load(open(os.path.join(root, "stl", did + ".dims.json")))
    D = float(dims["backbone_outer_dia"])
    for r in csv.DictReader(open(path)):
        N = float(r["tension_newton"])
        if N == 0:
            continue
        n = int(r["n_spine"])
        P = np.array([[float(r[f"def_{a}{i}"]) for a in "xyz"] for i in range(n)])
        gap = min_gap(P, D)
        label = f"{round(N / 9.8 * 1000)}g"
        log = os.path.join(root, "logs", "sweep", f"{did}_c{r['cable']}_{label}.log")
        timeout = os.path.exists(log) and "TIMEOUT" in open(log, errors="ignore").read()
        flags = ("CONTACT " if gap < D else "") + ("TIMEOUT" if timeout else "")
        tot += 1; n_contact += gap < D; n_timeout += timeout
        print(f"{did:>7} {r['cable']:>5} {N:5.2f} {float(r['bend_angle_deg']):6.1f} {gap:6.1f}  {flags}")
print(f"\n{tot} loaded rows: {n_contact} self-contact, {n_timeout} timeout")
