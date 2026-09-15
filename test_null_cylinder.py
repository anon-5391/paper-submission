#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_null_cylinder.py — bending BASELINE: a plain, notch-free cylinder STL run
through the exact same tendon / mesh / load parameters as the notched-backbone
pipeline, for direct comparison against results/BL_100.csv (same 25mm outer
dia x 100mm length envelope as the default caddesign geometry, minus the
flexure notches). If the notches are what makes a TDCR bend, this design
should barely bend at all under the same cable loads.

Same parameters as a real sweep design, end to end:
  * mesh   — pipeline_config.py's default CGAL resolution (no overrides;
             same knobs run_one.sh / sweep_cli.py Phase 1 use),
  * cables — 3 (TENDON_COUNT, same as run_one.sh's default tendon_count),
  * tendon radial offset — 9.5 mm (same default as caddesign / run_one.sh),
  * loads  — CABLE_PULL_LEVELS from pipeline_config.py (all 10 default levels),
  * one fresh runSofa pull per (cable, load) from rest — same methodology as
    sweep_cli.py's Phase 2 (not a single ratcheting run through all levels).

The input STL is auto-rotated if its long axis isn't Y: this pipeline assumes
a +Y backbone everywhere (caddesign's own export convention, and the only
axis keyboard_fixing_box/synth_cable-style tendon generation supports), so a
foreign STL with e.g. a Z-long axis needs a lossless 90 deg reorientation
first or the synthesized tendon path ends up outside the solid.

Because every (cable, load) pull is an independent, fresh-from-rest runSofa
process, they can safely run in parallel (--jobs) as long as each writes to
its own file — so each task logs to its own CSV under analysis/null_test/,
and this script merges them into results/NULL_CYL.csv (same schema as every
other design's results CSV) once everything finishes, ready for plot.py /
sweep_analysis.py or a manual diff against results/BL_100.csv.

Usage:
    python3 test_null_cylinder.py --stl Null_Test_Cylinder.stl
    python3 test_null_cylinder.py --stl Null_Test_Cylinder.stl --jobs 10
    python3 test_null_cylinder.py --stl Null_Test_Cylinder.stl --headless
"""
import argparse
import csv
import json
import os
import shutil
import struct
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mesh_utils import mesh_stats                                    # stdlib-only VTK reader
from pipeline_config import CABLE_PULL_LEVELS, CABLE_VALUE_TYPE, SCENE_FORCE_PER_NEWTON

DESIGN_ID     = "NULL_CYL"
TENDON_COUNT  = 3      # same as run_one.sh's default tendon_count (positional #6)
TENDON_DIST   = 9.5    # mm — same default tendon radial offset as caddesign.py
CABLE_SAMPLES = 25
SIM_STEPS_CAP = 20000  # runSofa -n cap; one pull settles well under SETTLE_MAX_STEPS_PULL


# ── STL reorientation (lossless 90 deg rotation, not a shape change) ───────────

def _read_stl_triangles(path):
    with open(path, "rb") as f:
        header = f.read(80)
        ntri = struct.unpack("<I", f.read(4))[0]
        data = f.read()
    return header, ntri, data


def _bbox_extents(ntri, data):
    xs, ys, zs = [], [], []
    for i in range(ntri):
        rec = data[i * 50:(i + 1) * 50]
        v = struct.unpack("<12f", rec[:48])
        for p in (v[3:6], v[6:9], v[9:12]):
            xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
    return [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]


def reorient_if_needed(src, dst):
    """Copy src -> dst, rotating -90 deg about X ((x,y,z) -> (x,z,-y)) if the
    long axis isn't already Y. That rotation has determinant +1 (a proper
    rotation, not a mirror), so triangle winding stays outward-facing and no
    vertex reordering is needed — normals get the same transform."""
    header, ntri, data = _read_stl_triangles(src)
    extents = _bbox_extents(ntri, data)
    axis = extents.index(max(extents))
    if axis == 1:
        shutil.copyfile(src, dst)
        print(f"[geom] backbone already along Y (extents X={extents[0]:.1f} "
              f"Y={extents[1]:.1f} Z={extents[2]:.1f}) — no reorientation needed")
        return

    print(f"[geom] backbone along {'XYZ'[axis]} (extents X={extents[0]:.1f} "
          f"Y={extents[1]:.1f} Z={extents[2]:.1f}) — rotating -90 deg about X so it's Y")

    def rot(p):
        return (p[0], p[2], -p[1])

    out = bytearray(header)
    out += struct.pack("<I", ntri)
    for i in range(ntri):
        rec = data[i * 50:(i + 1) * 50]
        v = struct.unpack("<12f", rec[:48])
        attr = rec[48:50]
        n, v1, v2, v3 = rot(v[0:3]), rot(v[3:6]), rot(v[6:9]), rot(v[9:12])
        out += struct.pack("<12f", *n, *v1, *v2, *v3)
        out += attr
    with open(dst, "wb") as f:
        f.write(out)


# ── shell / io helpers ───────────────────────────────────────────────────────

def run(cmd, env, log_path):
    t0 = time.monotonic()
    with open(log_path, "w") as lf:
        rc = subprocess.call(cmd, env=env, cwd=SCRIPT_DIR, stdout=lf, stderr=subprocess.STDOUT)
    return rc, time.monotonic() - t0


def all_rows(csv_path):
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def synth_cable(backbone_length, radius, n=CABLE_SAMPLES):
    """Straight tendon-1 centerline in the pipeline's +Y frame, full base->tip."""
    return [[radius, round(backbone_length * i / (n - 1), 4), 0.0] for i in range(n)]


def hms(seconds):
    s = int(round(seconds))
    return (f"{s // 3600}h{s % 3600 // 60:02d}m{s % 60:02d}s" if s >= 3600 else
            f"{s // 60}m{s % 60:02d}s" if s >= 60 else f"{s}s")


# ── driver ───────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl", required=True, help="plain-cylinder STL to test")
    ap.add_argument("--jobs", type=int, default=1,
                    help="parallel runSofa sim processes (mesh step is always single)")
    ap.add_argument("--headless", action="store_true", help="wrap runSofa in xvfb-run")
    ap.add_argument("--tendon-dist", type=float, default=TENDON_DIST)
    ap.add_argument("--tendon-count", type=int, default=TENDON_COUNT)
    a = ap.parse_args()

    if CABLE_VALUE_TYPE != "force":
        print(f"[warn] pipeline_config.CABLE_VALUE_TYPE = {CABLE_VALUE_TYPE!r}; "
              f"CABLE_PULL_LEVELS are scene-force units and assume \"force\" mode.")

    out_dir = os.path.join(SCRIPT_DIR, "analysis", "null_test", DESIGN_ID)
    os.makedirs(out_dir, exist_ok=True)
    runsofa = ["xvfb-run", "-a", "runSofa"] if a.headless else ["runSofa"]

    src = a.stl if os.path.isabs(a.stl) else os.path.join(SCRIPT_DIR, a.stl)
    if not os.path.exists(src):
        sys.exit(f"STL not found: {src}")
    stl = os.path.join(out_dir, f"{DESIGN_ID}.stl")
    reorient_if_needed(src, stl)

    # ── mesh once, at the pipeline's default CGAL resolution (no TDCR_CGAL_*
    #    overrides — same as every real design's Stage-2 mesh) ───────────────
    vtk = os.path.join(out_dir, f"{DESIGN_ID}.vtk")
    print("[mesh] tetrahedralizing at pipeline default resolution ...")
    menv = dict(os.environ)
    menv.update({"TDCR_STL": stl, "TDCR_VTK": vtk, "TDCR_MESH_FORCE": "1"})
    rc, t = run(runsofa + ["-l", "SofaPython3", "-l", "CGALPlugin", "-g", "batch",
                           os.path.join(SCRIPT_DIR, "dup.py")],
                menv, os.path.join(out_dir, "mesh.log"))
    if rc != 0 or not os.path.exists(vtk):
        sys.exit(f"mesh failed (rc={rc}) — see {out_dir}/mesh.log")
    st = mesh_stats(vtk)
    if "error" in st:
        sys.exit(f"unreadable mesh ({st['error']})")
    yi = st["extents"].index(max(st["extents"]))
    backbone_length = st["extents"][yi]
    print(f"[mesh] {st['n_points']} nodes, backbone_length={backbone_length:.2f} mm  ({hms(t)})")

    cable_json = os.path.join(out_dir, "cable1.json")
    with open(cable_json, "w") as f:
        json.dump(synth_cable(backbone_length, a.tendon_dist), f)

    # ── one fresh runSofa pull per (cable, load) — same methodology as
    #    sweep_cli.py's Phase 2, so results are directly comparable ─────────
    tasks = [(ci, tension) for ci in range(a.tendon_count) for tension in CABLE_PULL_LEVELS]
    print(f"[sim] {len(tasks)} sims ({a.tendon_count} cables x {len(CABLE_PULL_LEVELS)} loads), "
          f"jobs={a.jobs}")

    def _task(ci, tension):
        tn = tension / SCENE_FORCE_PER_NEWTON
        task_csv = os.path.join(out_dir, f"task_c{ci}_{tn:g}N.csv")
        if os.path.exists(task_csv):
            os.remove(task_csv)
        senv = dict(os.environ)
        senv.update({
            "TDCR_STL": stl, "TDCR_VTK": vtk, "TDCR_CABLE_JSON": cable_json,
            "TDCR_DESIGN_NAME": DESIGN_ID, "TDCR_TENDON_COUNT": str(a.tendon_count),
            "TDCR_CABLE_INDEX": str(ci), "TDCR_TENSION_LEVELS": str(tension),
            "TDCR_OUTPUT_CSV": task_csv, "TDCR_BACKBONE_LENGTH": str(backbone_length),
        })
        log = os.path.join(out_dir, f"sim_c{ci}_{tn:g}N.log")
        rc, s = run(runsofa + ["-l", "SofaPython3", "-l", "STLIB", "-l", "SoftRobots",
                               "-g", "batch", os.path.join(SCRIPT_DIR, "main.py"),
                               "-n", str(SIM_STEPS_CAP)],
                    senv, log)
        rows = all_rows(task_csv)
        return ci, tension, rc, s, rows, log

    merged = {}   # (cable, tension_newton) -> row
    t0 = time.monotonic()
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        futs = {pool.submit(_task, ci, tension): (ci, tension) for ci, tension in tasks}
        for n_done, fut in enumerate(as_completed(futs), 1):
            ci, tension, rc, s, rows, log = fut.result()
            tn = tension / SCENE_FORCE_PER_NEWTON
            for row in rows:
                merged[(row.get("cable"), row.get("tension_newton"))] = row
            pulled = rows[-1] if rows else None
            ang = f"{float(pulled['bend_angle_deg']):.4f} deg" if pulled else "--"
            flag = "" if pulled else f"  [no row rc={rc}, see {os.path.basename(log)}]"
            eta = (time.monotonic() - t0) / n_done * (len(tasks) - n_done)
            print(f"  [{n_done}/{len(tasks)}] cable {ci + 1}  {tn:.2f} N -> {ang}  "
                  f"({hms(s)})  eta {hms(eta)}{flag}", flush=True)

    # ── merge into results/NULL_CYL.csv, same schema as the real pipeline ───
    results_csv = os.path.join(SCRIPT_DIR, "results", f"{DESIGN_ID}.csv")
    os.makedirs(os.path.dirname(results_csv), exist_ok=True)
    if not merged:
        sys.exit(f"\n[FAIL] no sim produced a usable row — check {out_dir}/sim_*.log")

    fieldnames = list(next(iter(merged.values())).keys())
    with open(results_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for key in sorted(merged, key=lambda k: (int(k[0]), float(k[1]))):
            w.writerow(merged[key])

    print(f"\n[done] total wall time {hms(time.monotonic() - t0)}")
    print(f"[done] wrote {results_csv}  ({len(merged)} rows)")
    print("       compare directly against results/BL_100.csv — same 25mm dia x "
          "100mm length envelope, no flexure notches. A much smaller bend_angle_deg "
          "here at matching loads is the notches doing their job.")
    print("       visualize both together:  python3 plot.py results/")


if __name__ == "__main__":
    main()
