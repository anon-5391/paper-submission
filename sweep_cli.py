#!/usr/bin/env python3
"""
sweep_cli.py — TDCR (caddesign) parameter sweep launcher.

Runs the interactive parameter collection UI that matches caddesign.py's own
prompts, then drives the full CAD+mesh+sim pipeline for every generated
design. Results log to results/<design_id>.csv, one file per design.
"""

import csv as _csv
import os
import re
import sys
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed, wait, FIRST_COMPLETED
from pathlib import Path

from pipeline_config import CABLE_PULL_LEVELS, CABLE_VALUE_TYPE, SCENE_FORCE_PER_NEWTON, TENDON_WALL_MARGIN_MM

SCRIPT_DIR = Path(__file__).parent.resolve()


def _couple_tendon_to_dia(p, pname):
    """On a backbone-outer-diameter sweep, derive the tendon radial offset to
    keep a fixed wall margin from the outer surface. No-op for other params."""
    p = dict(p)
    if pname == "backbone_outer_dia":
        p["tendon_hole_dist"] = p["backbone_outer_dia"] / 2.0 - TENDON_WALL_MARGIN_MM
    return p


# ── Input helpers ──────────────────────────────────────────────────────────────

def _ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"  {prompt}{suffix}: ").strip()
    return val if val else (str(default) if default is not None else "")

def ask_float(prompt, default=None):
    while True:
        try:
            return float(_ask(prompt, default))
        except ValueError:
            print("    Not a number — try again.")

def ask_int(prompt, default=None, lo=1):
    while True:
        try:
            v = int(_ask(prompt, default))
            if v >= lo:
                return v
            print(f"    Must be at least {lo}.")
        except ValueError:
            print("    Not an integer — try again.")

def ask_choice(prompt, n):
    while True:
        try:
            idx = int(input(f"  {prompt} [1-{n}]: ").strip()) - 1
            if 0 <= idx < n:
                return idx
        except ValueError:
            pass
        print(f"    Enter 1–{n}.")


# ── Value generation ───────────────────────────────────────────────────────────

def _fmt(v):
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s else "0"

def _linspace(start, end, n):
    if n == 1:
        return [float(start)]
    return [round(start + (end - start) * i / (n - 1), 6) for i in range(n)]

def _arange(start, end, step):
    vals, v = [], start
    while v <= end + 1e-9:
        vals.append(round(v, 6))
        v = round(v + step, 10)
    return vals

def collect_range(param_label):
    """Ask start/end/step-or-count and return list of float values."""
    print(f"\n  --- Range for {param_label} ---")
    start = ask_float("  Start value")
    end   = ask_float("  End value")
    if end < start:
        start, end = end, start
    print()
    print("  Generate by:")
    print("  1. Number of designs  (even spacing)")
    print("  2. Step size")
    mode = ask_choice("  Choice", 2)
    print()
    if mode == 0:
        n = ask_int("  Number of designs", default=5)
        return _linspace(start, end, n)
    else:
        step = ask_float("  Step size")
        return _arange(start, end, step)


# ── caddesign parameter collection ────────────────────────────────────────────
# design_list entries: (design_id, args_list, runner_script)

def _validate_cad(inner, disc, offset, backbone_outer, backbone_len, tendon_dist):
    """Geometry-constraint check for a caddesign parameter set. Inlined rather
    than imported from caddesign — sweep_cli runs under the system python, which
    has no cadquery."""
    errors = []
    if inner >= disc:
        errors.append(f"Central Cylinder Diameter ({inner} mm) must be < Rolling Disc Diameter ({disc} mm).")
    if inner >= backbone_outer:
        errors.append(f"Central Cylinder Diameter ({inner} mm) must be < Backbone Outer Diameter ({backbone_outer} mm).")
    if disc <= backbone_outer:
        errors.append(f"Rolling Disc Diameter ({disc} mm) must be > Backbone Outer Diameter ({backbone_outer} mm).")
    outer_r = backbone_outer / 2
    max_tendon = outer_r - 2.0
    if tendon_dist > max_tendon:
        errors.append(f"Tendon Hole Distance ({tendon_dist} mm) must be ≤ {max_tendon:.2f} mm (backbone outer radius {outer_r:.2f} mm − 2 mm).")
    inner_r = inner / 2
    min_tendon = inner_r + 1.0
    if tendon_dist < min_tendon:
        errors.append(f"Tendon Hole Distance ({tendon_dist} mm) must be ≥ {min_tendon:.2f} mm (central cylinder radius {inner_r:.2f} mm + 1 mm).")
    if backbone_len <= disc:
        errors.append(f"Backbone Length ({backbone_len} mm) must be > Rolling Disc Diameter ({disc} mm).")
    return errors


def collect_caddesign():
    # Mirrors caddesign.py's own 6-parameter menu, reusing _validate_cad to
    # reject out-of-range sweeps.
    D = {
        "inner_dia":          14.0,
        "disc_dia":           30.0,
        "offset":              2.0,
        "backbone_outer_dia": 25.0,
        "backbone_length":   100.0,
        "tendon_hole_dist":    9.5,
        "tendon_count":        3,
    }

    print()
    print("  --- Parameter to Vary ---")
    print("  1. Central Cylinder Diameter  (inner_dia)")
    print("  2. Rolling Disc Diameter      (disc_dia)")
    print("  3. Notch Offset               (offset)")
    print("  4. Backbone Diameter          (backbone_outer_dia)")
    print("  5. Backbone Length            (backbone_length)")
    print("  6. Tendon Hole Distance       (tendon_hole_dist)")
    print()
    choice = ask_choice("Choice", 6)

    VARY = [
        ("inner_dia",          "CD", "Central Cylinder Diameter"),
        ("disc_dia",           "RD", "Rolling Disc Diameter"),
        ("offset",             "OF", "Notch Offset"),
        ("backbone_outer_dia", "BD", "Backbone Diameter"),
        ("backbone_length",    "BL", "Backbone Length"),
        ("tendon_hole_dist",   "TD", "Tendon Hole Distance"),
    ]
    pname, prefix, plabel = VARY[choice]
    print(f"\n  → varying: {plabel}\n")

    FIXED_LABELS = {
        "inner_dia":          "Central Cylinder Diameter  [must be < Backbone Dia, mm]",
        "disc_dia":           "Rolling Disc Diameter      [must be > Backbone Dia, mm]",
        "offset":             "Notch Offset               [notch lateral offset, mm]",
        "backbone_outer_dia": "Backbone Diameter          [must be < Rolling Disc Dia, mm]",
        "backbone_length":    "Backbone Length            [must be > Rolling Disc Dia, mm]",
        "tendon_hole_dist":   "Tendon Hole Distance       [from centre, mm]",
        "tendon_count":       "Tendon Count               [number of pulling cables]",
    }
    print("  --- Fixed Parameters ---")
    print()
    for name, label in FIXED_LABELS.items():
        if name == pname:
            continue
        D[name] = ask_float(label, default=D[name])

    values = collect_range(plabel)
    if pname == "backbone_outer_dia":
        print(f"  (outer-dia sweep: tendon_dist derived = outer_dia/2 - {TENDON_WALL_MARGIN_MM} mm per design)")

    # Reject the sweep up front if any generated value violates a geometry
    # constraint — better than silently producing degenerate STLs downstream.
    for v in values:
        p = dict(D)
        p[pname] = v
        p = _couple_tendon_to_dia(p, pname)
        errs = _validate_cad(p["inner_dia"], p["disc_dia"], p["offset"],
                            p["backbone_outer_dia"], p["backbone_length"],
                            p["tendon_hole_dist"])
        if errs:
            print(f"\n  ✗ {plabel}={_fmt(v)} violates constraints:")
            for e in errs:
                print(f"      - {e}")
            print("  Fix the range or fixed parameters and re-run.\n")
            sys.exit(1)

    design_list = []
    for v in values:
        p = dict(D)
        p[pname] = v
        p = _couple_tendon_to_dia(p, pname)
        did  = f"{prefix}_{_fmt(v)}"
        # run_one.sh positional args:
        #   id inner_dia disc_dia offset outer_dia length count tendon_dist
        # inner_dia must be a real value — "0" makes a degenerate .circle(0) bore.
        args = [did,
                str(p["inner_dia"]),
                str(p["disc_dia"]),
                str(p["offset"]),      str(p["backbone_outer_dia"]),
                str(p["backbone_length"]), str(int(p["tendon_count"])),
                str(p["tendon_hole_dist"])]
        design_list.append((did, args, "run_one.sh"))

    return design_list


# ── CSV sort ──────────────────────────────────────────────────────────────────

def sort_csv(csv_path):
    """Sort rows by (numeric suffix of design_id, design_id, cable, tension_newton).
    This groups all rows for a given design value together, e.g. BL_100 then BL_150."""
    if not Path(csv_path).exists():
        return
    with open(csv_path, newline='') as f:
        reader = _csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)
    if not rows:
        return

    def _key(row):
        design = row.get("design", "")
        m = re.search(r'([\d.]+)$', design)
        num = float(m.group(1)) if m else 0.0
        try:
            cable = int(row.get("cable", 0))
        except (ValueError, TypeError):
            cable = 0
        try:
            tension = float(row.get("tension_newton", 0))
        except (ValueError, TypeError):
            tension = 0.0
        return (num, design, cable, tension)

    rows.sort(key=_key)
    with open(csv_path, 'w', newline='') as f:
        writer = _csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  CSV sorted by (parameter value, design, cable, tension).")


# ── Non-interactive design-list builder (CLI / batch) ─────────────────────────
# Fixed defaults, sweepable parameters (param name → id prefix), and arg order
# for run_one.sh. Kept in sync with collect_caddesign above.

CAD_DEFAULTS = {
    "inner_dia": 14.0, "disc_dia": 30.0, "offset": 2.0,
    "backbone_outer_dia": 25.0, "backbone_length": 100.0, "tendon_hole_dist": 9.5,
    "tendon_count": 3,
}
CAD_PREFIX = {
    "inner_dia": "CD", "disc_dia": "RD", "offset": "OF",
    "backbone_outer_dia": "BD", "backbone_length": "BL", "tendon_hole_dist": "TD",
}


def _cad_args(did, p):
    # run_one.sh: id inner_dia disc_dia offset outer_dia length count tendon_dist
    return [did, str(p["inner_dia"]), str(p["disc_dia"]), str(p["offset"]),
            str(p["backbone_outer_dia"]), str(p["backbone_length"]), str(int(p["tendon_count"])),
            str(p["tendon_hole_dist"])]


def build_design_list(design_type, param, values, fixed=None, id_suffix=""):
    """Build a (did, args, runner) design_list without any prompts.

    design_type : "cad" (only type; kept for CLI-argument compatibility).
    param       : parameter name to vary (a key of CAD_DEFAULTS).
    values      : list of floats for that parameter.
    fixed       : optional dict overriding other fixed-parameter defaults.
    id_suffix   : appended to every design id — disambiguates two sweeps that
                  vary the same param/values under different --fixed overrides
                  (they'd otherwise generate identical ids and clobber each
                  other's stl/vtk/results files).
    Raises ValueError on an unknown param or a geometry-constraint violation.
    """
    if design_type != "cad":
        raise ValueError(f"design_type must be 'cad', got {design_type!r}")
    defaults, prefix_map, runner, argf = CAD_DEFAULTS, CAD_PREFIX, "run_one.sh", _cad_args
    if param not in prefix_map:
        raise ValueError(f"cannot vary {param!r}; choose one of {sorted(prefix_map)}")

    D = dict(defaults)
    for k, v in (fixed or {}).items():
        if k not in defaults:
            raise ValueError(f"unknown fixed param {k!r}")
        D[k] = float(v)

    design_list = []
    for v in values:
        p = dict(D)
        p[param] = v
        p = _couple_tendon_to_dia(p, param)
        errs = _validate_cad(p["inner_dia"], p["disc_dia"], p["offset"],
                             p["backbone_outer_dia"], p["backbone_length"],
                             p["tendon_hole_dist"])
        if errs:
            raise ValueError(f"{prefix_map[param]}_{_fmt(v)} violates constraints: "
                             + "; ".join(errs))
        did = f"{prefix_map[param]}_{_fmt(v)}{id_suffix}"
        design_list.append((did, argf(did, p), runner))
    return design_list


# ── Pipeline ──────────────────────────────────────────────────────────────────

def run_cmd(cmd, env):
    return subprocess.call(cmd, env=env, cwd=str(SCRIPT_DIR))


def _run_task(cmd, env, log_path=None):
    """One run_one.sh call. With a log path (parallel mode) its output goes to
    that file instead of interleaving with other workers on the terminal."""
    if log_path is None:
        return run_cmd(cmd, env)
    with open(log_path, "w") as f:
        return subprocess.call(cmd, env=env, cwd=str(SCRIPT_DIR),
                               stdout=f, stderr=subprocess.STDOUT)


def _run_parallel_sims(tasks, jobs, env_for, log_dir):
    """Run Phase-2 tasks (t_scene, t_label, cidx, did, args, runner) in the
    given tension-first order, up to `jobs` SOFA processes at once. Two runs of
    the same design never overlap: they share results/<id>.csv (rewritten by
    the controller's _prune_own_rows) and logs/<id>.sim.log."""
    pending, running, busy = list(tasks), {}, set()
    n, done, t0 = len(tasks), 0, time.monotonic()
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        while pending or running:
            while len(running) < jobs:
                nxt = next((t for t in pending if t[3] not in busy), None)
                if nxt is None:
                    break
                pending.remove(nxt)
                t_scene, t_label, cidx, did, args, runner = nxt
                busy.add(did)
                log = log_dir / f"{did}_c{cidx+1}_{t_label}.log"
                fut = pool.submit(_run_task, ["bash", str(SCRIPT_DIR / runner)] + args,
                                  env_for(t_scene, cidx, did), log)
                running[fut] = (nxt, log)
            finished, _ = wait(running, return_when=FIRST_COMPLETED)
            for fut in finished:
                (t_scene, t_label, cidx, did, _, _), log = running.pop(fut)
                busy.discard(did)
                done += 1
                rc = fut.result()
                timeout = "TIMEOUT" in log.read_text(errors="ignore")
                eta = (time.monotonic() - t0) / done * (n - done)
                print(f"  [{done}/{n}]  {did}  cable={cidx+1}  {t_label}  "
                      f"{'✓' if rc == 0 else '✗ exit=' + str(rc)}"
                      f"{'  [TIMEOUT]' if timeout else ''}  "
                      f"(ETA {eta/3600:.1f} h)", flush=True)


def main():
    print()
    print("════════════════════════════════════════════════════════")
    print("  TDCR Sweep — Parameter Study (caddesign)")
    print("════════════════════════════════════════════════════════")

    design_list = collect_caddesign()
    run_sweep(design_list, "caddesign")


def run_sweep(design_list, label, tension_levels=None, assume_yes=False,
              jobs=1, phase="all"):
    """Execute a prepared sweep: CAD+mesh, tension-first sim, sort, validate.
    Shared by the interactive TUI and the CLI path. `tension_levels` overrides
    CABLE_PULL_LEVELS for this run only; `assume_yes` skips the confirm prompt.
    `jobs` > 1 runs that many designs concurrently; `phase` is "all", "mesh"
    (CAD+mesh only, to inspect meshes first) or "sim" (reuse existing meshes).
    """
    jobs = max(1, int(jobs))
    levels = list(tension_levels) if tension_levels else list(CABLE_PULL_LEVELS)

    results_dir = SCRIPT_DIR / "results"
    design_ids  = [d[0] for d in design_list]
    if CABLE_VALUE_TYPE == "displacement":
        # levels are already raw mm pull targets — no unit conversion.
        pull_labels = [f"{t:g}mm" for t in levels]
        pull_unit   = "pulls"
    else:
        tensions_N  = [round(t / SCENE_FORCE_PER_NEWTON, 4) for t in levels]
        pull_labels = [f"{round(n / 9.8 * 1000)}g" for n in tensions_N]
        pull_unit   = "weights"

    # tendon_count is a fixed parameter (run_one.sh positional #6), so the first
    # entry is representative of the whole sweep.
    N_CABLES = int(float(design_list[0][1][6])) if design_list else 3
    n_total  = len(design_list) * len(pull_labels) * N_CABLES
    print()
    print("════════════════════════════════════════════════════════")
    print(f"  Designs  ({len(design_list)}) : {design_ids}")
    print(f"  Pulls    ({len(pull_labels)})  : {pull_labels}")
    print(f"  Total sims : {len(design_list)} × {len(pull_labels)} {pull_unit} × {N_CABLES} cables = {n_total}")
    print(f"  Output     : results/<design_id>.csv  (one file per design)")
    print(f"  Parallel   : {jobs} run(s) at once   phase: {phase}")
    print("════════════════════════════════════════════════════════")
    if not assume_yes and input("  Run pipeline now? [Y/n]: ").strip().lower() == "n":
        print("  Aborted.")
        return
    print()

    os.makedirs(str(results_dir), exist_ok=True)

    # Clear any existing per-design CSVs from a previous run.
    if phase != "mesh":
        for did, _, runner in design_list:
            p = results_dir / f"{did}.csv"
            if p.exists():
                p.unlink()
                print(f"  Cleared: {p.name}")

    base_env = dict(os.environ)
    base_env.update({
        "FORCE_CAD":  "0",
    })
    # Parallel mode: per-task output goes here instead of the terminal.
    task_log_dir = SCRIPT_DIR / "logs" / "sweep"
    if jobs > 1:
        task_log_dir.mkdir(parents=True, exist_ok=True)

    # ── Phase 1: CAD + Mesh ───────────────────────────────────────────────────
    if phase == "sim":
        # Reuse the meshes from an earlier --phase mesh run.
        mesh_ok = {did for did, _, _ in design_list
                   if (SCRIPT_DIR / "vtk" / f"{did}.vtk").exists()}
        print(f"  PHASE 1 skipped (--phase sim): "
              f"{len(mesh_ok)}/{len(design_list)} meshes found in vtk/")
    else:
        print("════════════════════════════════════════════════════════")
        print("  PHASE 1 — CAD + Mesh")
        print("════════════════════════════════════════════════════════")
        ph1_env              = dict(base_env)
        ph1_env["SKIP_SIM"]  = "1"
        ph1_env["FORCE_CAD"] = "1"
        mesh_ok = set()

        def _mesh(did, args, runner):
            log = task_log_dir / f"{did}.phase1.log" if jobs > 1 else None
            return _run_task(["bash", str(SCRIPT_DIR / runner)] + args, ph1_env, log)

        with ThreadPoolExecutor(max_workers=jobs) as pool:
            futs = {pool.submit(_mesh, *d): d[0] for d in design_list}
            for idx, fut in enumerate(as_completed(futs), 1):
                did, rc = futs[fut], fut.result()
                if rc == 0:
                    mesh_ok.add(did)
                    print(f"  [{idx}/{len(design_list)}]  ✓  {did}", flush=True)
                else:
                    print(f"  [{idx}/{len(design_list)}]  ✗  {did}  (exit {rc}) "
                          f"— will be skipped in sim phase", flush=True)

    if phase == "mesh":
        print(f"\n  Mesh phase done: {len(mesh_ok)}/{len(design_list)} meshed. "
              f"Re-run with --phase sim to simulate.")
        return

    # ── Phase 2: Sim — tension-first, one cable per SOFA run ─────────────────
    print()
    print("════════════════════════════════════════════════════════")
    print("  PHASE 2 — Simulation  (tension-first, one cable per run)")
    print("════════════════════════════════════════════════════════")

    def _sim_env(t_scene, cidx, did):
        e = dict(base_env)
        e["SKIP_MESH"]           = "1"
        e["TDCR_TENSION_LEVELS"] = str(t_scene)
        e["TDCR_CABLE_INDEX"]    = str(cidx)
        e["TDCR_SWEEP_CSV"]      = str(results_dir / f"{did}.csv")
        return e

    if jobs > 1:
        tasks = [(t_scene, t_label, cidx, did, args, runner)
                 for t_scene, t_label in zip(levels, pull_labels)
                 for cidx in range(N_CABLES)
                 for did, args, runner in design_list if did in mesh_ok]
        _run_parallel_sims(tasks, jobs, _sim_env, task_log_dir)
    else:
        n_t = len(levels)
        for tidx, (t_scene, t_label) in enumerate(zip(levels, pull_labels), 1):
            print(f"\n  ── Pull {tidx}/{n_t}: {t_label} ──")
            for cidx in range(N_CABLES):
                print(f"\n    cable {cidx+1}/{N_CABLES}")
                for didx, (did, args, runner) in enumerate(design_list, 1):
                    if did not in mesh_ok:
                        print(f"      [{didx}/{len(design_list)}]  {did}  — SKIPPED (mesh failed)")
                        continue
                    print(f"\n      [{didx}/{len(design_list)}]  {did}  @  {t_label}  cable={cidx+1}")
                    rc = run_cmd(["bash", str(SCRIPT_DIR / runner)] + args,
                                 _sim_env(t_scene, cidx, did))
                    print(f"      {'✓' if rc == 0 else '✗'}  exit={rc}")

    # ── Sort each design's CSV by (cable, tension) ────────────────────────────
    print()
    for did, _, runner in design_list:
        sort_csv(str(results_dir / f"{did}.csv"))

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'═'*56}")
    print(f"  Sweep complete — {label}")
    total_rows = 0
    for did, _, runner in design_list:
        p = results_dir / f"{did}.csv"
        if p.exists():
            with open(p) as f:
                n = sum(1 for _ in f) - 1
            print(f"  {p.name}  →  {n} rows")
            total_rows += n
    print(f"  Total: {total_rows} rows across {len(design_list)} designs")
    print(f"{'═'*56}\n")

    # ── Plots — spine shapes for this sweep's designs ──────────────────────────
    # Run plot.py under CQ_PYTHON (matplotlib is in the cadquery env). Best-effort:
    # a plotting failure must not fail the sweep.
    print(f"{'═'*56}")
    print("  PLOTS")
    print(f"{'═'*56}")
    # CQ_PYTHON must be exported — the python of your CadQuery env (numpy +
    # matplotlib), as in run_one.sh. No fallback path is guessed here.
    cq_python = os.environ.get("CQ_PYTHON", "")
    # Open the interactive window only if there's a display, so a headless batch
    # sweep doesn't hang waiting on a GUI.
    show = ["--show"] if os.environ.get("DISPLAY") else []
    if show:
        print("  Opening interactive viewer (3D ⇄ 2D toggle) — "
              "close the window to finish.\n")
    try:
        rc = subprocess.call(
            ["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", cq_python,
             str(SCRIPT_DIR / "plot.py"), str(results_dir), *show],
            cwd=str(SCRIPT_DIR),
        )
        if rc != 0:
            print(f"  ⚠  plot.py exited {rc} — plots may be incomplete.\n")
        else:
            print(f"  ✓  Plots written to results/plots/\n")
    except Exception as e:
        print(f"  ⚠  Could not run plot.py: {e}\n")

    # ── Post-sweep analysis — 6 tendon quantities (runs AFTER the full sweep) ──
    print(f"{'═'*56}")
    print("  POST-SWEEP ANALYSIS (6 tendon quantities)")
    print(f"{'═'*56}")
    prefix = design_ids[0].split("_")[0] if design_ids else None
    if not prefix:
        print("  ⚠  could not derive sweep prefix from design ids — skipped.\n")
    else:
        try:
            rc = subprocess.call(
                ["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", cq_python,
                 str(SCRIPT_DIR / "sweep_analysis.py"), str(results_dir),
                 "--prefix", prefix],
                cwd=str(SCRIPT_DIR),
            )
            if rc != 0:
                print(f"  ⚠  sweep_analysis.py exited {rc} — analysis incomplete.\n")
        except Exception as e:
            print(f"  ⚠  Could not run sweep_analysis.py: {e}\n")


def _parse_values(args):
    """Values from either --values a,b,c or --range start end (step|count)."""
    if args.values:
        return [float(x) for x in args.values.split(",") if x.strip()]
    start, end = args.range[0], args.range[1]
    if end < start:
        start, end = end, start
    if args.count is not None:
        return _linspace(start, end, args.count)
    if args.step is not None:
        return _arange(start, end, args.step)
    raise SystemExit("  --range needs either --step or --count")


def cli(argv):
    import argparse
    ap = argparse.ArgumentParser(
        prog="sweep_cli.py",
        description="Non-interactive TDCR parameter sweep. Omit all args for the "
                    "interactive menu.")
    ap.add_argument("--type", required=True, choices=["cad"],
                    help="design type (only 'cad' — notched caddesign — is supported)")
    ap.add_argument("--param", required=True,
                    help="parameter to vary (e.g. disc_dia, backbone_length, offset)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--values", help="explicit comma list, e.g. 30,40,50")
    g.add_argument("--range", nargs=2, type=float, metavar=("START", "END"),
                   help="range endpoints; pair with --step or --count")
    ap.add_argument("--step", type=float, help="range step size")
    ap.add_argument("--count", type=int, help="number of evenly spaced designs")
    ap.add_argument("--fixed", action="append", default=[], metavar="K=V",
                    help="override a fixed parameter (repeatable), e.g. --fixed backbone_dia=25")
    ap.add_argument("--tension-cap", type=float, metavar="NEWTON",
                    help="drop tension levels above this many N (avoids self-folding)")
    ap.add_argument("--yes", action="store_true", help="skip the confirm prompt")
    ap.add_argument("--jobs", type=int, default=1,
                    help="SOFA runs in parallel (distinct designs only); default 1")
    ap.add_argument("--phase", choices=["all", "mesh", "sim"], default="all",
                    help="'mesh' = CAD+mesh only; 'sim' = simulate existing meshes")
    ap.add_argument("--id-suffix", default="", metavar="TAG",
                    help="append TAG to every design id (e.g. '_t4') so two "
                         "sweeps over the same --values with different --fixed "
                         "don't collide on the same results/stl/vtk filenames")
    a = ap.parse_args(argv)

    values = _parse_values(a)
    fixed = {}
    for kv in a.fixed:
        if "=" not in kv:
            raise SystemExit(f"  --fixed expects K=V, got {kv!r}")
        k, v = kv.split("=", 1)
        fixed[k.strip()] = v.strip()

    try:
        design_list = build_design_list(a.type, a.param, values, fixed, id_suffix=a.id_suffix)
    except ValueError as e:
        raise SystemExit(f"  ✗ {e}")

    levels = list(CABLE_PULL_LEVELS)
    if a.tension_cap is not None:
        cap_scene = a.tension_cap * SCENE_FORCE_PER_NEWTON
        levels = [t for t in levels if t <= cap_scene + 1e-9]
        if not levels:
            raise SystemExit(f"  ✗ --tension-cap {a.tension_cap} N drops every level")

    run_sweep(design_list, f"{a.type}:{a.param}", tension_levels=levels,
              assume_yes=a.yes, jobs=a.jobs, phase=a.phase)


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            cli(sys.argv[1:])
        else:
            main()
    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
        sys.exit(0)
