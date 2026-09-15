#!/bin/bash
# run_one.sh — generate ONE TDCR design from parameters and test it end-to-end,
# fully automatic: CAD -> mesh -> SOFA sim -> logged bending results.
# Called by sweep_cli.py as the per-design worker; also runnable directly.
#
# Usage (direct):
#   bash run_one.sh <id> [inner_dia] [disc_dia] [offset] [backbone_outer_dia] [backbone_length] [tendon_count]
#
# Usage (from sweep_cli.py):
#   Env vars SKIP_MESH, SKIP_SIM can be set to "1" to skip stages.
#   DESIGN_LOG_CSV overrides the path of the master design log.
#
# Examples:
#   bash run_one.sh demo
#   bash run_one.sh demo 14 30 2 25 100 3
#
# Stages:
#   1.   CAD  : caddesign.create_model -> stl/<id>.stl + cables/<id>.cable1.json  (cadquery env)
#   2.   MESH : dup.py (CGAL)          -> vtk/<id>.vtk                            (runSofa)
#   3.   SIM  : main.py                -> results/<id>.csv                        (runSofa)
#
# Logging:
#   After Stage 1, design parameters + computed geometry are appended to design_log.csv.
#
# Setup (see README.md / REQUIREMENTS.md):
#   export CQ_PYTHON=/path/to/cadquery-env/bin/python
#   export PATH="/path/to/SOFA/bin:$PATH"     # SOFA 23.06 w/ SofaPython3 CGALPlugin STLIB SoftRobots

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── interpreter paths ────────────────────────────────────────────────────────
# CQ_PYTHON must be exported before running this script — the python of your
# own CadQuery env (the env with `cadquery` installed). No fallback is guessed
# here; the check below (exit 11) fails clearly if it's unset or not executable.
CQ_PYTHON="${CQ_PYTHON:-}"
# runSofa is expected on PATH; else set RUNSOFA to its absolute path.
RUNSOFA="runSofa"

# TDCR_HEADLESS=1: wrap runSofa with xvfb-run. Not needed on SOFA 25.06, whose
# batch GUI (-g batch) never opens an OpenGL context.
if [ "${TDCR_HEADLESS:-0}" = "1" ]; then
    RUNSOFA="xvfb-run -a runSofa"
fi

# SOFA 25.06 dropped runSofa's --batch flag: the UI is chosen with -g, and the
# step cap -n is a batch-GUI-only option. TDCR_GUI=imgui opens the SOFA GUI for
# the sim stage so the pull can be watched live (-a starts the animation; the
# controller exits the process once every level is logged).
TDCR_GUI="${TDCR_GUI:-batch}"

# ── parameters (with defaults matching caddesign.py) ─────────────────────────
ID="${1:?usage: bash run_one.sh <id> [inner_dia disc_dia offset outer_dia length count tendon_dist]}"
INNER_DIA="${2:-14}"
DISC_DIA="${3:-30}"
OFFSET="${4:-2}"
OUTER_DIA="${5:-25}"
LENGTH="${6:-100}"
TENDON_COUNT="${7:-3}"
TENDON_DIST="${8:-9.5}"

# ── pipeline stage skip flags ─────────────────────────────────────────────────
SKIP_MESH="${SKIP_MESH:-0}"
SKIP_SIM="${SKIP_SIM:-0}"

# Set FORCE_CAD=1 to always regenerate the STL even if it already exists.
# Default: skip CAD if both STL and cable JSON are already present — mirrors
# dup.py's behaviour of skipping mesh conversion when the VTK already exists.
FORCE_CAD="${FORCE_CAD:-0}"

# Steps budget — one generous flat cap covering the worst case (every
# CABLE_PULL_LEVELS level pulled in one process at SETTLE_MAX_STEPS_PULL each).
SOFA_N_STEPS=10000

# ── paths for this design ────────────────────────────────────────────────────
STL="$SCRIPT_DIR/stl/$ID.stl"
DIMS_JSON="$SCRIPT_DIR/stl/$ID.dims.json"
VTK="$SCRIPT_DIR/vtk/$ID.vtk"
CABLE="$SCRIPT_DIR/cables/$ID.cable1.json"
OUTPUT_CSV="${TDCR_SWEEP_CSV:-$SCRIPT_DIR/results/$ID.csv}"
LOG_DIR="$SCRIPT_DIR/logs"

# Master design log — one row per design across all runs. Multiple sweep
# workers can write to it concurrently, so DESIGN_LOG_LOCK (flock) serializes
# the read-modify-write status updates below to prevent clobbered rows.
DESIGN_LOG="${DESIGN_LOG_CSV:-$SCRIPT_DIR/design_log.csv}"
DESIGN_LOG_LOCK="$DESIGN_LOG.lock"

mkdir -p "$SCRIPT_DIR/stl" "$SCRIPT_DIR/vtk" "$SCRIPT_DIR/cables" \
         "$SCRIPT_DIR/results" "$LOG_DIR"

# ── preflight: fail fast with a specific message instead of letting a missing
# interpreter/PATH entry cascade into a cryptic downstream "no VTK"/"no CSV" ──
if [ "$SKIP_MESH" = "0" ] || [ "$SKIP_SIM" = "0" ]; then
    if ! command -v runSofa >/dev/null 2>&1; then
        echo "[RUN] FAIL: 'runSofa' not found on PATH."
        echo "[RUN]       Put your SOFA 25.06 build's bin/ on PATH, e.g."
        echo "[RUN]         export PATH=\"/path/to/SOFA/bin:\$PATH\""
        echo "[RUN]       or set RUNSOFA to the full runSofa path at the top of this script."
        exit 10
    fi
fi

echo "================================================================"
echo "[RUN] Design '$ID'"
echo "[RUN]   inner_dia=$INNER_DIA disc_dia=$DISC_DIA offset=$OFFSET"
echo "[RUN]   outer_dia=$OUTER_DIA length=$LENGTH tendon_count=$TENDON_COUNT tendon_dist=$TENDON_DIST"
echo "[RUN]   skip: mesh=$SKIP_MESH sim=$SKIP_SIM"
echo "[RUN]   sofa_steps_cap=$SOFA_N_STEPS  force_cad=$FORCE_CAD"
echo "================================================================"

# ── STAGE 1: CAD (cadquery env) ───────────────────────────────────────────────
echo ""
if [ "$FORCE_CAD" = "0" ] && [ -f "$STL" ] && [ -f "$CABLE" ]; then
    echo "[RUN] === STAGE 1: CAD — SKIPPED (STL+cable exist; set FORCE_CAD=1 to rebuild) ==="
    # Remove stale downstream artifacts, but keep the VTK when SKIP_MESH=1 and
    # keep the shared sweep CSV when TDCR_SWEEP_CSV is set (append-only).
    [ "${SKIP_MESH:-0}" = "0" ] && rm -f "$VTK"
    [ -z "${TDCR_SWEEP_CSV:-}" ]  && rm -f "$OUTPUT_CSV"
else
    # Full regeneration: wipe all artifacts for this design, except the shared
    # sweep CSV when TDCR_SWEEP_CSV is set (would clobber rows from this sweep).
    rm -f "$STL" "$DIMS_JSON" "$VTK" "$CABLE"
    [ -z "${TDCR_SWEEP_CSV:-}" ] && rm -f "$OUTPUT_CSV"

    if [ ! -x "$CQ_PYTHON" ]; then
        echo "[RUN] FAIL: CQ_PYTHON is not set to an executable file: $CQ_PYTHON"
        echo "[RUN]       export CQ_PYTHON=/path/to/your/cadquery-env/bin/python before running."
        exit 11
    fi

    echo "[RUN] === STAGE 1: CAD (STL + cable + dims) ==="
    # Tendon count is a sim-time parameter (TDCR_TENDON_COUNT), not a CAD arg.
    env -u PYTHONPATH -u PYTHONHOME "$CQ_PYTHON" "$SCRIPT_DIR/generate_design.py" \
        --id "$ID" \
        --inner-dia "$INNER_DIA" --disc-dia "$DISC_DIA" --offset "$OFFSET" \
        --backbone-outer-dia "$OUTER_DIA" --backbone-length "$LENGTH" \
        --tendon-dist "$TENDON_DIST" \
        --out-stl "$STL" --out-cable "$CABLE" 2>&1 | tee "$LOG_DIR/$ID.cad.log"

    if [ ! -f "$STL" ] || [ ! -f "$CABLE" ]; then
        echo "[RUN] FAIL: Stage 1 (CAD) did not produce STL+cable for '$ID'"; exit 1
    fi
fi

# ── LOG: append one row to design_log.csv ─────────────────────────────────────
TIMESTAMP="$(date '+%Y-%m-%d %H:%M:%S')"

if [ -f "$DIMS_JSON" ]; then
    # Extract computed geometry from the dims sidecar written by generate_design.py
    flock "$DESIGN_LOG_LOCK" python3 - <<PYEOF
import json, csv, os, sys

dims_path   = "$DIMS_JSON"
log_path    = "$DESIGN_LOG"
timestamp   = "$TIMESTAMP"
vtk_file    = "$VTK"
cable_file  = "$CABLE"

with open(dims_path) as f:
    d = json.load(f)

row = {
    "timestamp":          timestamp,
    "design_type":        "caddesign",
    "design_id":          d["design_id"],
    "stl_file":           d["stl_file"],
    "vtk_file":           os.path.abspath(vtk_file),
    "cable_file":         os.path.abspath(cable_file),
    "inner_dia":          d["inner_dia"],
    "disc_dia":           d["disc_dia"],
    "offset":             d["offset"],
    "backbone_outer_dia": d["backbone_outer_dia"],
    "backbone_dia":       "",
    "backbone_length":    d["backbone_length"],
    "tendon_count":       "$TENDON_COUNT",
    "overlap":            "",
    "tendon_radius":      "",
    "hole_dia":           "",
    "central_dia":        "",
    "num_notches":        d.get("num_notches", ""),
    "pitch_mm":           d.get("pitch_mm", ""),
    "cut_height_mm":      d.get("cut_height_mm", ""),
    "hourglass_count":    d.get("hourglass_count", ""),
    "stl_size_bytes":     d.get("stl_size_bytes", ""),
    "pipeline_status":    "cad_ok",
}

fieldnames = list(row.keys())
write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0

with open(log_path, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    if write_header:
        w.writeheader()
    w.writerow(row)

print(f"[RUN] Logged to {log_path}")
PYEOF
else
    echo "[RUN] WARN: no dims.json found for '$ID' — design_log.csv row skipped"
fi

# Env vars needed by dup.py (single-file mode) and the sim stage.
export TDCR_STL="$STL"
export TDCR_VTK="$VTK"
export TDCR_CABLE_JSON="$CABLE"
export TDCR_DESIGN_NAME="$ID"
# Recorded per CSV row so a stale/duplicated results file is self-evident.
export TDCR_BACKBONE_LENGTH="$LENGTH"
# Tells main.py how many PullingCables to attach (tdcr_model.TDCR's num_cables).
export TDCR_TENDON_COUNT="$TENDON_COUNT"

# ── STAGE 2: MESH (CGAL via dup.py) ──────────────────────────────────────────
if [ "$SKIP_MESH" = "1" ]; then
    echo ""
    echo "[RUN] === STAGE 2: MESH — SKIPPED (SKIP_MESH=1) ==="
else
    echo ""
    echo "[RUN] === STAGE 2: MESH (STL -> VTK) ==="
    TDCR_MESH_FORCE=1 $RUNSOFA -l SofaPython3 -l CGALPlugin -g batch "$SCRIPT_DIR/dup.py" \
        2>&1 | tee "$LOG_DIR/$ID.mesh.log"

    if [ ! -f "$VTK" ]; then
        # Scene (Python) errors first: SOFA 25.06 logs an unrelated
        # "Plugin not found: SofaDistanceGrid.CUDA" on every run, which the
        # plugin check below would otherwise misreport as the cause.
        if grep -q "Unable to completely load the scene" "$LOG_DIR/$ID.mesh.log" 2>/dev/null; then
            echo "[RUN] FAIL: Stage 2 (mesh) — dup.py raised a Python exception:"
            grep -A6 "Python exception" "$LOG_DIR/$ID.mesh.log" | sed 's/^/[RUN]   /'
        elif grep -qiE "unable to load plugin|cannot open shared object|no module named .sofa|Plugin not found: \"?(SofaPython3|CGALPlugin)" \
                "$LOG_DIR/$ID.mesh.log" 2>/dev/null; then
            echo "[RUN] FAIL: Stage 2 (mesh) — SOFA failed to load a required plugin (SofaPython3/CGALPlugin)."
            echo "[RUN]       Check your SOFA build includes CGALPlugin and SOFA_PLUGIN_PATH/PATH is set correctly."
            echo "[RUN]       See $LOG_DIR/$ID.mesh.log"
        else
            echo "[RUN] FAIL: Stage 2 (mesh) produced no VTK for '$ID' — CGAL may have failed."
            echo "[RUN]       Try adjusting CGAL_CELL_SIZE in pipeline_config.py."
        fi
        exit 2
    fi

    # Update pipeline_status in the log
    flock "$DESIGN_LOG_LOCK" python3 - <<PYEOF
import csv, os

log_path  = "$DESIGN_LOG"
design_id = "$ID"

if not os.path.exists(log_path):
    exit()

rows = []
with open(log_path, newline="") as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    for row in reader:
        if row["design_id"] == design_id and row.get("design_type") == "caddesign":
            row["pipeline_status"] = "mesh_ok"
        rows.append(row)

with open(log_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    w.writerows(rows)
PYEOF
fi

# ── STAGE 3: SIM ─────────────────────────────────────────────────────────────
if [ "$SKIP_SIM" = "1" ]; then
    echo ""
    echo "[RUN] === STAGE 3: SIM — SKIPPED (SKIP_SIM=1) ==="
else
    echo ""
    echo "[RUN] === STAGE 3: SIM (pull each cable, bend) ==="
    export TDCR_OUTPUT_CSV="$OUTPUT_CSV"
    if [ "$TDCR_GUI" = "batch" ]; then
        SIM_UI=(-g batch -n "$SOFA_N_STEPS")
    else
        SIM_UI=(-g "$TDCR_GUI" -a)
    fi
    $RUNSOFA -l SofaPython3 -l STLIB -l SoftRobots "${SIM_UI[@]}" "$SCRIPT_DIR/main.py" \
        2>&1 | tee "$LOG_DIR/$ID.sim.log"
fi

# ── RESULT ───────────────────────────────────────────────────────────────────
echo ""
echo "================================================================"

if [ "$SKIP_SIM" = "1" ]; then
    echo "[RUN] DONE '$ID' — simulation skipped."
elif grep -q "All measurements done" "$LOG_DIR/$ID.sim.log" 2>/dev/null; then
    echo "[RUN] DONE '$ID' — pulled and bent every cable."
    echo "[RUN]   Displacement / tip per cable:"
    grep -E "\[CTRL\] (LOGGED|TIMEOUT)" "$LOG_DIR/$ID.sim.log" | sed 's/^/[RUN]   /'
    echo "[RUN] Logs: $LOG_DIR/$ID.{cad,mesh,sim}.log"

    # Mark sim complete in log
    flock "$DESIGN_LOG_LOCK" python3 - <<PYEOF
import csv, os

log_path  = "$DESIGN_LOG"
design_id = "$ID"

if not os.path.exists(log_path):
    exit()

rows = []
with open(log_path, newline="") as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    for row in reader:
        if row["design_id"] == design_id and row.get("design_type") == "caddesign":
            row["pipeline_status"] = "sim_ok"
        rows.append(row)

with open(log_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    w.writerows(rows)
PYEOF
elif grep -q "Unable to completely load the scene" "$LOG_DIR/$ID.sim.log" 2>/dev/null; then
    echo "[RUN] FAIL: Stage 3 (sim) — main.py raised a Python exception:"
    grep -A8 "Python exception" "$LOG_DIR/$ID.sim.log" | sed 's/^/[RUN]   /'
    exit 3
elif grep -qiE "unable to load plugin|cannot open shared object|no module named .sofa|Plugin not found: \"?(SofaPython3|SoftRobots|STLIB)" \
        "$LOG_DIR/$ID.sim.log" 2>/dev/null; then
    echo "[RUN] FAIL: Stage 3 (sim) — SOFA failed to load a required plugin (SofaPython3/SoftRobots)."
    echo "[RUN]       Check your SOFA build includes SoftRobots and SOFA_PLUGIN_PATH/PATH is set correctly."
    echo "[RUN]       See $LOG_DIR/$ID.sim.log"
    exit 3
else
    echo "[RUN] FAIL: Stage 3 (sim) did not finish for '$ID'. See $LOG_DIR/$ID.sim.log"
    exit 3
fi
