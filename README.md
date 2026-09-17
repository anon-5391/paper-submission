# SWEEP: Simulation Workflow for Evaluating Elastomeric Performance

Automated pipeline for designing, meshing and simulating **tendon-driven
continuum robots (TDCRs)** in [SOFA](https://www.sofa-framework.org/), and
extracting bending behaviour versus cable pull.

Given a set of geometric parameters the pipeline:

1. generates a 3-D notched-backbone design (CadQuery → STL + tendon centerline),
2. tetrahedralizes it into an FEM volume mesh (CGAL),
3. simulates cable-driven bending in SOFA, clamping the base and pulling one
   tendon at a time through a load sequence, and
4. logs the deformed centerline plus per-sample bend and constant-curvature
   metrics.

The backbone is a bored cylinder with periodic notch cuts acting as flexure
hinges, with a configurable number of tendons (default 3).

> **Scope of this folder.** This is the pipeline code only — no experimental
> data, simulation results, meshes, renders or plots are included. Running
> any of the commands below against your own SOFA + CadQuery setup will
> generate `stl/`, `vtk/`, `cables/`, `results/`, `logs/` and `design_log.csv`
> locally; none of that is checked in here.

---

## Requirements

- **Linux** (the orchestration is `bash` + `flock` + `xvfb-run`; use WSL2 on Windows).
- **SOFA v25.06**, built with `SofaPython3`, `CGALPlugin`, `STLIB`, `SoftRobots`;
  `runSofa` on `PATH`. (The scene code also runs on SOFA 23.06 — see the
  version note below — but 25.06 is the version this code is developed and
  tested against.)
- **SOFA's Python** (embedded in the SOFA binaries; 3.12 on SOFA 25.06) with
  `numpy` importable from it.
- A **separate CadQuery env** (`$CQ_PYTHON`, Python 3.10/3.11) with `cadquery`
  and `numpy` — see [requirements-cadquery.txt](requirements-cadquery.txt).

### Required SOFA plugins

| Plugin | Used by | For |
|---|---|---|
| `SofaPython3` | every stage | runs the `.py` scenes |
| `CGALPlugin` | `dup.py` (Stage 2) | `MeshGenerationFromPolyhedron` — STL → tetrahedral VTK |
| `STLIB` | `main.py`, `tdcr_model.py` | `stlib3.scene.MainHeader`, `stlib3.physics.deformable.ElasticMaterialObject`, `stlib3.physics.constraints.FixedBox`, and `splib3` |
| `SoftRobots` | `tdcr_model.py` | `softrobots.actuators.PullingCable` (cable/tendon constraints) |

A standard SOFA 25.06 build compiled **with** `CGALPlugin`, `STLIB` and
`SoftRobots` provides all of the above.

### Two separate Python interpreters — do not mix them

1. **SOFA's Python** (runs inside `runSofa`) — needs `numpy` importable from
   it. `Sofa.Core`, `Sofa.Simulation`, `stlib3`, `splib3`, `softrobots` come
   from the plugins above, not from pip.
2. **CadQuery env** (`$CQ_PYTHON`) — Stage 1 CAD generation
   (`generate_design.py` → `caddesign.py`). Python 3.10/3.11 with `cadquery`
   and `numpy`. `sweep_cli.py`, `mesh_fidelity.py` and `sweep_check.py` also
   run fine under this env (or any plain `python3` ≥ 3.8 — they're
   dependency-free beyond `numpy` for the latter two).

```bash
conda create -n cq python=3.11
conda activate cq
pip install -r requirements-cadquery.txt
```

### SOFA version note

The scene code targets **SOFA 25.06**, which dropped `runSofa`'s `--batch`
flag (the UI is now chosen with `-g`, and `-n` is a batch-GUI-only step cap)
and moved `VTKExporter` into `Sofa.Component.IO.Mesh` (the old `SofaExporter`
plugin no longer exists). It also embeds Python 3.12. If you're still on
SOFA 23.06, `run_one.sh`'s `runSofa` invocations will need the older
`--batch` flag restored.

### Scene units

mm–kg–s. Gravity `9800` mm/s². `1 MPa = 1000` scene units
(`YOUNG_MODULUS = 56000` ⇒ 56 MPa). `1 N = 1000` scene units
(`SCENE_FORCE_PER_NEWTON`), so the default `CABLE_PULL_LEVELS` of
`980 … 9800` = 0.98 … 9.8 N.

---

## Setup

```bash
export CQ_PYTHON=/abs/path/to/cadquery-env/bin/python
export PATH="/abs/path/to/SOFA/bin:$PATH"     # so runSofa resolves

# sanity check
runSofa --help
runSofa -l SofaPython3 -l CGALPlugin --help      # CGALPlugin present
$CQ_PYTHON -c "import cadquery, numpy; print('cq env OK')"
```

`CQ_PYTHON` has no fallback default — `run_one.sh` and `sweep_cli.py` both
fail with a clear error (exit 11) if it isn't exported to an executable
`python`.

---

## Pipeline architecture

```
STAGE 1: CAD                STAGE 2: MESH               STAGE 3: SIM
──────────────              ──────────────              ─────────────
generate_design.py          dup.py                      main.py  (runSofa)
  → caddesign.py            (runSofa + CGALPlugin)        │
  ($CQ_PYTHON)                  │                         ├─ tdcr_model.TDCR(): load mesh,
    │                           │                         │   clamp base, attach cables,
    ├─ stl/<id>.stl            └─ vtk/<id>.vtk            │   build embedded spine probe
    ├─ stl/<id>.dims.json         (tetrahedral FEM mesh)  ├─ auto_controller: pull one cable
    └─ cables/<id>.cable1.json                            │   through the load sequence, settle
                                                          └─ results/<id>.csv
```

- **CAD** — `caddesign.py` builds the parametric notched backbone; `generate_design.py`
  is its CLI wrapper and also writes a `dims.json` geometry sidecar and the first
  tendon centerline, spanning the full part height (base → tip).
- **Mesh** — `dup.py` runs inside `runSofa` and tetrahedralizes the STL with
  CGAL, tuned by the `CGAL_*` constants in `pipeline_config.py` (overridable per
  run via `TDCR_CGAL_*` env vars).
- **Sim** — `tdcr_model.py` builds the scene (`stlib3.ElasticMaterialObject`
  corotational tets, `MeshMatrixMass`, `FixedBox` base clamp, `softrobots.PullingCable`
  per tendon). `auto_controller.py` pulls **one** cable (selected by
  `TDCR_CABLE_INDEX`) through `CABLE_PULL_LEVELS`, detects settle, and logs the
  deformed centerline.
- **Centerline tracking** — `bending_utils.SpineTracker` reads an *embedded probe*:
  a ring of points at each axial station, bound into the tet mesh with a
  `BarycentricMapping`, so the centerline is made of true material points (no
  rest-slab centroid bias). Built by `tdcr_model._build_spine_probe()`.

`run_one.sh` chains all three stages for one design; `sweep_cli.py` chains many.

---

## Run one design

```bash
bash run_one.sh <id> [inner_dia disc_dia offset outer_dia length count tendon_dist]
# defaults:              14        30       2      25        100    3     9.5

bash run_one.sh BL_100 14 30 2 25 100 3 9.5      # 100 mm backbone, 3 tendons
bash run_one.sh BL_100_4C 14 30 2 25 100 4 9.5   # same geometry, 4 tendons
```

`count` sets the number of `PullingCable`s attached at simulation time
(`TDCR_TENDON_COUNT`) — `tdcr_model.py` rotates the single cable centerline into
that many evenly-spaced tendons. It is **not** a CAD parameter; the backbone
geometry does not depend on it.

| Env var | Effect |
|---|---|
| `TDCR_HEADLESS=1` | wrap `runSofa` in `xvfb-run` (not usually needed on SOFA 25.06's batch GUI) |
| `TDCR_GUI=imgui` | open the live SOFA GUI for the sim stage instead of batch, to watch a pull |
| `SKIP_MESH=1` | reuse the existing VTK, re-run sim only |
| `SKIP_SIM=1` | CAD + mesh only |
| `FORCE_CAD=1` | regenerate the STL even if it exists |
| `TDCR_CABLE_INDEX=0` | pull cable 1 (0-indexed — this is what `sweep_cli.py` sets per task) |
| `TDCR_TENSION_LEVELS` | comma list of scene-force pull targets, overrides `CABLE_PULL_LEVELS` for this run |
| `TDCR_SWEEP_CSV` | override the output CSV path (defaults to `results/<id>.csv`) |
| `DESIGN_LOG_CSV` | override the path of the master design log |

Exit codes: `1` CAD failed, `2` mesh failed, `3` sim failed, `10` `runSofa` not
on `PATH`, `11` `CQ_PYTHON` not executable.

---

## Parameter sweeps — `sweep_cli.py`

```bash
python3 sweep_cli.py                                   # interactive menu

# scripted (non-interactive)
python3 sweep_cli.py --type cad --param disc_dia --values 26,32,38,44,50 --yes

python3 sweep_cli.py --type cad --param backbone_length \
    --range 100 200 --step 12.5 --fixed tendon_count=4 --yes

python3 sweep_cli.py --type cad --param disc_dia --values 26,32,38 \
    --fixed backbone_length=150 --fixed tendon_count=4 \
    --id-suffix _gridBL150 --jobs 6 --yes
```

| Flag | Meaning |
|---|---|
| `--type cad` | required — `cad` (notched `caddesign`) is the only design type |
| `--param NAME` | which CAD parameter to vary — one of `inner_dia` (CD), `disc_dia` (RD), `offset` (OF), `backbone_outer_dia` (BD), `backbone_length` (BL), `tendon_hole_dist` (TD) |
| `--values 30,40,50` | explicit comma list of values |
| `--range START END` | paired with `--step` or `--count` |
| `--step N` / `--count N` | range step size / number of evenly spaced designs |
| `--fixed K=V` | override another fixed parameter (repeatable), e.g. `--fixed tendon_count=4` |
| `--tension-cap N` | drop `CABLE_PULL_LEVELS` entries above `N` newtons (avoids self-folding at high load) |
| `--yes` | skip the confirmation prompt |
| `--jobs N` | run up to `N` **distinct designs** concurrently in Phase 2 (tasks belonging to the *same* design always run sequentially — they'd otherwise race on that design's results CSV) |
| `--phase {all,mesh,sim}` | `mesh` = CAD+mesh only (inspect before simulating); `sim` = simulate meshes from a prior `--phase mesh` run |
| `--id-suffix TAG` | append `TAG` to every design id — needed when sweeping the same `--values` twice under different `--fixed` overrides, so the two runs don't collide on the same `stl/vtk/results` filenames |

Phase 1 runs CAD+mesh for every design (parallel across designs, up to
`--jobs`); Phase 2 runs one `runSofa` per `(load, cable, design)`, in
tension-first order. Results land in `results/<id>.csv` (one file per
design; `--id-suffix` disambiguates two sweeps over the same values).

After results are written, `sweep_cli.py` best-effort-calls `plot.py` and a
post-sweep `sweep_analysis.py` under `$CQ_PYTHON` — neither script is part of
this pipeline-only folder, so every sweep ends with two harmless
`⚠ ... exited 2` warnings after the `Sweep complete` / row-count summary. The
results CSVs themselves are already complete and correct at that point;
build your own analysis on top of `results/<id>.csv` (see the output schema
below), or drop in your own `plot.py`/`sweep_analysis.py` at the repo root.

**Splitting a 2D sweep for speed.** `sweep_cli.py` only varies one CAD
parameter per invocation. To sweep two parameters together (e.g. `disc_dia` ×
`backbone_length`), launch one invocation per value of the second parameter,
each with a distinct `--id-suffix`, in the background:

```bash
for bl in 100 125 150 175 200; do
  nohup env CQ_PYTHON=/abs/path/to/cq/python python3 sweep_cli.py \
    --type cad --param disc_dia --values 26,32,38,44,50 \
    --fixed backbone_length=$bl --fixed tendon_count=4 \
    --id-suffix "_gridBL$bl" --jobs 6 --yes \
    > /tmp/sweep_grid_BL$bl.log 2>&1 &
done
```

Each of the 5 processes above gets its own `--jobs 6`, but concurrency within
one process is capped at however many valid (non-failed) designs it has — the
same-design-never-overlaps rule above applies per process, not globally.

### Custom tension levels from Python

For a one-off sweep at load points the interactive menu / `--tension-cap`
can't express (e.g. specific small loads, not a subset of the default
`CABLE_PULL_LEVELS` grid), call `sweep_cli`'s functions directly instead of
going through the CLI:

```python
import sys; sys.path.insert(0, "/path/to/tdcr_pipeline")
from sweep_cli import build_design_list, run_sweep

design_list = build_design_list("cad", "disc_dia", [67.89],
                                fixed={"tendon_count": 1})
# scene units: 1 N = 1000; label_g = round(N/9.8*1000)
run_sweep(design_list, "cad:disc_dia", tension_levels=[196.0, 392.0, 588.0],
          assume_yes=True, jobs=1, phase="all")
```

The zero-load anchor row is always logged automatically per cable before any
pull — it does not need to be included in `tension_levels`.

---

## Convenience & QA scripts

| Script | What it does |
|---|---|
| `run_finerange_sweep.sh` | Launches a specific 28-job `disc_dia` + `backbone_length` sweep (4-tendon, a 10-step 0–4.5 N force ramp) as a template for a larger backgrounded, multi-job sweep — edit the value lists / tension ramp for your own study. |
| `sweep_status.sh` | Live status dashboard (`--watch [seconds]` to refresh in place) — as written it's wired to one specific older 8-way sweep naming convention (`RD/BD/OF/CD_*_t{3,4}`), so treat it as a template for your own status script rather than a drop-in tool for an arbitrary `sweep_cli.py` run. |
| `mesh_fidelity.py <root>` | Compares each CGAL tet mesh's volume (from `vtk/RD_*.vtk`) against its source STL's volume (`stl/RD_*.stl`). A `mesh/cad` ratio noticeably above 1.0 means CGAL filled in a thin notch gap it couldn't resolve — the simulated robot is stiffer than the one designed. |
| `sweep_check.py <root> [prefix]` | Flags two failure modes per settled row in `results/<prefix>_*.csv`: **CONTACT** (the deformed centerline passes through itself — the scene has no self-collision) and **TIMEOUT** (the per-task log shows the pull hit `SETTLE_MAX_STEPS_PULL` without settling). |
| `test_null_cylinder.py` | Runs a plain, notch-free cylinder through the identical mesh/tendon/load pipeline as a bending baseline — if the notches are what makes a TDCR bend, this design should barely bend at all under the same cable loads. |

---

## Configuration — `pipeline_config.py`

| Constant | Default | Meaning |
|---|---|---|
| `YOUNG_MODULUS` / `POISSON_RATIO` | `56000` (56 MPa) / `0.48` | measured TPU |
| `MATERIAL_DENSITY` / `INFILL_PERCENT` | `1.24e-6` kg/mm³ / `100` | fed to `MeshMatrixMass` |
| `RAYLEIGH_STIFFNESS` / `RAYLEIGH_MASS` | `1.5` / `0.1` | damping |
| `CABLE_VALUE_TYPE` | `"force"` | `"force"` or `"displacement"` |
| `CABLE_PULL_LEVELS` | `980 … 9800` | per-cable pull targets (scene-force units; ÷1000 = N) |
| `SOLVER_TOLERANCE` / `SOLVER_MAX_ITERATIONS` | `1e-6` / `2000` | `GenericConstraintSolver` |
| `SETTLE_TIP_THRESHOLD_MM` / `SETTLE_CONSECUTIVE_STEPS` | `0.15` / `10` | settle test (env-overridable, `TDCR_SETTLE_*`) |
| `SETTLE_MIN_STEPS` / `SETTLE_MAX_STEPS_PULL` | `500` / `6000` | settle wait / hard cap before a pull is force-logged as timed out |
| `SPINE_SEGMENTS` | `25` | points along the tracked centerline (base→tip) |
| `SPINE_PROBE_RING` / `SPINE_PROBE_END_OFFSET_MM` | `8` / `1.5` | embedded-probe ring size / end inset |
| `CGAL_CELL_SIZE` / `CGAL_CELL_RATIO` | `2.0` / `2.0` | mesh resolution — tet size / radius-edge ratio |
| `CGAL_FACET_ANGLE` / `CGAL_FACET_SIZE` / `CGAL_FACET_APPROX` | `25.0` / `1.25` / `0.4` | surface triangulation quality |
| `TENDON_WALL_MARGIN_MM` | `3.0` | tendon radial offset from the outer surface on a `backbone_outer_dia` sweep |

The material constants are for one printed TPU part — re-measure them for
your own material/printer before trusting absolute numbers. Mesh resolution
should be validated for your geometry scale with a convergence study before
trusting results (not included in this folder — see `mesh_fidelity.py` for a
lighter-weight sanity check in the meantime).

---

## Output schema (what the pipeline produces, once you run it)

### `results/<design_id>.csv`

One row per settled `(cable, load)`, plus a first row per cable at zero load
(the `tension_newton = 0` anchor, deformed spine = rest spine):

```
design, backbone_length, cable, tension_newton, displacement_mm, settled, n_spine,
tip_x, tip_y, tip_z,
bend_angle_deg,
arc_length_mm, radius_mm, kappa_mean_permm, kappa_cv, cc_rmse_mm, cc_rmse_norm,
plane_nx, plane_ny, plane_nz,
true_base_x/y/z, true_tip_x/y/z,
rest_x0..rest_z{n-1}, def_x0..def_z{n-1}
```

- `n_spine` = number of probe stations: `SPINE_SEGMENTS` evenly spaced, or the
  design's notch/hourglass positions + 2 end stations if `dims.json` supplies them.
- `rest_*` is the undeformed centerline (identical for every row of a design);
  `def_*` is the deformed centerline.
- `settled` — `1` if the pull's tip motion dropped below
  `SETTLE_TIP_THRESHOLD_MM` for `SETTLE_CONSECUTIVE_STEPS` in a row before
  `SETTLE_MAX_STEPS_PULL`; `0` if it was force-logged at the step cap instead
  (check for `TIMEOUT` in that task's log, or run `sweep_check.py`).
- `bend_angle_deg` — base-to-tip deflection from the PCA rest axis.
- `arc_length_mm`, `radius_mm`, `kappa_mean_permm`, `kappa_cv`, `cc_rmse_mm`,
  `cc_rmse_norm` — constant-curvature diagnostics on the deformed spine.
  `kappa_cv` (coefficient of variation of the pointwise curvature) and
  `cc_rmse_norm` (RMS radial residual to the best-fit circle, ÷ arc length) are
  ≈ 0 for a perfect circular arc.
- `tension_newton` is always the resulting reaction force; in `"displacement"`
  mode it is a readout, `displacement_mm` is the driving input.

### `design_log.csv`

One row appended per `run_one.sh` invocation (concurrent sweep workers
serialize their writes with `flock`):

```
timestamp, design_type, design_id, stl_file, vtk_file, cable_file,
inner_dia, disc_dia, offset, backbone_outer_dia, backbone_dia, backbone_length,
tendon_count, overlap, tendon_radius, hole_dia, central_dia,
num_notches, pitch_mm, cut_height_mm, hourglass_count, stl_size_bytes,
pipeline_status
```

`pipeline_status` progresses `cad_ok → mesh_ok → sim_ok`. `tendon_count` is
the sim-time cable count. `backbone_dia`, `overlap`, `tendon_radius`,
`hole_dia`, `central_dia`, `num_notches`, `pitch_mm`, `cut_height_mm` and
`hourglass_count` are legacy columns kept blank by the current CAD path.

---

## Repository layout

| Path | What it is |
|---|---|
| `caddesign.py`, `generate_design.py` | Stage 1 — notched-backbone CAD |
| `dup.py`, `mesh_utils.py` | Stage 2 — CGAL STL→VTK meshing + VTK stats |
| `tdcr_model.py` | Stage 3 — SOFA scene graph (mesh, clamp, cables, spine probe) |
| `main.py` | Stage 3 — SOFA scene entry point |
| `auto_controller.py` | Stage 3 — per-run controller: pull, settle, log |
| `bending_utils.py` | `SpineTracker` (embedded-probe centerline) + bend/curvature math |
| `pipeline_config.py` | all tunable constants |
| `run_one.sh` | end-to-end runner for one design |
| `sweep_cli.py` | multi-design sweep orchestrator |
| `run_finerange_sweep.sh` | template for a larger backgrounded multi-job sweep |
| `sweep_status.sh` | template for a live sweep-status dashboard |
| `mesh_fidelity.py` | CGAL mesh volume vs CAD STL volume sanity check |
| `sweep_check.py` | post-sweep self-contact / timeout flagging |
| `test_null_cylinder.py` | notch-free bending baseline |
| `requirements-cadquery.txt` | pip packages for the CadQuery env |
| `stl/ vtk/ cables/ results/ logs/`, `design_log.csv` | generated per-design artifacts — created by running the pipeline, not part of this folder |

---

## Citations
 
This pipeline is built on top of the third-party software below. 
 
| Software | Role in this pipeline | Repository |
|---|---|---|
| **SOFA** | FEM simulation engine — Stage 3 (`main.py`, `tdcr_model.py`) | [github.com/sofa-framework/sofa](https://github.com/sofa-framework/sofa) |
| **SofaPython3** | Python 3 scripting bindings for SOFA scenes, used by every stage | [github.com/sofa-framework/SofaPython3](https://github.com/sofa-framework/SofaPython3) |
| **CGALPlugin** | SOFA wrapper exposing `MeshGenerationFromPolyhedron` — Stage 2 STL → tetrahedral VTK (`dup.py`) | [github.com/sofa-framework/CGALPlugin](https://github.com/sofa-framework/CGALPlugin) |
| **CGAL** | Computational-geometry / mesh-generation library underlying `CGALPlugin` | [github.com/CGAL/cgal](https://github.com/CGAL/cgal) |
| **STLIB** (`stlib3`) | Scene-building utilities — `MainHeader`, `ElasticMaterialObject`, `FixedBox`, `splib3` | [github.com/SofaDefrost/STLIB](https://github.com/SofaDefrost/STLIB) |
| **SoftRobots** | `PullingCable` tendon/cable actuator constraint (`tdcr_model.py`) | [github.com/SofaDefrost/SoftRobots](https://github.com/SofaDefrost/SoftRobots) |
| **CadQuery** | Parametric CAD generation of the notched backbone — Stage 1 (`caddesign.py`, `generate_design.py`) | [github.com/CadQuery/cadquery](https://github.com/CadQuery/cadquery) |
| **NumPy** | Array/numeric backend used throughout the pipeline | [github.com/numpy/numpy](https://github.com/numpy/numpy) |

---

## Notes

- **Force vs displacement.** `pipeline_config.py` currently ships in `"force"`
  mode with ten load levels 0.98–9.8 N. Switch `CABLE_VALUE_TYPE` /
  `CABLE_PULL_LEVELS` for displacement-driven pulls.
- **Headless is not much faster.** The bottleneck is the constraint solver
  (CPU-bound), not rendering.
- **`n_spine`** is `SPINE_SEGMENTS` for pipeline-generated designs (the
  `dims.json` written by `generate_design.py` carries no notch positions). Each
  design's CSV is internally consistent; a results CSV whose header predates a
  schema change is discarded and rewritten rather than appended to.
- **Same-design tasks never run concurrently.** Within one `sweep_cli.py`
  process, two `(load, cable)` tasks of the same design are never scheduled
  at the same time — they'd otherwise race writing that design's results CSV.
  `--jobs` buys concurrency *across* designs, not within one.
