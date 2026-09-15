# -*- coding: utf-8 -*-
"""
Shared TDCR pipeline settings — kept in sync with KEYBOARD/tdcr_simulation.py
so manual and automated runs produce comparable bending angles.
"""
import os

# SOFA scene — mm-g-s units. gravity 9800 = 9.8 m/s^2 in mm/s^2; TOTAL_MASS 0.03 = 30 g.
# 1 MPa = 1000 scene units, so 56 MPa = 56000 (tolerance +/-3 MPa = +/-3000).
GRAVITY = [0.0, -9800.0, 0.0]
YOUNG_MODULUS = 56000        # 56 MPa (measured 56 +/- 3 MPa)
POISSON_RATIO = 0.48

# Density-based mass (MeshMatrixMass), integrated over tet volume. Units: kg/mm³.
#   TPU ≈ 1.21e-6, PLA ≈ 1.24e-6, Resin ≈ 1.10e-6 kg/mm³. Match your print material.
MATERIAL_DENSITY = 1.24e-6

# Print infill (0-100). No lattice geometry in the mesh, so infill is approximated
# as a uniform density scaling: X% infill ≈ X% of solid mass for the same volume.
INFILL_PERCENT = 100
MATERIAL_DENSITY_EFFECTIVE = MATERIAL_DENSITY * (INFILL_PERCENT / 100.0)

RAYLEIGH_STIFFNESS = 1.5   # more dissipation; settles large bends faster
RAYLEIGH_MASS = 0.1

# Base clamp — height of the clamped region (y=0..FIX_BASE_HEIGHT_MM).
FIX_BASE_HEIGHT_MM = 3.0
# Cross-section half-width is measured from each mesh's own bounds in tdcr_model.py.
# This factor is a safety margin so boundary nodes at the true CAD radius aren't clipped.
FIXING_BOX_MARGIN_FACTOR = 1.05
FIXING_BASE_COORD = 0.0

# Pull the cable from below the fixed box so the tension reaction is anchored
# beneath the clamp → the free length above bends smoothly and stably.
CABLE_PULL_POINT_OFFSET_MM = -10.0

# On an outer-diameter sweep the tendon radial offset is derived from the backbone
# radius so it stays a fixed wall margin from the outer surface.
#   tendon_dist = outer_dia/2 - margin -> 25/2 - 3.0 = 9.5
TENDON_WALL_MARGIN_MM = 3.0

GRAVITY_MAG = 9.80

# Cable actuation mode: "force" (scene-unit force targets, 1 N = 1000 scene units)
# or "displacement" (raw mm cable-shortening targets, applied directly).
CABLE_VALUE_TYPE = "force"

# Cable pull levels (see CABLE_VALUE_TYPE). 0.98 N -> 980 ... 9.8 N -> 9800.
CABLE_PULL_LEVELS = [980.0,1960,2940, 3820.0, 4900.0,5880,6860,7840,8820,9800]

# Stability cap: max cable length-change allowed per step (mm).
CABLE_MAX_DISP_VARIATION = 1

# ── Constraint solver ─────────────────────────────────────────────────────────
SOLVER_TOLERANCE      = 1e-6
SOLVER_MAX_ITERATIONS = 2000

# ── Convergence-based settling ────────────────────────────────────────────────
# A pull phase settles when the tip moves < SETTLE_TIP_THRESHOLD_MM per step for
# SETTLE_CONSECUTIVE_STEPS steps in a row, after at least SETTLE_MIN_STEPS elapsed.
# Every constant below has an env-var override, all unused (defaults unchanged)
# by the normal batch/quantitative pipeline. It exists for capture_renders.py:
# imgui-GUI rendering steps at only a few FPS (physics + redraw per step,
# instead of batch mode's physics-only), so the same step counts that are fast
# headless can take many real-time minutes on screen. The override loosens
# settle detection for that visual-only capture pass; it never touches
# material properties (YOUNG_MODULUS/RAYLEIGH_STIFFNESS).
SETTLE_TIP_THRESHOLD_MM  = float(os.environ.get("TDCR_SETTLE_TIP_THRESHOLD_MM", "0.15"))
SETTLE_CONSECUTIVE_STEPS = int(os.environ.get("TDCR_SETTLE_CONSECUTIVE_STEPS", "10"))
SETTLE_MIN_STEPS         = int(os.environ.get("TDCR_SETTLE_MIN_STEPS", "500"))
# 600 was tuned for small (few-mm) deflections; large-tension pulls (tens of mm)
# take proportionally longer to damp below the fixed SETTLE_TIP_THRESHOLD_MM floor,
# so the old 500-600 window (only 100 steps to catch a quiet streak) left most
# mid/high-load pulls hitting this cap without ever truly settling — see the
# 89%-timeout rate observed on the 80-250mm backbone_length sweep.
SETTLE_MAX_STEPS_PULL    = int(os.environ.get("TDCR_SETTLE_MAX_STEPS_PULL", "6000"))

# Unit conversion for reporting. Scene is mm-kg-s: 1 N = 1000 kg·mm/s².
SCENE_FORCE_PER_NEWTON = 1000.0

# Backbone spine sampling — points along the tracked centerline, base→tip.
SPINE_SEGMENTS = 25
REST_CAPTURE_FRAME = 5

# Backbone centerline probe (bending_utils.SpineTracker): a ring of
# SPINE_PROBE_RING points is embedded per axial station via BarycentricMapping;
# the spine point is the ring mean. END_OFFSET insets the two end stations.
SPINE_PROBE_RING = 8
SPINE_PROBE_END_OFFSET_MM = 1.5

# CGAL tetrahedralisation.
# Resolution study on BL_100 (2026-07-10): coarse 8.0/5.0 gave bend 62.7° vs 31.3°
# at fine 2.0/1.25 — the old 8mm cell size exceeded BL_100's 2mm hinge offset.
# medium 4.0/2.5 landed within 3.3° of fine, so fine is treated as converged and
# is the default for all designs. A finer 1.0/0.6 tier was tried and reverted: it
# made the solver unusable (~0.1 fps) — a small facet size alone forces a dense
# surface triangulation regardless of cell size, so don't decouple the two.
# CGAL_FACET_APPROX kept at 0.5-era 0.4 per the dup.py TDCR_MESH_REFERENCE study
# (surface-deviation tolerance, independent of the cell/facet SIZE knobs).
# Thin-feature resolution (2mm tendon channel, high-disc_dia notches) is still
# unresolved — needs CGAL local/feature-size refinement, not a global constant.
CGAL_CELL_SIZE    =  2.0
CGAL_CELL_RATIO   =  2.0
CGAL_FACET_ANGLE  = 25.0
CGAL_FACET_SIZE   =  1.25
CGAL_FACET_APPROX =  0.4
