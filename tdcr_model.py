# -*- coding: utf-8 -*-
# tdcr_model.py
import Sofa.Core
import Sofa.Simulation
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
import json
import math
import os
import sys
import numpy as np

from pipeline_config import (
    CABLE_MAX_DISP_VARIATION,
    CABLE_PULL_POINT_OFFSET_MM,
    CABLE_VALUE_TYPE,
    FIX_BASE_HEIGHT_MM,
    FIXING_BASE_COORD,
    FIXING_BOX_MARGIN_FACTOR,
    MATERIAL_DENSITY_EFFECTIVE,
    RAYLEIGH_MASS,
    RAYLEIGH_STIFFNESS,
    POISSON_RATIO,
    SOLVER_MAX_ITERATIONS,
    SOLVER_TOLERANCE,
    SPINE_PROBE_END_OFFSET_MM,
    SPINE_PROBE_RING,
    SPINE_SEGMENTS,
    YOUNG_MODULUS,
)

# One bright, distinct colour per cable so they're identifiable in the live
# SOFA GUI (same palette style as view_meshes.py's static inspector).
CABLE_VIS_COLORS = [
    [0.95, 0.10, 0.10, 1.0],   # red
    [0.95, 0.85, 0.05, 1.0],   # yellow
    [0.95, 0.10, 0.85, 1.0],   # magenta
    [0.10, 0.85, 0.95, 1.0],   # cyan
    [0.20, 0.90, 0.20, 1.0],   # green
    [0.60, 0.35, 0.95, 1.0],   # violet
]


# ── Mesh geometry ─────────────────────────────────────────────────────────────

def get_mesh_points(vtu_path):
    """Read the node coordinates from a legacy-VTK / VTU file as an (N,3) array."""
    points = []
    reading_points = False
    count = 0
    expected = 0
    with open(vtu_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("POINTS"):
                expected = int(line.split()[1])
                reading_points = True
                continue
            if reading_points:
                vals = line.split()
                if len(vals) >= 3:
                    points.append([float(vals[0]), float(vals[1]), float(vals[2])])
                    count += 1
                    if count >= expected:
                        break
    return np.array(points)


def get_mesh_bounds(vtu_path):
    """Read mesh bounds from VTU file."""
    pts = get_mesh_points(vtu_path)
    # Cast to native float: SOFA's Data-field string parser can't read NumPy>=2.0
    # scalar reprs ("np.float64(9.5)"), which silently empties fields like
    # position/pullPoint/box and crashes downstream.
    return (float(pts[:,0].min()), float(pts[:,0].max()),
            float(pts[:,1].min()), float(pts[:,1].max()),
            float(pts[:,2].min()), float(pts[:,2].max()))


def detect_backbone_axis(x_min, x_max, y_min, y_max, z_min, z_max):
    """Detect which axis is the backbone (longest extent)."""
    extents = [x_max-x_min, y_max-y_min, z_max-z_min]
    axis = int(np.argmax(extents))
    cross_axes = [i for i in range(3) if i != axis]
    print(f"[GEOM] Extents — X:{extents[0]:.2f}  Y:{extents[1]:.2f}  Z:{extents[2]:.2f}")
    print(f"[GEOM] Backbone axis detected: {'XYZ'[axis]}")
    print(f"[GEOM] Cross axes: {'XYZ'[cross_axes[0]]}, {'XYZ'[cross_axes[1]]}")
    return axis, cross_axes


# ── Cable generation ──────────────────────────────────────────────────────────

def load_cable_jsons(cable_dir, backbone_axis, num_cables=3):
    """Prefer cab1/2/3.json (same as KEYBOARD manual runs)."""
    paths = [os.path.join(cable_dir, f"cab{i}.json") for i in range(1, num_cables + 1)]
    if all(os.path.exists(p) for p in paths):
        cables = []
        for p in paths:
            with open(p) as f:
                cables.append(json.load(f))
        for i in range(len(cables)):
            cables[i] = clip_to_body(cables[i], backbone_axis)
        print(f"[CABLE] Loaded {num_cables} cable JSON files from {cable_dir}")
        return cables
    return None


def generate_cables_from_base(cable1_points, cross_axes, num_cables=3, centroid=None,
                              start_angle_deg=0.0):
    """
    Generate num_cables cables evenly spaced around the backbone axis. Cable 1
    keeps its position rotated by start_angle_deg; cables 2,3,... are further
    rotations of it about the cross-section centroid, in the plane spanned by
    cross_axes, so every tendon stays at the same radius. cross_axes is
    re-derived from the cable's own geometry in case the cable file's backbone
    axis differs from the mesh's detected one.

    start_angle_deg lets a design line one tendon up with the notch's flex
    axis (at 90 deg from cable 1's default 0 deg position) — see caddesign.py's
    notch cut, which is only 2-fold symmetric (thin wall at 90/270 deg), so it
    can never align with all of a 3- or 4-fold tendon pattern at once. For
    num_cables=3 this puts exactly one tendon on the notch (0/3 -> 1/3
    aligned); num_cables=4 is left at 0 deg since it already gets 2/4 aligned
    by default and any nonzero offset would only make that worse.
    """
    pts = np.array(cable1_points, dtype=float)
    spans = pts.max(axis=0) - pts.min(axis=0)
    cable_backbone = int(np.argmax(spans))
    derived_cross = [k for k in range(3) if k != cable_backbone]
    if list(derived_cross) != list(cross_axes):
        print(f"[CABLE] cross-axis override: mesh said {cross_axes}, cable varies "
              f"along {'XYZ'[cable_backbone]} -> using cross axes {derived_cross}")
        cross_axes = derived_cross

    a0, a1 = cross_axes
    c0 = centroid[a0] if centroid is not None else 0.0
    c1 = centroid[a1] if centroid is not None else 0.0
    cables = []
    for i in range(num_cables):
        angle_rad = math.radians(start_angle_deg + i * 360.0 / num_cables)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        cable = []
        for pt in cable1_points:
            # Offset from centroid in the cross-section plane, rotate, add back.
            u = pt[a0] - c0
            v = pt[a1] - c1
            new_pt = [float(x) for x in pt]
            new_pt[a0] = float(c0 + (u * cos_a - v * sin_a))
            new_pt[a1] = float(c1 + (u * sin_a + v * cos_a))
            cable.append(new_pt)
        cables.append(cable)
    return cables


def clip_to_body(points, backbone_axis, base_coord=None):
    """Clip cable points below the robot base — matches KEYBOARD clamp_base()."""
    if base_coord is None:
        base_coord = FIXING_BASE_COORD if backbone_axis == 1 else None
    if base_coord is None:
        return points
    return [
        [float(v) if i != backbone_axis else float(max(v, base_coord)) for i, v in enumerate(pt)]
        for pt in points
    ]


def keyboard_fixing_box(cross_half_mm, backbone_axis=1, y_min=0.0):
    """Clamp box at the actual model base (y_min), sized from cross_half_mm which
    is measured from the mesh's own bounds — a fixed half-width under-clamps the
    base for any backbone larger than the one it was tuned for."""
    if backbone_axis != 1:
        raise ValueError("keyboard_fixing_box only supports Y-backbone (axis=1) for now")
    h = float(cross_half_mm)
    y_min = float(y_min)
    return [-h, y_min, -h, h, y_min + FIX_BASE_HEIGHT_MM, h]


def pull_point_for_cable(cable_pts, backbone_axis, y_min=0.0):
    """Pull point anchored BELOW the fixed box using the actual model base (y_min)."""
    base = float(y_min) + CABLE_PULL_POINT_OFFSET_MM
    if backbone_axis == 1:
        return [float(cable_pts[0][0]), base, float(cable_pts[0][2])]
    pp = [float(v) for v in cable_pts[0]]
    pp[backbone_axis] = base
    return pp


# ── SOFA scene ────────────────────────────────────────────────────────────────

def _build_spine_probe(soft_body, vtk_path, backbone_axis, cross_axes, centroid,
                       y_lo, y_hi, spine_stations):
    """Embedded backbone-centerline probe: a ring of SPINE_PROBE_RING points per
    axial station, bound into the tet mesh with a BarycentricMapping so they move
    as true material points. Spine point = ring mean. Ring radius = median node
    radius over the mid-backbone (always inside the annular wall).
    Returns a dict for SpineTracker(probe=...): {mo, n_ring, stations, axis}."""
    a0, a1 = cross_axes
    all_pts = get_mesh_points(vtk_path)
    node_r = np.sqrt((all_pts[:, a0] - centroid[a0]) ** 2 +
                     (all_pts[:, a1] - centroid[a1]) ** 2)
    axv = all_pts[:, backbone_axis]
    mid = ((axv > y_lo + 0.40 * (y_hi - y_lo)) &
           (axv < y_lo + 0.60 * (y_hi - y_lo)))
    r_track = float(np.median(node_r[mid] if mid.sum() > 10 else node_r))

    off = SPINE_PROBE_END_OFFSET_MM
    if spine_stations:
        interior = sorted(float(s) for s in spine_stations
                          if y_lo + off < float(s) < y_hi - off)
        stations = [y_lo + off] + interior + [y_hi - off]
    else:
        stations = list(np.linspace(y_lo + off, y_hi - off, max(SPINE_SEGMENTS, 3)))

    n_ring = SPINE_PROBE_RING
    ring_pts = []
    for sy in stations:
        for k in range(n_ring):
            ang = 2.0 * math.pi * k / n_ring
            p = [0.0, 0.0, 0.0]
            p[backbone_axis] = float(sy)
            p[a0] = float(centroid[a0] + r_track * math.cos(ang))
            p[a1] = float(centroid[a1] + r_track * math.sin(ang))
            ring_pts.append(p)

    probe = soft_body.addChild("spineProbe")
    probe_mo = probe.addObject('MechanicalObject', name='probeDOFs', template='Vec3d',
                               position=ring_pts,
                               showObject=os.environ.get("TDCR_HIDE_MARKERS") != "1",
                               showObjectScale=2.0,
                               showColor=[0.15, 1.0, 0.35, 1.0])
    # mapForces/mapMasses off: passive monitor, never feeds the FEM solve.
    probe.addObject('BarycentricMapping', input='@../dofs', output='@probeDOFs',
                    mapForces=False, mapMasses=False)
    print(f"[GEOM] Spine probe   : {len(stations)} stations x {n_ring}-pt ring "
          f"@ r={r_track:.2f} mm  ({len(ring_pts)} embedded points, BarycentricMapping)")
    return {"mo": probe_mo, "n_ring": n_ring,
            "stations": [float(s) for s in stations], "axis": backbone_axis}


def TDCR(parentNode, cables_ref, mesh_info, name="TDCR",
         rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
         cable1_path=None, num_cables=3, value_type=None, spine_stations=None):
    # Caller may override the cable mode ("force" vs "displacement");
    # defaults to CABLE_VALUE_TYPE in pipeline_config.py.
    value_type = value_type or CABLE_VALUE_TYPE

    stl_path  = mesh_info['stl']
    vtk_path  = mesh_info['vtk']
    mesh_name = mesh_info['name']

    print(f"\n{'='*50}")
    print(f"Loading mesh : {mesh_name}")
    print(f"  STL: {stl_path}")
    print(f"  VTK: {vtk_path}")

    bounds = get_mesh_bounds(vtk_path)
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    mins = [x_min, y_min, z_min]
    maxs = [x_max, y_max, z_max]

    backbone_axis, cross_axes = detect_backbone_axis(*bounds)
    robot_length = maxs[backbone_axis] - mins[backbone_axis]

    # Center of the cross-section
    centroid = [(mins[i] + maxs[i]) / 2 for i in range(3)]

    # Actual mesh base along the backbone axis — measured, not assumed to be 0.
    y_min_mesh = mins[backbone_axis]

    # Fixing box sized from the mesh's own measured cross-section so it fully
    # covers the base regardless of backbone_outer_dia/backbone_dia/disc_dia.
    if backbone_axis == 1:
        a0, a1 = cross_axes
        cross_half_extent = max(abs(mins[a0]), abs(maxs[a0]), abs(mins[a1]), abs(maxs[a1]))
        cross_half_extent *= FIXING_BOX_MARGIN_FACTOR
        fbox = keyboard_fixing_box(cross_half_extent, backbone_axis, y_min=y_min_mesh)
    else:
        base_end = y_min_mesh + FIX_BASE_HEIGHT_MM
        fbox = [x_min, y_min, z_min, x_max, y_max, z_max]
        fbox[backbone_axis]     = y_min_mesh
        fbox[backbone_axis + 3] = base_end
    print(f"[GEOM] Robot length : {robot_length:.2f} mm")
    print(f"[GEOM] Mesh base    : {y_min_mesh:.2f} mm  (backbone axis {backbone_axis})")
    print(f"[GEOM] Fixing box   : {[round(v,2) for v in fbox]}")
    print(f"[GEOM] Centroid     : {centroid}")

    cable_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cables")
    if cable1_path is None:
        # Pipeline runs pass a per-design cable centerline via TDCR_CABLE_JSON.
        # Fall back to the legacy shared cab1.json for manual/parity runs.
        cable1_path = os.environ.get("TDCR_CABLE_JSON") or os.path.join(cable_dir, "cab1.json")

    # When a per-design cable is supplied, always generate the full set from it
    # (rotate cable1 into num_cables tendons) rather than the shared cab1/2/3.json.
    if os.environ.get("TDCR_CABLE_JSON"):
        all_cables = None
    else:
        all_cables = load_cable_jsons(cable_dir, backbone_axis, num_cables)
    if all_cables is None:
        with open(cable1_path) as f:
            c1 = json.load(f)
        print(f"[CABLE] cab2/cab3 missing — generating from {cable1_path}")
        c1 = clip_to_body(c1, backbone_axis, base_coord=y_min_mesh)
        # 3 tendons: rotate 90 deg so one lands on the notch's flex axis (see
        # generate_cables_from_base docstring). 4 tendons: leave at 0 deg,
        # already the best achievable alignment (2 of 4 land on the notch).
        start_angle_deg = 90.0 if num_cables == 3 else 0.0
        all_cables = generate_cables_from_base(c1, cross_axes, num_cables, centroid=centroid,
                                               start_angle_deg=start_angle_deg)
        for i in range(len(all_cables)):
            all_cables[i] = clip_to_body(all_cables[i], backbone_axis, base_coord=y_min_mesh)
    c1 = all_cables[0]

    cable_names = [f"PullingCable_{i+1}" for i in range(num_cables)]

    pull_points = [pull_point_for_cable(pts, backbone_axis, y_min=y_min_mesh) for pts in all_cables]

    # Tendon radius from cable1 distance to cross-section centroid
    if backbone_axis == 0:
        r = np.sqrt((c1[0][1] - centroid[1])**2 + (c1[0][2] - centroid[2])**2)
    elif backbone_axis == 1:
        r = np.sqrt((c1[0][0] - centroid[0])**2 + (c1[0][2] - centroid[2])**2)
    else:
        r = np.sqrt((c1[0][0] - centroid[0])**2 + (c1[0][1] - centroid[1])**2)
    
    tendon_radius = float(r)
    print(f"[GEOM] Tendon radius: {tendon_radius:.2f} mm")
    
    # Verify cable positions
    print(f"\n[CABLE] Generated {num_cables} cables:")
    for i, cable in enumerate(all_cables):
        print(f"  Cable {i+1}: first point {cable[0]}, last point {cable[-1]}")
    print(f"{'='*50}")

    # Scene graph order matches KEYBOARD/tdcr_simulation.py exactly
    tdcr = parentNode.addChild(name)

    soft_body = ElasticMaterialObject(tdcr,
        volumeMeshFileName=vtk_path,
        surfaceMeshFileName=stl_path,
        youngModulus=YOUNG_MODULUS,
        poissonRatio=POISSON_RATIO,
        totalMass=1e-9,            # near-zero so UniformMass is negligible vs MeshMatrixMass
        surfaceColor=[0.96, 0.87, 0.70, 1.0],
        rotation=rotation,
        translation=translation
    )
    tdcr.addChild(soft_body)

    # TDCR_HYPERELASTIC=1: swap the corotational linear-tet forcefield (locks at
    # high Poisson ratio) for a NeoHookean hyperelastic one.
    if os.environ.get("TDCR_HYPERELASTIC") == "1":
        soft_body.removeObject(soft_body.forcefield)
        mu = YOUNG_MODULUS / (2 * (1 + POISSON_RATIO))
        k_bulk = YOUNG_MODULUS / (3 * (1 - 2 * POISSON_RATIO))
        soft_body.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3', name='geomAlgo')
        soft_body.forcefield = soft_body.addObject('TetrahedronHyperelasticityFEMForceField',
                                                    name='forcefield',
                                                    ParameterSet=f"{mu} {k_bulk}",
                                                    materialName='NeoHookean')
        print(f"[MODEL] Using TetrahedronHyperelasticityFEMForceField (NeoHookean) "
              f"mu={mu:.1f} k={k_bulk:.1f}")
    # TDCR_PARALLEL_FEM=1: same corotational linear-tet forcefield, multi-
    # threaded assembly via the MultiThreading plugin (bundled + autoloaded in
    # this SOFA build). A true drop-in — identical parameters, identical
    # algorithm, just threaded — unlike TDCR_PARALLEL_SOLVER below.
    elif os.environ.get("TDCR_PARALLEL_FEM") == "1":
        soft_body.removeObject(soft_body.forcefield)
        soft_body.forcefield = soft_body.addObject('ParallelTetrahedronFEMForceField',
                                                    template='Vec3', method='large', name='forcefield',
                                                    poissonRatio=POISSON_RATIO, youngModulus=YOUNG_MODULUS)
        print("[MODEL] Using ParallelTetrahedronFEMForceField (multi-threaded FEM assembly)")

    # TDCR_PARALLEL_SOLVER=1: replace the DIRECT SparseLDLSolver with the
    # ITERATIVE ParallelCGLinearSolver. Unlike the forcefield swap above, this
    # changes the numerical method, not just its threading — an iterative
    # solver converges to within iterations/tolerance rather than factorizing
    # exactly, so verify settled bend_angle_deg still agrees before trusting
    # it for real results, not just timing.
    if os.environ.get("TDCR_PARALLEL_SOLVER") == "1":
        soft_body.removeObject(soft_body.solver)
        soft_body.solver = soft_body.addObject('ParallelCGLinearSolver',
                                               name='solver',
                                               template='ParallelCompressedRowSparseMatrixMat3x3d',
                                               iterations=SOLVER_MAX_ITERATIONS,
                                               tolerance=SOLVER_TOLERANCE,
                                               threshold=SOLVER_TOLERANCE)
        print("[MODEL] Using ParallelCGLinearSolver (iterative, multi-threaded) "
              "instead of SparseLDLSolver (direct)")

    # MeshMatrixMass integrates density × tet_volume per element. A node holds
    # only one mass: SOFA 25.06 warns (and silently swaps out) the prefab's
    # 1e-9 kg UniformMass if it is left in, so remove it explicitly first.
    soft_body.removeObject(soft_body.mass)
    soft_body.addObject('MeshMatrixMass',
                        name='mmMass',
                        massDensity=MATERIAL_DENSITY_EFFECTIVE,
                        showGravityCenter=False)

    # ElasticMaterialObject's internal EulerImplicitSolver has zero damping —
    # undamped implicit Euler + cable force explodes. Damp that inner solver
    # directly (a solver on the parent node would be ignored).
    soft_body.integration.rayleighStiffness.value = RAYLEIGH_STIFFNESS
    soft_body.integration.rayleighMass.value = RAYLEIGH_MASS

    FixedBox(soft_body, atPositions=fbox, doVisualization=True)

    for i in range(num_cables):
        print(f"[CABLE] Creating {cable_names[i]} with pull point {pull_points[i]}")
        cable = PullingCable(
            soft_body,
            cable_names[i],
            pullPointLocation=pull_points[i],
            rotation=rotation,
            translation=translation,
            cableGeometry=all_cables[i],
            valueType=value_type,   # "force" or "displacement"
        )
        # Cap per-step cable-length change so a one-shot target eases in over
        # several steps instead of destabilizing the FEM solve.
        cable.CableConstraint.maxDispVariation = CABLE_MAX_DISP_VARIATION
        if value_type == "force":
            # A cable can only pull, never push: keep the tension >= 0.
            cable.CableConstraint.minForce = 0.0
            cable.CableConstraint.maxForce = sys.float_info.max

        # Draw each cable as a coloured base->tip polyline (its MechanicalObject
        # isn't drawn by default), showing the real deformed position. Purely a
        # debug/dev visual aid -- TDCR_HIDE_MARKERS=1 (set by capture_renders.py's
        # --transparent mode) turns it off for a clean geometry-only paper render;
        # doesn't touch the constraint/physics behavior, only the drawing flag.
        _hide_markers = os.environ.get("TDCR_HIDE_MARKERS") == "1"
        cable.MechanicalObject.showObject = not _hide_markers
        cable.MechanicalObject.showObjectScale = 3
        cable.MechanicalObject.showColor = CABLE_VIS_COLORS[i % len(CABLE_VIS_COLORS)]
        edges = [[j, j + 1] for j in range(len(all_cables[i]) - 1)]
        cable.addObject('EdgeSetTopologyContainer', name='cableEdges', edges=edges)
        cable.addObject('EdgeSetTopologyModifier')
        cable.addObject('EdgeSetGeometryAlgorithms', template='Vec3d')

        cables_ref.append(cable)

    # Embedded centerline probe (stations = notch/hourglass positions when given).
    spine_probe = _build_spine_probe(
        soft_body, vtk_path, backbone_axis, cross_axes, centroid,
        y_lo=mins[backbone_axis], y_hi=maxs[backbone_axis],
        spine_stations=spine_stations,
    )

    return (tdcr, soft_body, robot_length, tendon_radius, backbone_axis, bounds,
            spine_probe)


def loadRequiredPlugins(rootNode):
    for plugin in [
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Constraint.Lagrangian.Correction',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.LinearSolver.Iterative',
        'Sofa.Component.Mapping.Linear',
        'Sofa.Component.Mass',
        'Sofa.Component.SolidMechanics.FEM.HyperElastic',
        'MultiThreading',   # ParallelTetrahedronFEMForceField / ParallelCGLinearSolver
                            # (TDCR_PARALLEL_FEM / TDCR_PARALLEL_SOLVER) -- already
                            # autoloaded by this SOFA build, declared explicitly anyway
    ]:
        rootNode.addObject('RequiredPlugin', name=plugin)