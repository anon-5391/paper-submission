# -*- coding: utf-8 -*-
# main.py — runSofa -l SofaPython3 -g batch -n 2000 main.py   (SOFA 25.06)
import sys, os, json
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from stlib3.scene import MainHeader
from tdcr_model import TDCR, loadRequiredPlugins
from auto_controller import CableController
from pipeline_config import GRAVITY, SOLVER_TOLERANCE, SOLVER_MAX_ITERATIONS


def _look_at_quat(eye, target, up):
    """Quaternion (SOFA [x,y,z,w] order) orienting a camera at `eye` to look at
    `target` with the given `up`. SOFA cameras look down local -Z with local +Y up."""
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)

    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    z = -fwd
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.column_stack((x, y, z))

    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return [float(qx), float(qy), float(qz), float(w)]


def createScene(rootNode):
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    vtu_file    = os.environ.get("TDCR_VTK")
    stl_file    = os.environ.get("TDCR_STL")
    design_name = os.environ.get("TDCR_DESIGN_NAME")
    # Nominal backbone length, passed through from the run_*.sh worker. Recorded
    # per CSV row so stale/duplicated result files are self-evident. Falls back to
    # the mesh-measured robot_length below when unset.
    bl_env = os.environ.get("TDCR_BACKBONE_LENGTH")
    backbone_length = float(bl_env) if bl_env else None
    output_csv  = os.environ.get(
        "TDCR_OUTPUT_CSV",
        os.path.join(script_dir, "results", f"{design_name}.csv"),
    )

    mesh_info = {
        "name": design_name,
        "stl":  stl_file,
        "vtk":  vtu_file,
    }

    # Tension override priority:
    #   1. TDCR_TENSION_LEVELS  — comma-separated scene-unit values (sweep_cli sets one per run)
    #   2. CABLE_PULL_LEVELS in pipeline_config.py  (default)
    tension_levels = None
    tl_env = os.environ.get("TDCR_TENSION_LEVELS")
    if tl_env:
        tension_levels = [float(x.strip()) for x in tl_env.split(",") if x.strip()]
        print(f"[MAIN] Using TDCR_TENSION_LEVELS: {tension_levels}")

    # Number of tendons/cables this design was built with (from stl/<id>.dims.json
    # via run_one.sh) so a 4-tendon caddesign gets 4 PullingCables not a hardcoded 3.
    num_cables = int(os.environ.get("TDCR_TENDON_COUNT", "3"))

    # Notch (flex-hinge) and hourglass (secondary relief cut) Y-positions, read from
    # the CAD's dims.json sidecar. Bending concentrates at these features, so the
    # spine is sampled there instead of at evenly-spaced points. None (older/missing
    # dims.json) falls back to SpineTracker's evenly-spaced default.
    notch_positions = None
    if stl_file:
        dims_path = stl_file.replace(".stl", ".dims.json")
        if os.path.exists(dims_path):
            with open(dims_path) as f:
                dims = json.load(f)
            notches    = dims.get("notch_positions") or []
            hourglass  = dims.get("hourglass_positions") or []
            combined   = sorted(notches + hourglass)
            notch_positions = combined if combined else None

    loadRequiredPlugins(rootNode)
    MainHeader(rootNode, gravity=GRAVITY, plugins=["SoftRobots"])
    rootNode.dt = 0.01

    # Optional solid viewport background for paper-figure captures (replaces
    # SOFA's default tiled checkerboard+logo texture) -- set TDCR_RENDER_BG to
    # a "R G B" string in 0..1 (e.g. "0 1 0" for chroma-key green) to enable.
    # Untouched (no BackgroundSetting object added) unless explicitly requested,
    # so the default batch sweep pipeline's rendering is unaffected.
    render_bg = os.environ.get("TDCR_RENDER_BG")
    if render_bg:
        rootNode.addObject("RequiredPlugin", name="Sofa.Component.Setting")
        rootNode.addObject("BackgroundSetting", color=render_bg)

    # Constraint solver pipeline for the cable (force) constraints. No ContactHeader:
    # there is nothing to collide with, and its self-collision detection injects
    # spurious forces that blow up a bending soft body.
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver',
                       tolerance=SOLVER_TOLERANCE,
                       maxIterations=SOLVER_MAX_ITERATIONS)

    cables = []

    # robot_length, tendon_radius and bounds are auto-computed from the mesh;
    # spine_probe is the embedded centerline probe for the controller's SpineTracker.
    tdcr, soft_body, robot_length, tendon_radius, backbone_axis, bounds, spine_probe = TDCR(
        rootNode, cables_ref=cables, mesh_info=mesh_info, num_cables=num_cables,
        spine_stations=notch_positions,
    )

    # Camera bbox matched to the mesh size with padding so the robot fills the viewport.
    pad = robot_length * 0.4
    rootNode.bbox = (f"{bounds[0]-pad} {bounds[2]-pad} {bounds[4]-pad} "
                     f"{bounds[1]+pad} {bounds[3]+pad} {bounds[5]+pad}")

    # SIDE VIEW: look at the robot perpendicular to the backbone with the backbone
    # pointing up, so you see it bend left/right (overrides SOFA's default top view).
    mins = [bounds[0], bounds[2], bounds[4]]
    maxs = [bounds[1], bounds[3], bounds[5]]
    center = [(mins[k] + maxs[k]) / 2 for k in range(3)]
    up = [0.0, 0.0, 0.0]; up[backbone_axis] = 1.0
    cross = [k for k in range(3) if k != backbone_axis]
    eye = list(center)
    eye[cross[1]] += 3.0 * robot_length   # pull camera out to the side

    # SOFA's InteractiveCamera (23.06) takes no `up` attribute — up is implied by
    # `orientation` (a quaternion), which we build from eye/lookAt/up here.
    orientation = _look_at_quat(eye, center, up)
    rootNode.addObject('InteractiveCamera', name='camera',
                       position=eye, lookAt=center, orientation=orientation,
                       distance=3.0 * robot_length)

    print(f"[MAIN] robot_length={robot_length:.2f}mm  "
          f"tendon_radius={tendon_radius:.2f}mm")
    print(f"[CAMERA] side view: eye={[round(v,1) for v in eye]} "
          f"lookAt={[round(v,1) for v in center]} up={up}")

    controller = CableController(
        cables=cables,
        soft_body=soft_body,
        design_name=design_name,
        tendon_radius=tendon_radius,
        robot_length=robot_length,
        backbone_length=backbone_length,
        backbone_axis=backbone_axis,
        tension_levels=tension_levels,
        output_csv=output_csv,
        notch_positions=notch_positions,
        spine_probe=spine_probe,
    )
    soft_body.addObject(controller)
    rootNode.animate = True

    return rootNode
