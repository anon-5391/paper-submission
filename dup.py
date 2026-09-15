# -*- coding: utf-8 -*-
# dup.py — runSofa -l SofaPython3 -l CGALPlugin -g batch dup.py
#
# Converts every STL in stl/ to a tetrahedral VTK in vtk/ using CGAL.
# Skips files that already have a matching VTK unless TDCR_MESH_FORCE=1.
#
# To match a Gmsh reference mesh, drop the .vtk into vtk/ manually or tune
# CGAL_* values in pipeline_config.py, then delete the old CGAL output.

import Sofa.Simulation
import os
import glob

from pipeline_config import (
    CGAL_CELL_RATIO,
    CGAL_CELL_SIZE,
    CGAL_FACET_ANGLE,
    CGAL_FACET_APPROX,
    CGAL_FACET_SIZE,
)
from mesh_utils import print_mesh_stats


def _cgal(name, default):
    """Per-run override of a CGAL knob via env var (used by mesh_convergence.py)."""
    v = os.environ.get(name)
    return float(v) if v not in (None, "") else default


CGAL_CELL_SIZE    = _cgal("TDCR_CGAL_CELL_SIZE",    CGAL_CELL_SIZE)
CGAL_CELL_RATIO   = _cgal("TDCR_CGAL_CELL_RATIO",   CGAL_CELL_RATIO)
CGAL_FACET_ANGLE  = _cgal("TDCR_CGAL_FACET_ANGLE",  CGAL_FACET_ANGLE)
CGAL_FACET_SIZE   = _cgal("TDCR_CGAL_FACET_SIZE",   CGAL_FACET_SIZE)
CGAL_FACET_APPROX = _cgal("TDCR_CGAL_FACET_APPROX", CGAL_FACET_APPROX)


def createScene(rootNode):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stl_dir    = os.path.join(script_dir, "stl")
    vtk_dir    = os.path.join(script_dir, "vtk")
    os.makedirs(vtk_dir, exist_ok=True)

    force = os.environ.get("TDCR_MESH_FORCE", "0") == "1"
    ref_vtk = os.environ.get("TDCR_MESH_REFERENCE")

    rootNode.addObject('RequiredPlugin', name='CGALPlugin')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')
    # No 'SofaExporter' plugin in SOFA 25.06 — VTKExporter now lives in
    # Sofa.Component.IO.Mesh (required above).

    # Single-file mode: TDCR_STL/TDCR_VTK let run_one.sh target one specific file.
    # This avoids SOFA misinterpreting the stem when the filename contains dots
    # (SOFA treats any embedded "." as a file extension in the VTKExporter).
    target_stl = os.environ.get("TDCR_STL")
    target_vtk = os.environ.get("TDCR_VTK")

    if target_stl and os.path.exists(target_stl):
        stl_files = [target_stl]
        print(f"[DUP] Single-file mode: {target_stl}")
    else:
        stl_files = sorted(glob.glob(os.path.join(stl_dir, "*.stl")))
        print(f"[DUP] Found {len(stl_files)} STL file(s) in {stl_dir}")
    print(f"[DUP] CGAL cellSize={CGAL_CELL_SIZE} facetSize={CGAL_FACET_SIZE}")

    pending = []
    for stl_path in stl_files:
        stem     = os.path.splitext(os.path.basename(stl_path))[0]
        vtk_path = target_vtk if (target_vtk and stl_path == target_stl) \
                   else os.path.join(vtk_dir, stem + ".vtk")

        if os.path.exists(vtk_path) and not force:
            print(f"[DUP] SKIP {stem} — VTK already exists (set TDCR_MESH_FORCE=1 to rebuild)")
            print_mesh_stats(vtk_path, label=stem)
            continue

        # Sanitise the exporter base: replace dots so SOFA doesn't treat an
        # intermediate "." in the design id as the file extension.
        exporter_stem = stem.replace(".", "_")
        exporter_base = os.path.join(vtk_dir, exporter_stem)

        print(f"[DUP] Queued: {os.path.basename(stl_path)} → {os.path.basename(vtk_path)}")
        node = rootNode.addChild(f"convert_{exporter_stem}")
        node.addObject('MeshSTLLoader', name="loader", filename=stl_path)
        node.addObject('MeshGenerationFromPolyhedron', name="gen",
                       inputPoints="@loader.position",
                       inputTriangles="@loader.triangles",
                       drawTetras=False,
                       cellRatio=CGAL_CELL_RATIO,
                       cellSize=CGAL_CELL_SIZE,
                       facetAngle=CGAL_FACET_ANGLE,
                       facetSize=CGAL_FACET_SIZE,
                       facetApproximation=CGAL_FACET_APPROX)
        # 'Mesh' was an alias of MeshTopology, removed in SOFA v24.12.
        node.addObject('MeshTopology', name="outputMesh",
                       position="@gen.outputPoints",
                       tetrahedra="@gen.outputTetras")
        node.addObject('VTKExporter',
                       name="exporter",
                       filename=exporter_base,
                       XMLformat=0,
                       edges=0,
                       tetras=1,
                       exportAtBegin=1)
        pending.append((exporter_base, vtk_path, stem))

    if not pending:
        print("[DUP] Nothing to convert.")
        os._exit(0)

    Sofa.Simulation.init(rootNode)

    for exporter_base, vtk_path, stem in pending:
        exported = exporter_base + "_0.vtu"
        if os.path.exists(exported):
            os.rename(exported, vtk_path)
            print(f"[DUP] Wrote {vtk_path}")
            print_mesh_stats(vtk_path, label=stem)
            if ref_vtk and os.path.exists(ref_vtk):
                from mesh_utils import compare_meshes
                ok, msg = compare_meshes(ref_vtk, vtk_path)
                print(f"[DUP] Reference check ({os.path.basename(ref_vtk)}): {msg}")
                if not ok:
                    print("[DUP] WARNING: CGAL mesh differs from reference — "
                          "adjust CGAL_CELL_SIZE or use the Gmsh VTK directly.")
        else:
            print(f"[DUP] WARNING: expected {exported} not found — CGAL may have failed")

    print("[DUP] Conversion complete.")
    os._exit(0)
