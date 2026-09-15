# -*- coding: utf-8 -*-
"""Lightweight legacy-VTK reader for mesh validation (no numpy required)."""


def read_vtk_points(path):
    points = []
    with open(path, "r") as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("POINTS"):
            expected = int(line.split()[1])
            i += 1
            while len(points) < expected and i < len(lines):
                vals = lines[i].split()
                for j in range(0, len(vals) - 2, 3):
                    points.append([float(vals[j]), float(vals[j + 1]), float(vals[j + 2])])
                i += 1
            break
        i += 1
    return points


def mesh_stats(path):
    pts = read_vtk_points(path)
    if not pts:
        return {"path": path, "error": "no points found"}

    mins = [min(p[i] for p in pts) for i in range(3)]
    maxs = [max(p[i] for p in pts) for i in range(3)]
    extents = [maxs[i] - mins[i] for i in range(3)]
    backbone = extents.index(max(extents))

    return {
        "path": path,
        "n_points": len(pts),
        "mins": mins,
        "maxs": maxs,
        "extents": extents,
        "backbone_axis": ["X", "Y", "Z"][backbone],
    }


def print_mesh_stats(path, label=None):
    s = mesh_stats(path)
    tag = label or s["path"]
    if "error" in s:
        print(f"[MESH] {tag}: {s['error']}")
        return s
    print(f"[MESH] {tag}: {s['n_points']} nodes, "
          f"extents X={s['extents'][0]:.2f} Y={s['extents'][1]:.2f} Z={s['extents'][2]:.2f}, "
          f"backbone={s['backbone_axis']}")
    return s


def compare_meshes(reference_vtk, candidate_vtk, extent_tol=0.05):
    ref = mesh_stats(reference_vtk)
    cand = mesh_stats(candidate_vtk)
    if "error" in ref or "error" in cand:
        return False, "Could not read one or both meshes"

    ratio = cand["n_points"] / max(ref["n_points"], 1)
    extent_ok = all(
        abs(cand["extents"][i] - ref["extents"][i]) <= extent_tol * max(ref["extents"][i], 1e-9)
        for i in range(3)
    )
    ok = 0.5 <= ratio <= 2.0 and extent_ok
    msg = (f"node ratio={ratio:.2f} (ref={ref['n_points']}, cand={cand['n_points']}), "
           f"extent match={extent_ok}")
    return ok, msg
