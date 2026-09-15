"""Mesh fidelity gate for an RD (disc_dia) sweep: compare each CGAL tet mesh's
volume with its CAD STL volume. If CGAL cannot resolve the thin notch gaps it
fills them in, which shows up as mesh volume > CAD volume (a stiffer robot
than the one designed). Also prints the analytic notch dimensions."""
import glob, math, os, re, struct, sys
import numpy as np

ROOT = sys.argv[1]
BR, OFFSET = 12.5, 2.0   # backbone outer radius, notch offset (sweep defaults)


def stl_volume(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:      # binary
        a = np.frombuffer(data[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        tri = a["v"].reshape(-1, 3, 3).astype(float)
    else:                              # ascii
        v = [list(map(float, l.split()[1:4])) for l in data.decode().splitlines()
             if l.strip().startswith("vertex")]
        tri = np.array(v).reshape(-1, 3, 3)
    return abs(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()) / 6


def vtk_volume(path):
    lines = open(path).read().split("\n")
    i = next(k for k, l in enumerate(lines) if l.startswith("POINTS"))
    npts = int(lines[i].split()[1])
    vals, k = [], i + 1
    while len(vals) < 3 * npts:
        vals += lines[k].split(); k += 1
    P = np.array(vals[:3 * npts], float).reshape(-1, 3)
    j = next(k for k, l in enumerate(lines) if l.startswith("CELLS"))
    ncell = int(lines[j].split()[1])
    cells = [list(map(int, lines[j + 1 + c].split())) for c in range(ncell)]
    T = np.array([c[1:5] for c in cells if c[0] == 4])
    a, b, c, d = (P[T[:, m]] for m in range(4))
    vol = np.abs(np.einsum("ij,ij->i", b - a, np.cross(c - a, d - a))).sum() / 6
    return npts, len(T), vol


print(f"{'design':>7} {'nodes':>6} {'tets':>6} {'V_cad':>9} {'V_mesh':>9} {'mesh/cad':>8} "
      f"{'notch_h':>7} {'gap@hinge':>9}")
for stl in sorted(glob.glob(os.path.join(ROOT, "stl", "RD_*.stl")),
                  key=lambda p: float(re.search(r"RD_([\d.]+)", p).group(1))):
    did = os.path.basename(stl)[:-4]
    vtk = os.path.join(ROOT, "vtk", did + ".vtk")
    d = float(did.split("_")[1]); r = d / 2
    notch_h = d - 2 * math.sqrt(r * r - BR * BR)
    gap = d - 2 * math.sqrt(r * r - OFFSET * OFFSET)
    vc = stl_volume(stl)
    if not os.path.exists(vtk):
        print(f"{did:>7}  NO MESH   V_cad={vc:9.0f}"); continue
    n, t, vm = vtk_volume(vtk)
    flag = "  <-- notches filled?" if vm / vc > 1.02 else ""
    print(f"{did:>7} {n:6d} {t:6d} {vc:9.0f} {vm:9.0f} {vm/vc:8.3f} {notch_h:7.2f} {gap:9.3f}{flag}")
