# -*- coding: utf-8 -*-
import numpy as np

# Tolerance (fraction of the backbone's axial span) for isolating the true
# physical end-cap faces in bend_angle_deg — tighter than the shape spine's
# bands so it captures only the actual end-face nodes.
TRUE_ENDPOINT_TOL_FRAC = 0.01


# ── geometry helpers (shared by the sim-time logger and offline analysis) ─────

def rest_tangent(rest):
    """Best-fit straight line through the rest spine points (PCA/SVD), oriented
    base -> tip. Robust to per-band centroid noise — the undeformed backbone is
    a straight rod."""
    centered = rest - rest.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    tangent = vt[0]
    if np.dot(tangent, rest[-1] - rest[0]) < 0:
        tangent = -tangent
    n = np.linalg.norm(tangent)
    return tangent / n if n > 1e-9 else tangent


def bend_angle_deg(rest, spine, true_base=None, true_tip=None):
    """Base-to-tip deflection angle: angle at the base between the rest axis'
    tangent direction and the vector to the deformed tip.

    true_base/true_tip, when given, are the true physical end-cap centroids
    (tighter bands than rest[0]/spine[-1], which sit a few mm inside the real
    ends and add a systematic bias on small bends and sharply-curling tips).
    Falls back to rest[0]/spine[-1] for older CSVs without those columns."""
    base = true_base if true_base is not None else rest[0]
    tip  = true_tip  if true_tip  is not None else spine[-1]
    tangent = rest_tangent(rest)
    def_vec = tip - base
    nd = np.linalg.norm(def_vec)
    if nd < 1e-9:
        return 0.0
    cos_a = np.clip(np.dot(tangent, def_vec) / nd, -1.0, 1.0)
    return float(2 * np.degrees(np.arccos(cos_a)))


def bend_plane_normal(rest, spine):
    """Normal of the tendon's own bending plane: the plane spanned by the rest
    axis u and the tip's lateral drift v off it. Returns [0,0,0] when the
    spine is straight (plane undefined)."""
    base = rest[0]
    u = rest[-1] - rest[0]
    nu = np.linalg.norm(u)
    if nu < 1e-9:
        return [0.0, 0.0, 0.0]
    u = u / nu
    d = spine - base
    lat = d - np.outer(d @ u, u)
    tip_lat = lat[-1]
    nt = np.linalg.norm(tip_lat)
    if nt < 1e-9:
        return [0.0, 0.0, 0.0]
    v = tip_lat / nt
    nrm = np.cross(u, v)
    nn = np.linalg.norm(nrm)
    return [float(c) for c in (nrm / nn if nn > 1e-9 else nrm)]


# ── constant-curvature diagnostics ──────────────────────────────────────────
# One deformed spine (base→tip, (P,3)) in, per-sample metrics out. A perfect
# circular arc gives kappa_cv ≈ 0 and cc_rmse_norm ≈ 0.

def arc_length_coords(spine):
    """Cumulative arc-length s_i (s_0 = 0) from Δs_i = ‖p_{i+1} − p_i‖.
    Returns (s (P,), total_length)."""
    spine = np.asarray(spine, dtype=float)
    seg = np.linalg.norm(np.diff(spine, axis=0), axis=1)      # Δs_i
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return s, float(s[-1])


def menger_curvatures(spine):
    """Pointwise curvature κ_i (1/mm) at each interior point, from the circumcircle
    of (p_{i-1}, p_i, p_{i+1}):  κ_i = 2‖a×b‖ / (‖a‖‖b‖‖c‖) with a = p_i-p_{i-1},
    b = p_{i+1}-p_i, c = p_{i+1}-p_{i-1}. Length-(P-2); 0 where collinear."""
    P = np.asarray(spine, dtype=float)
    if len(P) < 3:
        return np.zeros(0)
    a = P[1:-1] - P[:-2]
    b = P[2:]   - P[1:-1]
    c = P[2:]   - P[:-2]
    num = 2.0 * np.linalg.norm(np.cross(a, b), axis=1)
    den = (np.linalg.norm(a, axis=1) *
           np.linalg.norm(b, axis=1) *
           np.linalg.norm(c, axis=1))
    return np.divide(num, den, out=np.zeros_like(num), where=den > 1e-12)


def fit_circle(spine):
    """Best-fit circle through the near-planar spine: algebraic (Kåsa) fit in the
    plane of its two dominant PCA axes, centre lifted back to 3-D.
    Returns (center_3d, radius, plane_normal); radius inf for a straight spine."""
    P = np.asarray(spine, dtype=float)
    c0 = P.mean(axis=0)
    Q = P - c0
    _, _, vt = np.linalg.svd(Q, full_matrices=False)
    e1, e2, n = vt[0], vt[1], vt[2]
    x, y = Q @ e1, Q @ e2
    # centre (A/2, B/2), R² = C + cx² + cy²  from  x²+y² ≈ A·x + B·y + C
    M = np.column_stack([x, y, np.ones_like(x)])
    sol, *_ = np.linalg.lstsq(M, x**2 + y**2, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r2 = sol[2] + cx**2 + cy**2
    if r2 <= 1e-12:
        return c0, float("inf"), n
    return c0 + cx * e1 + cy * e2, float(np.sqrt(r2)), n


def curvature_constancy(spine):
    """Constant-curvature diagnostics for one deformed spine (base→tip, (P,3)).
    Two tests, each ≈ 0 for a perfect circular arc:
      kappa_cv     = σ_κ / |κ̄|  (CV of the pointwise κ_i, σ_κ sample std ddof=1)
      cc_rmse_norm = sqrt(mean(d_i²)) / L,  d_i = ‖p_i − C‖ − R  (C,R best-fit circle)
    Returns a dict (…_mm / …_permm / kappa_cv / cc_rmse_norm / n_curv_pts);
    kappa_cv and radius_mm are NaN for a straight spine."""
    P = np.asarray(spine, dtype=float)
    _, L = arc_length_coords(P)
    kappa = menger_curvatures(P)
    straight = (len(kappa) == 0) or (float(np.max(kappa)) < 1e-9)

    if straight:
        kbar = float(np.mean(kappa)) if len(kappa) else 0.0
        return {
            "arc_length_mm":    L,
            "kappa_mean_permm": kbar,
            "kappa_std_permm":  0.0,
            "kappa_cv":         float("nan"),
            "radius_mm":        float("nan"),
            "cc_rmse_mm":       0.0,
            "cc_rmse_norm":     0.0,
            "n_curv_pts":       int(len(kappa)),
        }

    kbar = float(np.mean(kappa))
    kstd = float(np.std(kappa, ddof=1)) if len(kappa) > 1 else 0.0
    kcv  = kstd / abs(kbar) if abs(kbar) > 1e-12 else float("nan")

    center, R, _ = fit_circle(P)
    d = np.linalg.norm(P - center, axis=1) - R
    rmse = float(np.sqrt(np.mean(d ** 2)))
    rmse_norm = rmse / L if L > 1e-9 else float("nan")

    return {
        "arc_length_mm":    L,
        "kappa_mean_permm": kbar,
        "kappa_std_permm":  kstd,
        "kappa_cv":         kcv,
        "radius_mm":        R,
        "cc_rmse_mm":       rmse,
        "cc_rmse_norm":     rmse_norm,
        "n_curv_pts":       int(len(kappa)),
    }


class SpineTracker:
    """
    Backbone centerline (points base → tip) tracked through the FEM deformation.

    `probe` given (embedded BarycentricMapping ring set from tdcr_model.TDCR):
    spine point = mean of each station's ring of true material points — no
    rest-slab / mesh-density bias. `probe` None: legacy fallback — FEM nodes
    grouped into fixed rest bands, band node-centroid = spine point.

        st = SpineTracker(soft_body, n_segments=10, probe=spine_probe)
        st.capture_rest(backbone_axis=1)
        st.rest_spine / st.current_spine()   -> (P,3) points, base→tip
    """

    def __init__(self, soft_body, n_segments=10, probe=None):
        self.soft_body   = soft_body
        self._probe      = probe   # {mo, n_ring, stations, axis} or None (legacy mode)
        # Fallback count of evenly-spaced stations (when capture_rest() gets no
        # explicit station_positions).
        self.n_points    = max(int(n_segments), 3)
        self._stations   = None      # explicit station Y-values (e.g. notch positions), if given
        self._axis       = None      # backbone axis index (0=X, 1=Y, 2=Z)
        self.rest_axis   = None      # unit vector along the unbent backbone
        self.rest_base_pt = None     # centroid of the base nodes (rest)
        self.rest_tip_pt  = None     # centroid of the tip nodes (rest)
        self.rest_spine  = None      # (P,3) undeformed spine points, captured once
        self._point_idx  = None      # list of node-index arrays, base→tip order
        self.true_base   = None      # (3,) rest centroid of the true base end-face
        self.true_tip    = None      # (3,) rest centroid of the true tip end-face
        self._true_base_idx = None   # node indices making up the true base end-face
        self._true_tip_idx  = None   # node indices making up the true tip end-face

    # ── geometry helpers ──────────────────────────────────────────────────────

    def _positions(self):
        return np.array(self.soft_body.dofs.position.value)[:, :3]

    def _probe_spine(self):
        """(S,3) centerline: mean of each station ring (points are station-major)."""
        P  = np.array(self._probe["mo"].position.value)[:, :3]
        nr = self._probe["n_ring"]
        return P.reshape(len(P) // nr, nr, 3).mean(axis=1)

    def current_spine(self):
        """Deformed spine now, base→tip. Returns None before capture_rest."""
        if self._probe is not None:
            return self._probe_spine() if self.rest_spine is not None else None
        if self._point_idx is None:
            return None
        pts = self._positions()
        return np.array([pts[idx].mean(axis=0) for idx in self._point_idx])

    # backwards-compat: the last spine point is the physical distal end.
    def tip_point(self):
        """Deformed tip = last spine point (curl-safe distal end)."""
        spine = self.current_spine()
        return None if spine is None else spine[-1]

    def current_true_tip(self):
        """Deformed tip end point — for bend_angle_deg. Probe mode: the last
        station's ring mean; legacy: centroid of the tight tip end-face band."""
        if self._probe is not None:
            return self._probe_spine()[-1] if self.rest_spine is not None else None
        if self._true_tip_idx is None:
            return None
        return self._positions()[self._true_tip_idx].mean(axis=0)

    def current_true_base(self):
        """Deformed base end point (probe: first station ring mean)."""
        if self._probe is not None:
            return self._probe_spine()[0] if self.rest_spine is not None else None
        if self._true_base_idx is None:
            return None
        return self._positions()[self._true_base_idx].mean(axis=0)

    # ── one-time rest capture ─────────────────────────────────────────────────

    def _capture_rest_probe(self, backbone_axis):
        """Rest capture for the embedded-probe centerline."""
        spine = self._probe_spine()
        self._axis = (backbone_axis if backbone_axis is not None
                      else int(np.argmax(spine.max(axis=0) - spine.min(axis=0))))
        cross_axes = [i for i in range(3) if i != self._axis]

        self.rest_spine = spine.copy()
        # cross-axis values at rest are probe/mesh noise, not geometry — zero them
        for ax in cross_axes:
            self.rest_spine[:, ax] = 0.0

        self.true_base    = self.rest_spine[0].copy()
        self.true_tip     = self.rest_spine[-1].copy()
        self.rest_base_pt = self.true_base.copy()
        self.rest_tip_pt  = self.true_tip.copy()
        rest_vec = self.rest_tip_pt - self.rest_base_pt
        self.rest_axis = rest_vec / np.linalg.norm(rest_vec)

        axis_name = ["X", "Y", "Z"][self._axis]
        print(f"[SPINE] backbone axis: {axis_name}  "
              f"rest_axis={np.round(self.rest_axis, 3)}  "
              f"base={np.round(self.true_base, 2)}  tip={np.round(self.true_tip, 2)}  "
              f"spine_points={len(self.rest_spine)} "
              f"(embedded BarycentricMapping probe, {self._probe['n_ring']} pts/ring)  "
              f"(cross-axes {['XYZ'[a] for a in cross_axes]} hard-zeroed at rest)")

    def _build_points(self, pts):
        """Group node indices into cross-section bands along the rest axis.
        Each band is the nodes nearest one station; its centroid is one
        spine point. Empty bands are dropped. Stations are self._stations
        (e.g. notch positions) if set, else n_points evenly-spaced ones."""
        vals = pts[:, self._axis]
        if self._stations is not None and len(self._stations) > 0:
            stations = np.asarray(self._stations, dtype=float)
        else:
            lo, hi = vals.min(), vals.max()
            span = hi - lo
            if span < 1e-9:
                self._point_idx = [np.arange(len(pts))]
                return
            stations = np.linspace(lo, hi, self.n_points)
        nearest = np.abs(vals[:, None] - stations[None, :]).argmin(axis=1)
        groups = [np.where(nearest == k)[0] for k in range(len(stations))]
        self._point_idx = [g for g in groups if len(g) > 0]

    def capture_rest(self, backbone_axis=None, station_positions=None):
        if self._probe is not None:
            self._capture_rest_probe(backbone_axis)   # stations baked into the probe
            return

        self._stations = station_positions
        pts = self._positions()
        extents    = pts.max(axis=0) - pts.min(axis=0)
        self._axis = backbone_axis if backbone_axis is not None else int(np.argmax(extents))
        cross_axes = [i for i in range(3) if i != self._axis]

        self._build_points(pts)
        self.rest_spine = np.array([pts[idx].mean(axis=0) for idx in self._point_idx])

        # At rest the CAD backbone is a straight rod centered on the backbone
        # axis, so any nonzero cross-axis value here is CGAL meshing noise, not
        # geometry. Zero it at rest only — the deformed spine's cross-axis
        # displacement is the real physics being measured.
        for ax in cross_axes:
            self.rest_spine[:, ax] = 0.0

        vals      = pts[:, self._axis]
        span      = vals.max() - vals.min()
        base_mask = vals <= vals.min() + 0.1 * span
        tip_mask  = vals >= vals.max() - 0.1 * span
        self.rest_base_pt = pts[base_mask].mean(axis=0)
        self.rest_tip_pt  = pts[tip_mask].mean(axis=0)
        for ax in cross_axes:
            self.rest_base_pt[ax] = 0.0
            self.rest_tip_pt[ax]  = 0.0

        rest_vec       = self.rest_tip_pt - self.rest_base_pt
        self.rest_axis = rest_vec / np.linalg.norm(rest_vec)

        # True end-faces for bend_angle_deg — a much tighter band than the 10%
        # above (which only feeds the diagnostic print / rest_axis).
        true_tol = max(span * TRUE_ENDPOINT_TOL_FRAC, 1e-6)
        self._true_base_idx = np.where(vals <= vals.min() + true_tol)[0]
        self._true_tip_idx  = np.where(vals >= vals.max() - true_tol)[0]
        self.true_base = pts[self._true_base_idx].mean(axis=0)
        self.true_tip  = pts[self._true_tip_idx].mean(axis=0)
        for ax in cross_axes:
            self.true_base[ax] = 0.0
            self.true_tip[ax]  = 0.0

        station_kind = (f"{len(self._stations)} notch-aligned stations"
                        if self._stations is not None and len(self._stations) > 0
                        else f"{self.n_points} evenly-spaced stations (no notch positions given)")
        axis_name = ["X", "Y", "Z"][self._axis]
        print(f"[SPINE] backbone axis: {axis_name}  "
              f"rest_axis={np.round(self.rest_axis, 3)}  "
              f"base={np.round(self.rest_base_pt, 2)}  "
              f"tip={np.round(self.rest_tip_pt, 2)}  "
              f"spine_points={len(self._point_idx)} ({station_kind})  "
              f"(cross-axes {['XYZ'[a] for a in cross_axes]} hard-zeroed at rest)")
        print(f"[SPINE] true end-faces (used for bend_angle_deg, not the shape spine): "
              f"true_base={np.round(self.true_base, 2)} ({len(self._true_base_idx)} nodes)  "
              f"true_tip={np.round(self.true_tip, 2)} ({len(self._true_tip_idx)} nodes)")

    @property
    def n_spine(self):
        """Number of spine points (probe stations, or built node bands)."""
        if self._probe is not None:
            return 0 if self.rest_spine is None else len(self.rest_spine)
        return 0 if self._point_idx is None else len(self._point_idx)
